/****************************************************************************
 * (C) 2005-2006 - Emmanuel Ackaouy - XenSource Inc.
 ****************************************************************************
 *
 *        File: common/csched_credit.c
 *      Author: Emmanuel Ackaouy
 *
 * Description: Credit-based SMP CPU scheduler
 */

#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <asm/div64.h>
#include <xen/errno.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>
#include <xen/err.h>
#include <xen/rbtree.h>


/*
 * Locking:
 * - Scheduler-lock (a.k.a. runqueue lock):
 *  + is per-runqueue, and there is one runqueue per-cpu;
 *  + serializes all runqueue manipulation operations;
 * - Private data lock (a.k.a. private scheduler lock):
 *  + serializes accesses to the scheduler global state (weight,
 *    credit, balance_credit, etc);
 *  + serializes updates to the domains' scheduling parameters.
 *
 * Ordering is "private lock always comes first":
 *  + if we need both locks, we must acquire the private
 *    scheduler lock for first;
 *  + if we already own a runqueue lock, we must never acquire
 *    the private scheduler lock.
 */

/*
 * Basic constants
 */
#define CSCHED_DEFAULT_WEIGHT       256
#define CSCHED_TICKS_PER_TSLICE     3
/* Default timeslice: 30ms */
#define CSCHED_DEFAULT_TSLICE_MS    30
#define CSCHED_CREDITS_PER_MSEC     10
/* Never set a timer shorter than this value. */
#define CSCHED_MIN_TIMER            XEN_SYSCTL_SCHED_RATELIMIT_MIN


/*
 * Priorities
 */
#define CSCHED_PRI_TS_BOOST      0      /* time-share waking up */
#define CSCHED_PRI_TS_UNDER     -1      /* time-share w/ credits */
#define CSCHED_PRI_TS_OVER      -2      /* time-share w/o credits */
#define CSCHED_PRI_IDLE         -64     /* idle */


/*
 * Flags
 *
 * Note that svc->flags (where these flags live) is protected by an
 * inconsistent set of locks. Therefore atomic-safe bit operations must
 * be used for accessing it.
 */
#define CSCHED_FLAG_VCPU_PARKED    0x0  /* VCPU over capped credits */
#define CSCHED_FLAG_VCPU_YIELD     0x1  /* VCPU yielding */
#define CSCHED_FLAG_VCPU_MIGRATING 0x2  /* VCPU may have moved to a new pcpu */
#define CSCHED_FLAG_VCPU_PINNED    0x4  /* VCPU can run only on 1 pcpu */


/*
 * Useful macros
 */
#define CSCHED_PRIV(_ops)   \
    ((struct csched_private *)((_ops)->sched_data))
#define CSCHED_PCPU(_c)     \
    ((struct csched_pcpu *)per_cpu(schedule_data, _c).sched_priv)
#define CSCHED_VCPU(_vcpu)  ((struct csched_vcpu *) (_vcpu)->sched_priv)
#define CSCHED_DOM(_dom)    ((struct csched_dom *) (_dom)->sched_priv)
#define RUNQ(_cpu)          (&(CSCHED_PCPU(_cpu)->runq))
#define RBROOT(_cpu)        (&(CSCHED_PCPU(_cpu)->runq_root))

/*
 * CSCHED_STATS
 *
 * Manage very basic per-vCPU counters and stats.
 *
 * Useful for debugging live systems. The stats are displayed
 * with runq dumps ('r' on the Xen console).
 */
#ifdef SCHED_STATS

#define CSCHED_STATS

#define SCHED_VCPU_STATS_RESET(_V)                      \
    do                                                  \
    {                                                   \
        memset(&(_V)->stats, 0, sizeof((_V)->stats));   \
    } while ( 0 )

#define SCHED_VCPU_STAT_CRANK(_V, _X)       (((_V)->stats._X)++)

#define SCHED_VCPU_STAT_SET(_V, _X, _Y)     (((_V)->stats._X) = (_Y))

#else /* !SCHED_STATS */

#undef CSCHED_STATS

#define SCHED_VCPU_STATS_RESET(_V)         do {} while ( 0 )
#define SCHED_VCPU_STAT_CRANK(_V, _X)      do {} while ( 0 )
#define SCHED_VCPU_STAT_SET(_V, _X, _Y)    do {} while ( 0 )

#endif /* SCHED_STATS */


/*
 * Credit tracing events ("only" 512 available!). Check
 * include/public/trace.h for more details.
 */
#define TRC_CSCHED_SCHED_TASKLET TRC_SCHED_CLASS_EVT(CSCHED, 1)
#define TRC_CSCHED_ACCOUNT_START TRC_SCHED_CLASS_EVT(CSCHED, 2)
#define TRC_CSCHED_ACCOUNT_STOP  TRC_SCHED_CLASS_EVT(CSCHED, 3)
#define TRC_CSCHED_STOLEN_VCPU   TRC_SCHED_CLASS_EVT(CSCHED, 4)
#define TRC_CSCHED_PICKED_CPU    TRC_SCHED_CLASS_EVT(CSCHED, 5)
#define TRC_CSCHED_TICKLE        TRC_SCHED_CLASS_EVT(CSCHED, 6)
#define TRC_CSCHED_BOOST_START   TRC_SCHED_CLASS_EVT(CSCHED, 7)
#define TRC_CSCHED_BOOST_END     TRC_SCHED_CLASS_EVT(CSCHED, 8)
#define TRC_CSCHED_SCHEDULE      TRC_SCHED_CLASS_EVT(CSCHED, 9)
#define TRC_CSCHED_RATELIMIT     TRC_SCHED_CLASS_EVT(CSCHED, 10)
#define TRC_CSCHED_STEAL_CHECK   TRC_SCHED_CLASS_EVT(CSCHED, 11)

/*
 * Boot parameters
 */
static int __read_mostly sched_credit_tslice_ms = CSCHED_DEFAULT_TSLICE_MS;
integer_param("sched_credit_tslice_ms", sched_credit_tslice_ms);


struct load_weight {
	unsigned long weight, inv_weight;
};

/*
 * Physical CPU
 */
struct csched_pcpu {
    struct list_head runq;
    uint32_t runq_sort_last;

    unsigned int idle_bias;
    unsigned int nr_runnable;

    unsigned int nr_running;

    unsigned int tick;
    struct timer ticker;

    struct rb_root runq_root;
	struct rb_node *rb_leftmost;
	
	struct csched_vcpu *curr, *next, *last, *skip;
    struct load_weight load;        /* load of all vcpu on it */
    unsigned long runnable_weight;

    u64 exec_clock;
	s_time_t min_vruntime;
    s_time_t min_vruntime_copy;

    int cpu;
};

/*
 * Virtual CPU
 */
struct csched_vcpu {
    struct list_head runq_elem;
    struct list_head active_vcpu_elem;

    /* Up-pointers */
    struct csched_dom *sdom;
    struct vcpu *vcpu;

    s_time_t start_time;   /* When we were scheduled (used for credit) */
    unsigned flags;
    int pri;

    atomic_t credit;
    unsigned int residual;

    struct load_weight	load;		/* for load-balancing */
    unsigned long runnable_weight;

	struct rb_node	run_node;
	//struct list_head	group_node;
	unsigned int		on_rq;

	u64			exec_start;
	u64			sum_exec_runtime;
	s_time_t	vruntime;
	u64			prev_sum_exec_runtime;

	u64			nr_migrations;

#ifdef CSCHED_STATS
    struct {
        int credit_last;
        uint32_t credit_incr;
        uint32_t state_active;
        uint32_t state_idle;
        uint32_t migrate_q;
        uint32_t migrate_r;
        uint32_t kicked_away;
    } stats;
#endif
};

/*
 * Domain
 */
struct csched_dom {
    struct list_head active_vcpu;
    struct list_head active_sdom_elem;
    struct domain *dom;
    uint16_t active_vcpu_count;
    uint16_t weight;
    uint16_t cap;

    struct load_weight load;
};

/*
 * System-wide private data
 */
struct csched_private {
    /* lock for the whole pluggable scheduler, nests inside cpupool_lock */
    spinlock_t lock;

    cpumask_var_t idlers;
    cpumask_var_t cpus;
    uint32_t *balance_bias;
    uint32_t runq_sort;
    uint32_t ncpus;

    /* Period of master and tick in milliseconds */
    unsigned int tick_period_us, ticks_per_tslice;
    s_time_t ratelimit, tslice, vcpu_migr_delay;

    struct list_head active_sdom;
    uint32_t weight;
    uint32_t credit;
    int credit_balance;
    unsigned int credits_per_tslice;

    unsigned int master;
    struct timer master_ticker;
};

// treating sched_entity same as task_struct
// since vcpu is the smallest unit and cannot fork
// 
// treating cfs_rq same as rq
// 
// donot consider domain, since vcpu is sched directly by pcpu
#define sched_entity csched_vcpu
#define cfs_rq csched_pcpu

#define rq csched_pcpu
#define task_struct csched_vcpu

/*
 * Nice levels are multiplicative, with a gentle 10% change for every
 * nice level changed. I.e. when a CPU-bound task goes from nice 0 to
 * nice 1, it will get ~10% less CPU time than another CPU-bound task
 * that remained on nice 0.
 *
 * The "10% effect" is relative and cumulative: from _any_ nice level,
 * if you go up 1 level, it's -10% CPU usage, if you go down 1 level
 * it's +10% CPU usage. (to achieve that we use a multiplier of 1.25.
 * If a task goes up by ~10% and another task goes down by ~10% then
 * the relative distance between them is ~25%.)
 */
const int sched_prio_to_weight[40] = {
 /* -20 */     88761,     71755,     56483,     46273,     36291,
 /* -15 */     29154,     23254,     18705,     14949,     11916,
 /* -10 */      9548,      7620,      6100,      4904,      3906,
 /*  -5 */      3121,      2501,      1991,      1586,      1277,
 /*   0 */      1024,       820,       655,       526,       423,
 /*   5 */       335,       272,       215,       172,       137,
 /*  10 */       110,        87,        70,        56,        45,
 /*  15 */        36,        29,        23,        18,        15,
};

/*
 * Inverse (2^32/x) values of the sched_prio_to_weight[] array, precalculated.
 *
 * In cases where the weight does not change often, we can use the
 * precalculated inverse to speed up arithmetics by turning divisions
 * into multiplications:
 */
const u32 sched_prio_to_wmult[40] = {
 /* -20 */     48388,     59856,     76040,     92818,    118348,
 /* -15 */    147320,    184698,    229616,    287308,    360437,
 /* -10 */    449829,    563644,    704093,    875809,   1099582,
 /*  -5 */   1376151,   1717300,   2157191,   2708050,   3363326,
 /*   0 */   4194304,   5237765,   6557202,   8165337,  10153587,
 /*   5 */  12820798,  15790321,  19976592,  24970740,  31350126,
 /*  10 */  39045157,  49367440,  61356676,  76695844,  95443717,
 /*  15 */ 119304647, 148102320, 186737708, 238609294, 286331153,
};

#define DEQUEUE_SLEEP		0x01
#define DEQUEUE_SAVE		0x02 /* matches ENQUEUE_RESTORE */
#define DEQUEUE_MOVE		0x04 /* matches ENQUEUE_MOVE */
#define DEQUEUE_NOCLOCK		0x08 /* matches ENQUEUE_NOCLOCK */

#define ENQUEUE_WAKEUP		0x01
#define ENQUEUE_RESTORE		0x02
#define ENQUEUE_MOVE		0x04
#define ENQUEUE_NOCLOCK		0x08
#define ENQUEUE_HEAD		0x10
#define ENQUEUE_REPLENISH	0x20
#ifdef CONFIG_SMP
#define ENQUEUE_MIGRATED	0x40
#else
#define ENQUEUE_MIGRATED	0x00
#endif

/*
 * Integer metrics need fixed point arithmetic, e.g., sched/fair
 * has a few: load, load_avg, util_avg, freq, and capacity.
 *
 * We define a basic fixed point arithmetic range, and then formalize
 * all these metrics based on that basic range.
 */
# define SCHED_FIXEDPOINT_SHIFT		10

/*
 * Increase resolution of nice-level calculations for 64-bit architectures.
 * The extra resolution improves shares distribution and load balancing of
 * low-weight task groups (eg. nice +19 on an autogroup), deeper taskgroup
 * hierarchies, especially on larger systems. This is not a user-visible change
 * and does not change the user-interface for setting shares/weights.
 *
 * We increase resolution only if we have enough bits to allow this increased
 * resolution (i.e. 64bit). The costs for increasing resolution when 32bit are
 * pretty high and the returns do not justify the increased costs.
 *
 * Really only required when CONFIG_FAIR_GROUP_SCHED is also set, but to
 * increase coverage and consistency always enable it on 64bit platforms.
 */
#ifdef CONFIG_64BIT
# define NICE_0_LOAD_SHIFT	(SCHED_FIXEDPOINT_SHIFT + SCHED_FIXEDPOINT_SHIFT)
# define scale_load(w)		((w) << SCHED_FIXEDPOINT_SHIFT)
# define scale_load_down(w)	((w) >> SCHED_FIXEDPOINT_SHIFT)
#else
# define NICE_0_LOAD_SHIFT	(SCHED_FIXEDPOINT_SHIFT)
# define scale_load(w)		(w)
# define scale_load_down(w)	(w)
#endif

/*
 * Task weight (visible to users) and its load (invisible to users) have
 * independent resolution, but they should be well calibrated. We use
 * scale_load() and scale_load_down(w) to convert between them. The
 * following must be true:
 *
 *  scale_load(sched_prio_to_weight[USER_PRIO(NICE_TO_PRIO(0))]) == NICE_0_LOAD
 *
 */
#define NICE_0_LOAD		(1L << NICE_0_LOAD_SHIFT)

/*
 * Many a GCC version messes this up and generates a 64x64 mult :-(
 */
static inline u64 mul_u32_u32(u32 a, u32 b)
{
	return (u64)a * b;
}

static inline u64 mul_u64_u32_shr(u64 a, u32 mul, unsigned int shift)
{
	u32 ah, al;
	u64 ret;

	al = a;
	ah = a >> 32;

	ret = mul_u32_u32(al, mul) >> shift;
	if (ah)
		ret += mul_u32_u32(ah, mul) << (32 - shift);

	return ret;
}

/*
 * Targeted preemption latency for CPU-bound tasks:
 *
 * NOTE: this latency value is not the same as the concept of
 * 'timeslice length' - timeslices in CFS are of variable length
 * and have no persistent notion like in traditional, time-slice
 * based scheduling concepts.
 *
 * (to see the precise effective timeslice length of your workload,
 *  run vmstat and monitor the context-switches (cs) field)
 *
 * (default: 6ms * (1 + ilog(ncpus)), units: nanoseconds)
 */
unsigned int sysctl_sched_latency			= 6000000ULL;
unsigned int normalized_sysctl_sched_latency		= 6000000ULL;

/*
 * Minimal preemption granularity for CPU-bound tasks:
 *
 * (default: 0.75 msec * (1 + ilog(ncpus)), units: nanoseconds)
 */
unsigned int sysctl_sched_min_granularity		= 750000ULL;
unsigned int normalized_sysctl_sched_min_granularity	= 750000ULL;

/*
 * This value is kept at sysctl_sched_latency/sysctl_sched_min_granularity
 */
static unsigned int sched_nr_latency = 8;

/*
 * SCHED_OTHER wake-up granularity.
 *
 * This option delays the preemption effects of decoupled workloads
 * and reduces their over-scheduling. Synchronous workloads will still
 * have immediate wakeup/sleep latencies.
 *
 * (default: 1 msec * (1 + ilog(ncpus)), units: nanoseconds)
 */
unsigned int sysctl_sched_wakeup_granularity		= 1000000UL;
unsigned int normalized_sysctl_sched_wakeup_granularity	= 1000000UL;


static inline void update_load_add(struct load_weight *lw, unsigned long inc)
{
	lw->weight += inc;
	lw->inv_weight = 0;
}

static inline void update_load_sub(struct load_weight *lw, unsigned long dec)
{
	lw->weight -= dec;
	lw->inv_weight = 0;
}

static inline void update_load_set(struct load_weight *lw, unsigned long w)
{
	lw->weight = w;
	lw->inv_weight = 0;
}

#ifdef CONFIG_NO_HZ_FULL
#else
static inline void sched_update_tick_dependency(struct rq *rq) { }
#endif

static inline void add_nr_running(struct rq *rq, unsigned count)
{
	unsigned prev_nr = rq->nr_running;

	rq->nr_running = prev_nr + count;

	if (prev_nr < 2 && rq->nr_running >= 2) {
#ifdef CONFIG_SMP
		if (!rq->rd->overload)
			rq->rd->overload = true;
#endif
	}

	sched_update_tick_dependency(rq);
}

static inline void sub_nr_running(struct rq *rq, unsigned count)
{
	rq->nr_running -= count;
	/* Check if we still need preemption */
	sched_update_tick_dependency(rq);
}

#define WMULT_CONST	(~0U)
#define WMULT_SHIFT	32

static void __update_inv_weight(struct load_weight *lw)
{
	unsigned long w;

	if (likely(lw->inv_weight))
		return;

	w = scale_load_down(lw->weight);

	if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
		lw->inv_weight = 1;
	else if (unlikely(!w))
		lw->inv_weight = WMULT_CONST;
	else
		lw->inv_weight = WMULT_CONST / w;
}

/*
 * delta_exec * weight / lw.weight
 *   OR
 * (delta_exec * (weight * lw->inv_weight)) >> WMULT_SHIFT
 *
 * Either weight := NICE_0_LOAD and lw \e sched_prio_to_wmult[], in which case
 * we're guaranteed shift stays positive because inv_weight is guaranteed to
 * fit 32 bits, and NICE_0_LOAD gives another 10 bits; therefore shift >= 22.
 *
 * Or, weight =< lw.weight (because lw.weight is the runqueue weight), thus
 * weight/lw.weight <= 1, and therefore our shift will also be positive.
 */
static u64 __calc_delta(u64 delta_exec, unsigned long weight, struct load_weight *lw)
{
	u64 fact = scale_load_down(weight);
	int shift = WMULT_SHIFT;

	__update_inv_weight(lw);

	if (unlikely(fact >> 32)) {
		while (fact >> 32) {
			fact >>= 1;
			shift--;
		}
	}

	/* hint to use a 32x32->64 mul */
	fact = (u64)(u32)fact * lw->inv_weight;

	while (fact >> 32) {
		fact >>= 1;
		shift--;
	}

	return mul_u64_u32_shr(delta_exec, fact, shift);
}

static inline struct task_struct *task_of(struct sched_entity *se)
{
	return se;
}

static inline struct rq *rq_of(struct cfs_rq *cfs_rq)
{
	return cfs_rq;
}

#define entity_is_task(se)	1

#define for_each_sched_entity(se) \
		for (; se; se = NULL)

static inline struct cfs_rq *task_cfs_rq(struct task_struct *p)
{
    // changed, since vcpu contains the processor id
    // it will be easy to get he pcpu on the vcpu
    return CSCHED_PCPU(p->vcpu->processor);
}

static inline struct cfs_rq *cfs_rq_of(struct sched_entity *se)
{
    return task_cfs_rq(se);
}

/* runqueue "owned" by this group */
static inline struct cfs_rq *group_cfs_rq(struct sched_entity *grp)
{
	return NULL;
}

static inline void list_add_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

static inline void list_del_leaf_cfs_rq(struct cfs_rq *cfs_rq)
{
}

#define for_each_leaf_cfs_rq_safe(rq, cfs_rq, pos)	\
		for (cfs_rq = &rq->cfs, pos = NULL; cfs_rq; cfs_rq = pos)

static inline struct sched_entity *parent_entity(struct sched_entity *se)
{
	return NULL;
}

static inline void
find_matching_se(struct sched_entity **se, struct sched_entity **pse)
{
}

static inline
void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, u64 delta_exec);

/**************************************************************
 * Scheduling class tree data structure manipulation methods:
 */

static inline u64 max_vruntime(u64 max_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - max_vruntime);
	if (delta > 0)
		max_vruntime = vruntime;

	return max_vruntime;
}

static inline u64 min_vruntime(u64 min_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vruntime);
	if (delta < 0)
		min_vruntime = vruntime;

	return min_vruntime;
}

static inline int entity_before(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

static void update_min_vruntime(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	struct rb_node *leftmost = (cfs_rq->rb_leftmost);

	u64 vruntime = cfs_rq->min_vruntime;

	if (curr) {
		if (curr->on_rq)
			vruntime = curr->vruntime;
		else
			curr = NULL;
	}

	if (leftmost) { /* non-empty tree */
		struct sched_entity *se;
		se = rb_entry(leftmost, struct sched_entity, run_node);

		if (!curr)
			vruntime = se->vruntime;
		else
			vruntime = min_vruntime(vruntime, se->vruntime);
	}

	/* ensure we never gain time by being placed backwards. */
	cfs_rq->min_vruntime = max_vruntime(cfs_rq->min_vruntime, vruntime);
#ifndef CONFIG_64BIT
	smp_wmb();
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
}

/*
 * Enqueue an entity into the rb-tree:
 */
static void __enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	struct rb_node **link = &cfs_rq->runq_root.rb_node;
	struct rb_node *parent = NULL;
	struct sched_entity *entry;
	bool leftmost = true;

	/*
	 * Find the right place in the rbtree:
	 */
	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct sched_entity, run_node);
		/*
		 * We dont care about collisions. Nodes with
		 * the same key stay together.
		 */
		if (entity_before(se, entry)) {
			link = &parent->rb_left;
		} else {
			link = &parent->rb_right;
			leftmost = false;
		}
	}

    // Using a type in v4.13, decrpyted into rb_insert_color_cached in v4.14
    // Should change if having time
    /*
	 * Maintain a cache of leftmost tree entries (it is frequently
	 * used):
	 */
    if (leftmost)
		cfs_rq->rb_leftmost = &se->run_node;

	rb_link_node(&se->run_node, parent, link);
	rb_insert_color(&se->run_node,
			       &cfs_rq->runq_root);
}

static void __dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
    // Using a type in v4.13, decrpyted into rb_insert_color_cached in v4.14s
	// Should change if having time
    if (cfs_rq->rb_leftmost == &se->run_node) {
		struct rb_node *next_node;

		next_node = rb_next(&se->run_node);
		cfs_rq->rb_leftmost = next_node;
	}

	rb_erase(&se->run_node, &cfs_rq->runq_root);
}

struct sched_entity *__pick_first_entity(struct cfs_rq *cfs_rq)
{
    struct rb_node *left = cfs_rq->rb_leftmost;
	//struct rb_node *left = rb_first_cached(&cfs_rq->tasks_timeline);

	if (!left)
		return NULL;

	return rb_entry(left, struct sched_entity, run_node);
}

static struct sched_entity *__pick_next_entity(struct sched_entity *se)
{
	struct rb_node *next = rb_next(&se->run_node);

	if (!next)
		return NULL;

	return rb_entry(next, struct sched_entity, run_node);
}

//#define CONFIG_SCHED_DEBUG

#ifdef CONFIG_SCHED_DEBUG
struct sched_entity *__pick_last_entity(struct cfs_rq *cfs_rq)
{
	struct rb_node *last = rb_last(&cfs_rq->runq_root);

	if (!last)
		return NULL;

	return rb_entry(last, struct sched_entity, run_node);
}

#endif

/*
 * delta /= w
 */
static inline u64 calc_delta_fair(u64 delta, struct sched_entity *se)
{
	if (unlikely(se->load.weight != NICE_0_LOAD))
		delta = __calc_delta(delta, NICE_0_LOAD, &se->load);

	return delta;
}

/*
 * The idea is to set a period in which each task runs once.
 *
 * When there are too many tasks (sched_nr_latency) we have to stretch
 * this period because otherwise the slices get too small.
 *
 * p = (nr <= nl) ? l : l*nr/nl
 */
static u64 __sched_period(unsigned long nr_running)
{
	if (unlikely(nr_running > sched_nr_latency))
		return nr_running * sysctl_sched_min_granularity;
	else
		return sysctl_sched_latency;
}

/*
 * We calculate the wall-time slice from the period by taking a part
 * proportional to the weight.
 *
 * s = p*P[w/rw]
 */
static u64 sched_slice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	u64 slice = __sched_period(cfs_rq->nr_running + !se->on_rq);

    // the only vcpu, since vcpu do not have parenet or child
	for_each_sched_entity(se) {
		struct load_weight *load;
		struct load_weight lw;

        // get the load of pcpu
		cfs_rq = cfs_rq_of(se);
		load = &cfs_rq->load;

        // add vcpu load to pcpu if vcpu is not in pcpu
		if (unlikely(!se->on_rq)) {
			lw = cfs_rq->load;

            // will clear the inv_weight
            // should recalculate later
			update_load_add(&lw, se->load.weight);
			load = &lw;
		}
        // calculate the slice due to load of vcpu in pcpu
		slice = __calc_delta(slice, se->load.weight, load);
	}
	return slice;
}

static u64 sched_vslice(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	return calc_delta_fair(sched_slice(cfs_rq, se), se);
}

#ifdef CONFIG_SMP
#else /* !CONFIG_SMP */
void init_entity_runnable_average(struct sched_entity *se)
{
}
void post_init_entity_util_avg(struct sched_entity *se)
{
}
// static void update_tg_load_avg(struct cfs_rq *cfs_rq, int force)
// {
// }

#endif /* CONFIG_SMP */

/*
 * Update the current task's runtime statistics.
 */
static void update_curr(struct cfs_rq *cfs_rq)
{
	struct sched_entity *curr = cfs_rq->curr;
	//u64 now = rq_clock_task(rq_of(cfs_rq));
	u64 now = NOW();
    u64 delta_exec;

	if (unlikely(!curr))
		return;

	delta_exec = now - curr->exec_start;
	if (unlikely((s64)delta_exec <= 0))
		return;

	curr->exec_start = now;

	//schedstat_set(curr->statistics.exec_max,
	//	      max(delta_exec, curr->statistics.exec_max));

	curr->sum_exec_runtime += delta_exec;
	//schedstat_add(cfs_rq->exec_clock, delta_exec);

	curr->vruntime += calc_delta_fair(delta_exec, curr);
	update_min_vruntime(cfs_rq);

    // if(likely(entity_is_task(curr))){
	// if (entity_is_task(curr)) {
	// 	struct task_struct *curtask = task_of(curr);

	// 	trace_sched_stat_runtime(curtask, delta_exec, curr->vruntime);
	// 	cgroup_account_cputime(curtask, delta_exec);
	// 	account_group_exec_runtime(curtask, delta_exec);
	// }

	//account_cfs_rq_runtime(cfs_rq, delta_exec);
}

// static void update_curr_fair(struct rq *rq)
// {
// 	//update_curr(cfs_rq_of(&rq->curr->se));
//     update_curr(cfs_rq_of(rq->curr));
// }

/*
 * We are picking a new current task - update its stats:
 */
static inline void
update_stats_curr_start(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/*
	 * We are starting a new run period:
	 */
	//se->exec_start = rq_clock_task(rq_of(cfs_rq));
    se->exec_start = NOW();
}

/**************************************************
 * Scheduling class queueing methods:
 */

#ifdef CONFIG_NUMA_BALANCING
#else

static inline void account_numa_enqueue(struct rq *rq, struct task_struct *p)
{
}

static inline void account_numa_dequeue(struct rq *rq, struct task_struct *p)
{
}

#endif /* CONFIG_NUMA_BALANCING */

static void
account_entity_enqueue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
    // while enqueue, add the load of vcpu into pcpu
	update_load_add(&cfs_rq->load, se->load.weight);
	// treat rq same as pcpu
    // no need to reupdate
    //if (!parent_entity(se))
	//	update_load_add(&rq_of(cfs_rq)->load, se->load.weight);
#ifdef CONFIG_SMP
	if (entity_is_task(se)) {
		struct rq *rq = rq_of(cfs_rq);

		account_numa_enqueue(rq, task_of(se));
		list_add(&se->group_node, &rq->cfs_tasks);
	}
#endif
    // update nr_running
	cfs_rq->nr_running++;
}

static void
account_entity_dequeue(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	update_load_sub(&cfs_rq->load, se->load.weight);
	//if (!parent_entity(se))
	//	update_load_sub(&rq_of(cfs_rq)->load, se->load.weight);
#ifdef CONFIG_SMP
	if (entity_is_task(se)) {
		account_numa_dequeue(rq_of(cfs_rq), task_of(se));
		list_del_init(&se->group_node);
	}
#endif
	cfs_rq->nr_running--;
}

#ifdef CONFIG_SMP
#else
static inline void
enqueue_runnable_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se) { }
static inline void
dequeue_runnable_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se) { }
static inline void
enqueue_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se) { }
static inline void
dequeue_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se) { }
#endif

static void reweight_entity(struct cfs_rq *cfs_rq, struct sched_entity *se,
			    unsigned long weight, unsigned long runnable)
{
	if (se->on_rq) {
		/* commit outstanding execution time */
		if (cfs_rq->curr == se)
			update_curr(cfs_rq);
		account_entity_dequeue(cfs_rq, se);
		dequeue_runnable_load_avg(cfs_rq, se);
	}
	dequeue_load_avg(cfs_rq, se);

	se->runnable_weight = runnable;
	update_load_set(&se->load, weight);

#ifdef CONFIG_SMP
	do {
		u32 divider = LOAD_AVG_MAX - 1024 + se->avg.period_contrib;

		se->avg.load_avg = div_u64(se_weight(se) * se->avg.load_sum, divider);
		se->avg.runnable_load_avg =
			div_u64(se_runnable(se) * se->avg.runnable_load_sum, divider);
	} while (0);
#endif

	enqueue_load_avg(cfs_rq, se);
	if (se->on_rq) {
		account_entity_enqueue(cfs_rq, se);
		enqueue_runnable_load_avg(cfs_rq, se);
	}
}

void reweight_task(struct task_struct *p, int prio)
{
	struct sched_entity *se = p;
	struct cfs_rq *cfs_rq = cfs_rq_of(se);
	struct load_weight *load = &se->load;
	unsigned long weight = scale_load(sched_prio_to_weight[prio]);

	reweight_entity(cfs_rq, se, weight, weight);
	load->inv_weight = sched_prio_to_wmult[prio];
}

static inline void update_cfs_group(struct sched_entity *se)
{
}

#ifdef CONFIG_SMP
#else /* CONFIG_SMP */

static inline int
update_cfs_rq_load_avg(u64 now, struct cfs_rq *cfs_rq)
{
	return 0;
}

#define UPDATE_TG	0x0
#define SKIP_AGE_LOAD	0x0
#define DO_ATTACH	0x0

static inline void update_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se, int not_used1)
{
	//cfs_rq_util_change(cfs_rq, 0);
}

static inline void remove_entity_load_avg(struct sched_entity *se) {}

static inline void
attach_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags) {}
static inline void
detach_entity_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se) {}

//static inline int idle_balance(struct rq *rq, struct rq_flags *rf)
//{
//	return 0;
//}

static inline void
util_est_enqueue(struct cfs_rq *cfs_rq, struct task_struct *p) {}

static inline void
util_est_dequeue(struct cfs_rq *cfs_rq, struct task_struct *p,
		 bool task_sleep) {}

#endif /* CONFIG_SMP */

static void check_spread(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
#ifdef CONFIG_SCHED_DEBUG
	s64 d = se->vruntime - cfs_rq->min_vruntime;

	if (d < 0)
		d = -d;

	if (d > 3*sysctl_sched_latency)
		schedstat_inc(cfs_rq->nr_spread_over);
#endif
}

static void
place_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int initial)
{
	u64 vruntime = cfs_rq->min_vruntime;

	/*
	 * The 'current' period is already promised to the current tasks,
	 * however the extra weight of the new task will slow them down a
	 * little, place the new task so that it fits in the slot that
	 * stays open at the end.
	 */
	//if (initial && sched_feat(START_DEBIT))
	if (initial)
		vruntime += sched_vslice(cfs_rq, se);

	/* sleeps up to a single latency don't count. */
	if (!initial) {
		unsigned long thresh = sysctl_sched_latency;

		/*
		 * Halve their sleep time's effect, to allow
		 * for a gentler effect of sleepers:
		 */
		//if (sched_feat(GENTLE_FAIR_SLEEPERS))
			thresh >>= 1;

		vruntime -= thresh;
	}

	/* ensure we never gain time by being placed backwards. */
	se->vruntime = max_vruntime(se->vruntime, vruntime);
}

static void check_enqueue_throttle(struct cfs_rq *cfs_rq);

static inline void check_schedstat_required(void)
{
#ifdef CONFIG_SCHEDSTATS
	if (schedstat_enabled())
		return;

	/* Force schedstat enabled if a dependent tracepoint is active */
	if (trace_sched_stat_wait_enabled()    ||
			trace_sched_stat_sleep_enabled()   ||
			trace_sched_stat_iowait_enabled()  ||
			trace_sched_stat_blocked_enabled() ||
			trace_sched_stat_runtime_enabled())  {
		printk_deferred_once("Scheduler tracepoints stat_sleep, stat_iowait, "
			     "stat_blocked and stat_runtime require the "
			     "kernel parameter schedstats=enable or "
			     "kernel.sched_schedstats=1\n");
	}
#endif
}

static void
enqueue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	bool renorm = !(flags & ENQUEUE_WAKEUP) || (flags & ENQUEUE_MIGRATED);
	bool curr = cfs_rq->curr == se;

	/*
	 * If we're the current task, we must renormalise before calling
	 * update_curr().
	 */
	if (renorm && curr)
		se->vruntime += cfs_rq->min_vruntime;

	update_curr(cfs_rq);

	/*
	 * Otherwise, renormalise after, such that we're placed at the current
	 * moment in time, instead of some random moment in the past. Being
	 * placed in the past could significantly boost this task to the
	 * fairness detriment of existing tasks.
	 */
	if (renorm && !curr)
		se->vruntime += cfs_rq->min_vruntime;

	/*
	 * When enqueuing a sched_entity, we must:
	 *   - Update loads to have both entity and cfs_rq synced with now.
	 *   - Add its load to cfs_rq->runnable_avg
	 *   - For group_entity, update its weight to reflect the new share of
	 *     its group cfs_rq
	 *   - Add its new weight to cfs_rq->load.weight
	 */
	//update_load_avg(cfs_rq, se, UPDATE_TG | DO_ATTACH);
	//update_cfs_group(se);
	//enqueue_runnable_load_avg(cfs_rq, se);
	account_entity_enqueue(cfs_rq, se);

	if (flags & ENQUEUE_WAKEUP)
		place_entity(cfs_rq, se, 0);

	//check_schedstat_required();
	// update_stats_enqueue(cfs_rq, se, flags);
	//check_spread(cfs_rq, se);
	if (!curr)
		__enqueue_entity(cfs_rq, se);
	se->on_rq = 1;

	if (cfs_rq->nr_running == 1) {
		list_add_leaf_cfs_rq(cfs_rq);
		check_enqueue_throttle(cfs_rq);
	}
}

static void __clear_buddies_last(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->last != se)
			break;

		cfs_rq->last = NULL;
	}
}

static void __clear_buddies_next(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->next != se)
			break;

		cfs_rq->next = NULL;
	}
}

static void __clear_buddies_skip(struct sched_entity *se)
{
	for_each_sched_entity(se) {
		struct cfs_rq *cfs_rq = cfs_rq_of(se);
		if (cfs_rq->skip != se)
			break;

		cfs_rq->skip = NULL;
	}
}

static void clear_buddies(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	if (cfs_rq->last == se)
		__clear_buddies_last(se);

	if (cfs_rq->next == se)
		__clear_buddies_next(se);

	if (cfs_rq->skip == se)
		__clear_buddies_skip(se);
}

static inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void
dequeue_entity(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
	/*
	 * Update run-time statistics of the 'current'.
	 */
	update_curr(cfs_rq);

	/*
	 * When dequeuing a sched_entity, we must:
	 *   - Update loads to have both entity and cfs_rq synced with now.
	 *   - Substract its load from the cfs_rq->runnable_avg.
	 *   - Substract its previous weight from cfs_rq->load.weight.
	 *   - For group entity, update its weight to reflect the new share
	 *     of its group cfs_rq.
	 */
	//update_load_avg(cfs_rq, se, UPDATE_TG);
	//dequeue_runnable_load_avg(cfs_rq, se);

	//update_stats_dequeue(cfs_rq, se, flags);

	clear_buddies(cfs_rq, se);

	if (se != cfs_rq->curr)
		__dequeue_entity(cfs_rq, se);
	se->on_rq = 0;
	account_entity_dequeue(cfs_rq, se);

	/*
	 * Normalize after update_curr(); which will also have moved
	 * min_vruntime if @se is the one holding it back. But before doing
	 * update_min_vruntime() again, which will discount @se's position and
	 * can move min_vruntime forward still more.
	 */
	if (!(flags & DEQUEUE_SLEEP))
		se->vruntime -= cfs_rq->min_vruntime;

	/* return excess runtime on last dequeue */
	return_cfs_rq_runtime(cfs_rq);

	//update_cfs_group(se);

	/*
	 * Now advance min_vruntime if @se was the entity holding it back,
	 * except when: DEQUEUE_SAVE && !DEQUEUE_MOVE, in this case we'll be
	 * put back on, and if we advance min_vruntime, we'll be placed back
	 * further than we started -- ie. we'll be penalized.
	 */
	if ((flags & (DEQUEUE_SAVE | DEQUEUE_MOVE)) == DEQUEUE_SAVE)
		update_min_vruntime(cfs_rq);
}

/*
 * Preempt the current task with a newly woken task if needed:
 */
static void
check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	unsigned long ideal_runtime, delta_exec;
	struct sched_entity *se;
	s64 delta;

	ideal_runtime = sched_slice(cfs_rq, curr);
	delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
	if (delta_exec > ideal_runtime) {
		//resched_curr(rq_of(cfs_rq));
		cpu_raise_softirq(cfs_rq->cpu, SCHEDULE_SOFTIRQ);   
        /*
		 * The current task ran long enough, ensure it doesn't get
		 * re-elected due to buddy favours.
		 */
		clear_buddies(cfs_rq, curr);
		return;
	}

	/*
	 * Ensure that a task that missed wakeup preemption by a
	 * narrow margin doesn't have to wait for a full slice.
	 * This also mitigates buddy induced latencies under load.
	 */
	if (delta_exec < sysctl_sched_min_granularity)
		return;

	se = __pick_first_entity(cfs_rq);
	delta = curr->vruntime - se->vruntime;

	if (delta < 0)
		return;

	if (delta > ideal_runtime)
        cpu_raise_softirq(cfs_rq->cpu, SCHEDULE_SOFTIRQ);    
		//resched_curr(rq_of(cfs_rq));
}

static void
set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se)
{
	/* 'current' is not kept within the tree. */
	if (se->on_rq) {
		/*
		 * Any task has to be enqueued before it get to execute on
		 * a CPU. So account for the time it spent waiting on the
		 * runqueue.
		 */
		//update_stats_wait_end(cfs_rq, se);
		__dequeue_entity(cfs_rq, se);
		//update_load_avg(cfs_rq, se, UPDATE_TG);
	}

	update_stats_curr_start(cfs_rq, se);
	cfs_rq->curr = se;

	/*
	 * Track our maximum slice length, if the CPU's load is at
	 * least twice that of our own weight (i.e. dont track it
	 * when there are only lesser-weight tasks around):
	 */
	// if (schedstat_enabled() && rq_of(cfs_rq)->load.weight >= 2*se->load.weight) {
	// 	schedstat_set(se->statistics.slice_max,
	// 		max((u64)schedstat_val(se->statistics.slice_max),
	// 		    se->sum_exec_runtime - se->prev_sum_exec_runtime));
	// }

	se->prev_sum_exec_runtime = se->sum_exec_runtime;
}

static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se);

/*
 * Pick the next process, keeping these things in mind, in this order:
 * 1) keep things fair between processes/task groups
 * 2) pick the "next" process, since someone really wants that to run
 * 3) pick the "last" process, for cache locality
 * 4) do not run the "skip" process, if something else is available
 */
static struct sched_entity *
pick_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
	struct sched_entity *left = __pick_first_entity(cfs_rq);
	struct sched_entity *se;

	/*
	 * If curr is set we have to see if its left of the leftmost entity
	 * still in the tree, provided there was anything in the tree at all.
	 */
	if (!left || (curr && entity_before(curr, left)))
		left = curr;

	se = left; /* ideally we run the leftmost entity */

	/*
	 * Avoid running the skip buddy, if running something else can
	 * be done without getting too unfair.
	 */
	if (cfs_rq->skip == se) {
		struct sched_entity *second;

		if (se == curr) {
			second = __pick_first_entity(cfs_rq);
		} else {
			second = __pick_next_entity(se);
			if (!second || (curr && entity_before(curr, second)))
				second = curr;
		}

		if (second && wakeup_preempt_entity(second, left) < 1)
			se = second;
	}

	/*
	 * Prefer last buddy, try to return the CPU to a preempted task.
	 */
	if (cfs_rq->last && wakeup_preempt_entity(cfs_rq->last, left) < 1)
		se = cfs_rq->last;

	/*
	 * Someone really wants this to run. If it's not unfair, run it.
	 */
	if (cfs_rq->next && wakeup_preempt_entity(cfs_rq->next, left) < 1)
		se = cfs_rq->next;

	clear_buddies(cfs_rq, se);

	return se;
}

static bool check_cfs_rq_runtime(struct cfs_rq *cfs_rq);

static void put_prev_entity(struct cfs_rq *cfs_rq, struct sched_entity *prev)
{
	/*
	 * If still on the runqueue then deactivate_task()
	 * was not called and update_curr() has to be done:
	 */
	if (prev->on_rq)
		update_curr(cfs_rq);

	/* throttle cfs_rqs exceeding runtime */
	check_cfs_rq_runtime(cfs_rq);

	check_spread(cfs_rq, prev);

	if (prev->on_rq) {
		//update_stats_wait_start(cfs_rq, prev);
		/* Put 'current' back into the tree. */
		__enqueue_entity(cfs_rq, prev);
		/* in !on_rq case, update occurred at dequeue */
		update_load_avg(cfs_rq, prev, 0);
	}
	cfs_rq->curr = NULL;
}

static void
entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
	/*
	 * Update run-time statistics of the 'current'.
	 */
	update_curr(cfs_rq);

	/*
	 * Ensure that runnable average is periodically updated.
	 */
	//update_load_avg(cfs_rq, curr, UPDATE_TG);
	//update_cfs_group(curr);

#ifdef CONFIG_SCHED_HRTICK
	/*
	 * queued ticks are scheduled to match the slice, so don't bother
	 * validating it and just reschedule.
	 */
	if (queued) {
		resched_curr(rq_of(cfs_rq));
		return;
	}
	/*
	 * don't let the period tick interfere with the hrtick preemption
	 */
	if (!sched_feat(DOUBLE_TICK) &&
			hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
		return;
#endif

    // if (cfs_rq->nr_running > 1)
	// 	check_preempt_tick(cfs_rq, curr);
}

#ifdef CONFIG_CFS_BANDWIDTH
#else /* CONFIG_CFS_BANDWIDTH */
static inline u64 cfs_rq_clock_task(struct cfs_rq *cfs_rq)
{
    return NOW();
	//return rq_clock_task(rq_of(cfs_rq));
}

static void account_cfs_rq_runtime(struct cfs_rq *cfs_rq, u64 delta_exec) {}
static bool check_cfs_rq_runtime(struct cfs_rq *cfs_rq) { return false; }
static void check_enqueue_throttle(struct cfs_rq *cfs_rq) {}

// static inline void sync_throttle(struct task_group *tg, int cpu) {}
static inline void return_cfs_rq_runtime(struct cfs_rq *cfs_rq) {}

static inline int cfs_rq_throttled(struct cfs_rq *cfs_rq)
{
	return 0;
}

static inline int throttled_hierarchy(struct cfs_rq *cfs_rq)
{
	return 0;
}

// static inline int throttled_lb_pair(struct task_group *tg,
// 				    int src_cpu, int dest_cpu)
// {
// 	return 0;
// }

// void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}

// static inline struct cfs_bandwidth *tg_cfs_bandwidth(struct task_group *tg)
// {
// 	return NULL;
// }
// static inline void destroy_cfs_bandwidth(struct cfs_bandwidth *cfs_b) {}
static inline void update_runtime_enabled(struct rq *rq) {}
static inline void unthrottle_offline_cfs_rqs(struct rq *rq) {}

#endif /* CONFIG_CFS_BANDWIDTH */

#ifdef CONFIG_SCHED_HRTICK
#else /* !CONFIG_SCHED_HRTICK */
static inline void
hrtick_start_fair(struct rq *rq, struct task_struct *p)
{
}

static inline void hrtick_update(struct rq *rq)
{
}
#endif

/*
 * The enqueue_task method is called before nr_running is
 * increased. Here we update the fair scheduling stats and
 * then put the task into the rbtree:
 */
static void
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags)
{
	struct cfs_rq *_cfs_rq;
	struct sched_entity *se = p;

	/*
	 * If in_iowait is set, the code below may not trigger any cpufreq
	 * utilization updates, so do it here explicitly with the IOWAIT flag
	 * passed.
	 */
	// if (p->in_iowait)
	// 	cpufreq_update_util(rq, SCHED_CPUFREQ_IOWAIT);

		if (se->on_rq)
			return;
		_cfs_rq = cfs_rq_of(se);
		enqueue_entity(_cfs_rq, se, flags);

		/*
		 * end evaluation on encountering a throttled cfs_rq
		 *
		 * note: in the case of encountering a throttled cfs_rq we will
		 * post the final h_nr_running increment below.
		 */
		// if (cfs_rq_throttled(cfs_rq))
		// 	break;
		// cfs_rq->h_nr_running++;

		flags = ENQUEUE_WAKEUP;

    // For Group, no use
	// for_each_sched_entity(se) {
	// 	cfs_rq = cfs_rq_of(se);
	// 	cfs_rq->h_nr_running++;

	// 	if (cfs_rq_throttled(cfs_rq))
	// 		break;

	// 	update_load_avg(cfs_rq, se, UPDATE_TG);
	// 	update_cfs_group(se);
	// }

	// if (!se)
	// 	add_nr_running(rq, 1);

	// util_est_enqueue(rq, p);
	// hrtick_update(rq);
}

static void set_next_buddy(struct sched_entity *se);

/*
 * The dequeue_task method is called before nr_running is
 * decreased. We remove the task from the rbtree and
 * update the fair scheduling stats:
 */
static void dequeue_task_fair(struct rq *_rq, struct task_struct *p, int flags)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = p;
	//int task_sleep = flags & DEQUEUE_SLEEP;

		cfs_rq = cfs_rq_of(se);
		dequeue_entity(cfs_rq, se, flags);

		/*
		 * end evaluation on encountering a throttled cfs_rq
		 *
		 * note: in the case of encountering a throttled cfs_rq we will
		 * post the final h_nr_running decrement below.
		*/
		// if (cfs_rq_throttled(cfs_rq))
		// 	break;
		// cfs_rq->h_nr_running--;

		/* Don't dequeue parent if it has other entities besides us */
		if (cfs_rq->load.weight) {
			/* Avoid re-evaluating load for this entity: */
			// se = parent_entity(se);
			/*
			 * Bias pick_next to pick a task from this cfs_rq, as
			 * p is sleeping when it is within its sched_slice.
			 */
			// if (task_sleep && se && !throttled_hierarchy(cfs_rq))
			// 	set_next_buddy(se);
			return;
		}
		flags |= DEQUEUE_SLEEP;

	// for_each_sched_entity(se) {
	// 	cfs_rq = cfs_rq_of(se);
	// 	cfs_rq->h_nr_running--;

	// 	if (cfs_rq_throttled(cfs_rq))
	// 		break;

	// 	update_load_avg(cfs_rq, se, UPDATE_TG);
	// 	update_cfs_group(se);
	// }

	// if (!se)
	// 	sub_nr_running(_rq, 1);

	// util_est_dequeue(_rq, p, task_sleep);
	// hrtick_update(_rq);
}

static unsigned long wakeup_gran(struct sched_entity *se)
{
	unsigned long gran = sysctl_sched_wakeup_granularity;

	/*
	 * Since its curr running now, convert the gran from real-time
	 * to virtual-time in his units.
	 *
	 * By using 'se' instead of 'curr' we penalize light tasks, so
	 * they get preempted easier. That is, if 'se' < 'curr' then
	 * the resulting gran will be larger, therefore penalizing the
	 * lighter, if otoh 'se' > 'curr' then the resulting gran will
	 * be smaller, again penalizing the lighter task.
	 *
	 * This is especially important for buddies when the leftmost
	 * task is higher priority than the buddy.
	 */
	return calc_delta_fair(gran, se);
}

/*
 * Should 'se' preempt 'curr'.
 *
 *             |s1
 *        |s2
 *   |s3
 *         g
 *      |<--->|c
 *
 *  w(c, s1) = -1
 *  w(c, s2) =  0
 *  w(c, s3) =  1
 *
 */
static int
wakeup_preempt_entity(struct sched_entity *curr, struct sched_entity *se)
{
	s64 gran, vdiff = curr->vruntime - se->vruntime;

	if (vdiff <= 0)
		return -1;

	gran = wakeup_gran(se);
	if (vdiff > gran)
		return 1;

	return 0;
}

# define SCHED_WARN_ON(x)	({ (void)(x), 0; })

static void set_last_buddy(struct sched_entity *se)
{
	// if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
	// 	return;

	for_each_sched_entity(se) {
		if (SCHED_WARN_ON(!se->on_rq))
			return;
		cfs_rq_of(se)->last = se;
	}
}

static void set_next_buddy(struct sched_entity *se)
{
	// if (entity_is_task(se) && unlikely(task_of(se)->policy == SCHED_IDLE))
	// 	return;

	for_each_sched_entity(se) {
		if (SCHED_WARN_ON(!se->on_rq))
			return;
		cfs_rq_of(se)->next = se;
	}
}

static void set_skip_buddy(struct sched_entity *se)
{
	for_each_sched_entity(se)
		cfs_rq_of(se)->skip = se;
}

/*
 * Preempt the current task with a newly woken task if needed:
 */
static void check_preempt_wakeup(struct rq *_rq, struct task_struct *p, int wake_flags)
{
	struct task_struct *curr = _rq->curr;
	struct sched_entity *se = curr, *pse = p;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	int scale = cfs_rq->nr_running >= sched_nr_latency;
	int next_buddy_marked = 0;

	if (unlikely(se == pse))
		return;

	/*
	 * This is possible from callers such as attach_tasks(), in which we
	 * unconditionally check_prempt_curr() after an enqueue (which may have
	 * lead to a throttle).  This both saves work and prevents false
	 * next-buddy nomination below.
	 */
	if (unlikely(throttled_hierarchy(cfs_rq_of(pse))))
		return;

	// if (sched_feat(NEXT_BUDDY) && scale) {
    if (scale) {
		set_next_buddy(pse);
		next_buddy_marked = 1;
	}

	/*
	 * We can come here with TIF_NEED_RESCHED already set from new task
	 * wake up path.
	 *
	 * Note: this also catches the edge-case of curr being in a throttled
	 * group (e.g. via set_curr_task), since update_curr() (in the
	 * enqueue of curr) will have resulted in resched being set.  This
	 * prevents us from potentially nominating it as a false LAST_BUDDY
	 * below.
	 */
	// if (test_tsk_need_resched(curr))
	// 	return;

	/* Idle tasks are by definition preempted by non-idle tasks. */
	// if (unlikely(curr->policy == SCHED_IDLE) &&
	//     likely(p->policy != SCHED_IDLE))
	// 	goto preempt;

	/*
	 * Batch and idle tasks do not preempt non-idle tasks (their preemption
	 * is driven by the tick):
	 */
	// if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
	// 	return;

	find_matching_se(&se, &pse);
	update_curr(cfs_rq_of(se));
	BUG_ON(!pse);
	if (wakeup_preempt_entity(se, pse) == 1) {
		/*
		 * Bias pick_next to pick the sched entity that is
		 * triggering this preemption.
		 */
		if (!next_buddy_marked)
			set_next_buddy(pse);
		goto preempt;
	}

	return;

preempt:
	//resched_curr(rq);
    cpu_raise_softirq(_rq->cpu, SCHEDULE_SOFTIRQ);
	/*
	 * Only set the backward buddy when the current task is still
	 * on the rq. This can happen when a wakeup gets interleaved
	 * with schedule on the ->pre_schedule() or idle_balance()
	 * point, either of which can * drop the rq lock.
	 *
	 * Also, during early boot the idle thread is in the fair class,
	 * for obvious reasons its a bad idea to schedule back to it.
	 */
	// if (unlikely(!se->on_rq || curr == _rq->idle))
    if (unlikely(!se->on_rq))
		return;

	// if (sched_feat(LAST_BUDDY) && scale && entity_is_task(se))
	// 	set_last_buddy(se);
    if (scale && entity_is_task(se))
		set_last_buddy(se);
}

/*
 * Account for a descheduled task:
 */
static void put_prev_task_fair(struct rq *_rq, struct task_struct *prev)
{
	struct sched_entity *se = prev;
	struct cfs_rq *cfs_rq;

		cfs_rq = cfs_rq_of(se);
		put_prev_entity(cfs_rq, se);
}

// static struct task_struct *
// pick_next_task_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
static struct task_struct *
pick_next_task_fair(struct rq *_rq, struct task_struct *prev)
{
	struct cfs_rq *cfs_rq = _rq;
	struct sched_entity *se;
	struct task_struct *p;
	int new_tasks;

//again:
	if (!cfs_rq->nr_running)
		goto idle;

	put_prev_task_fair(_rq, prev);

	//do {
		se = pick_next_entity(cfs_rq, NULL);
		set_next_entity(cfs_rq, se);
		//cfs_rq = group_cfs_rq(se);
	//} while (cfs_rq);

	p = task_of(se);

done: __maybe_unused;
#ifdef CONFIG_SMP
	/*
	 * Move the next running task to the front of
	 * the list, so our cfs_tasks list becomes MRU
	 * one.
	 */
	list_move(&p->se.group_node, &rq->cfs_tasks);
#endif

	// if (hrtick_enabled(rq))
	// 	hrtick_start_fair(rq, p);

	return p;

idle:
	//new_tasks = idle_balance(rq, rf);
    new_tasks = 0;

	/*
	 * Because idle_balance() releases (and re-acquires) rq->lock, it is
	 * possible for any higher priority task to appear. In that case we
	 * must re-start the pick_next_entity() loop.
	 */
	// if (new_tasks < 0)
	// 	return RETRY_TASK;

	// if (new_tasks > 0)
	// 	goto again;

	return NULL;
}

/*
 * sched_yield() is very simple
 *
 * The magic of dealing with the ->skip buddy is in pick_next_entity.
 */
static void yield_task_fair(struct rq *_rq)
{
	struct task_struct *curr = _rq->curr;
	struct cfs_rq *cfs_rq = task_cfs_rq(curr);
	struct sched_entity *se = curr;

	/*
	 * Are we the only task in the tree?
	 */
	if (unlikely(_rq->nr_running == 1))
		return;

	clear_buddies(cfs_rq, se);

//	if (curr->policy != SCHED_BATCH) {
		//update_rq_clock(rq);
		/*
		 * Update run-time statistics of the 'current'.
		 */
		update_curr(cfs_rq);
		/*
		 * Tell update_rq_clock() that we've just updated,
		 * so we don't do microscopic update in schedule()
		 * and double the fastpath cost.
		 */
		//rq_clock_skip_update(rq);
//	}

	set_skip_buddy(se);
}

// static bool yield_to_task_fair(struct rq *rq, struct task_struct *p, bool preempt)
// {
// 	struct sched_entity *se = p;

// 	/* throttled hierarchies are not runnable */
// 	if (!se->on_rq || throttled_hierarchy(cfs_rq_of(se)))
// 		return false;

// 	/* Tell the scheduler that we'd really like pse to run next. */
// 	set_next_buddy(se);

// 	yield_task_fair(rq);

// 	return true;
// }


/*
 * scheduler tick hitting a task of our scheduling class.
 *
 * NOTE: This function can be called remotely by the tick offload that
 * goes along full dynticks. Therefore no local assumption can be made
 * and everything must be accessed through the @rq and @curr passed in
 * parameters.
 */
static void task_tick_fair(struct rq *_rq, struct task_struct *curr, int queued)
{
	struct cfs_rq *cfs_rq;
	struct sched_entity *se = curr;

	//for_each_sched_entity(se) {
		cfs_rq = cfs_rq_of(se);
		entity_tick(cfs_rq, se, queued);
	//}

	// if (static_branch_unlikely(&sched_numa_balancing))
	// 	task_tick_numa(rq, curr);
}

/* task_struct::on_rq states: */
#define TASK_ON_RQ_QUEUED	1
#define TASK_ON_RQ_MIGRATING	2

static inline int task_on_rq_queued(struct task_struct *p)
{
	return p->on_rq == TASK_ON_RQ_QUEUED;
}

static inline int task_on_rq_migrating(struct task_struct *p)
{
	return p->on_rq == TASK_ON_RQ_MIGRATING;
}

static inline bool vruntime_normalized(struct task_struct *p)
{
	struct sched_entity *se = p;

	/*
	 * In both the TASK_ON_RQ_QUEUED and TASK_ON_RQ_MIGRATING cases,
	 * the dequeue_entity(.flags=0) will already have normalized the
	 * vruntime.
	 */
	if (p->on_rq)
		return true;

	/*
	 * When !on_rq, vruntime of the task has usually NOT been normalized.
	 * But there are some cases where it has already been normalized:
	 *
	 * - A forked child which is waiting for being woken up by
	 *   wake_up_new_task().
	 * - A task which has been woken up by try_to_wake_up() and
	 *   waiting for actually being woken up by sched_ttwu_pending().
	 */
	// if (!se->sum_exec_runtime || p->state == TASK_WAKING)
	// 	return true;
    if (!se->sum_exec_runtime)
        return true;

	return false;
}

//static void propagate_entity_cfs_rq(struct sched_entity *se) { }

// static void detach_entity_cfs_rq(struct sched_entity *se)
// {
// 	struct cfs_rq *cfs_rq = cfs_rq_of(se);

// 	/* Catch up with the cfs_rq and remove our load when we leave */
// 	update_load_avg(cfs_rq, se, 0);
// 	detach_entity_load_avg(cfs_rq, se);
// 	update_tg_load_avg(cfs_rq, false);
// 	propagate_entity_cfs_rq(se);
// }

// static void attach_entity_cfs_rq(struct sched_entity *se)
// {
// 	struct cfs_rq *cfs_rq = cfs_rq_of(se);

// 	/* Synchronize entity with its cfs_rq */
// 	update_load_avg(cfs_rq, se, SKIP_AGE_LOAD);
// 	attach_entity_load_avg(cfs_rq, se, 0);
// 	update_tg_load_avg(cfs_rq, false);
// 	propagate_entity_cfs_rq(se);
// }

// static void detach_task_cfs_rq(struct task_struct *p)
// {
// 	struct sched_entity *se = p;
// 	struct cfs_rq *cfs_rq = cfs_rq_of(se);

// 	if (!vruntime_normalized(p)) {
// 		/*
// 		 * Fix up our vruntime so that the current sleep doesn't
// 		 * cause 'unlimited' sleep bonus.
// 		 */
// 		place_entity(cfs_rq, se, 0);
// 		se->vruntime -= cfs_rq->min_vruntime;
// 	}

// 	detach_entity_cfs_rq(se);
// }

// static void attach_task_cfs_rq(struct task_struct *p)
// {
// 	struct sched_entity *se = p;
// 	struct cfs_rq *cfs_rq = cfs_rq_of(se);

// 	//attach_entity_cfs_rq(se);

// 	if (!vruntime_normalized(p))
// 		se->vruntime += cfs_rq->min_vruntime;
// }

/* Account for a task changing its policy or group.
 *
 * This routine is mostly called to set cfs_rq->curr field when a task
 * migrates between groups/classes.
 */
// static void set_curr_task_fair(struct rq *rq)
// {
// 	struct sched_entity *se = rq->curr;

// 	for_each_sched_entity(se) {
// 		struct cfs_rq *cfs_rq = cfs_rq_of(se);

// 		set_next_entity(cfs_rq, se);
// 		/* ensure bandwidth has been allocated on our new cfs_rq */
// 		account_cfs_rq_runtime(cfs_rq, 0);
// 	}
// }

void init_cfs_rq(struct cfs_rq *cfs_rq)
{
	//cfs_rq->tasks_timeline = RB_ROOT_CACHED;
    cfs_rq->runq_root = RB_ROOT;
    cfs_rq->rb_leftmost = NULL;
	cfs_rq->min_vruntime = (u64)(-(1LL << 20));
#ifndef CONFIG_64BIT
	cfs_rq->min_vruntime_copy = cfs_rq->min_vruntime;
#endif
#ifdef CONFIG_SMP
	raw_spin_lock_init(&cfs_rq->removed.lock);
#endif
}


static void csched_tick(void *_cpu);
static void csched_acct(void *dummy);

static inline int
__vcpu_on_runq(struct csched_vcpu *svc)
{
    return !list_empty(&svc->runq_elem);
}

static inline struct csched_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct csched_vcpu, runq_elem);
}

/* Is the first element of cpu's runq (if any) cpu's idle vcpu? */
static inline bool_t is_runq_idle(unsigned int cpu)
{
    /*
     * We're peeking at cpu's runq, we must hold the proper lock.
     */
    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));

    return list_empty(RUNQ(cpu)) ||
           is_idle_vcpu(__runq_elem(RUNQ(cpu)->next)->vcpu);
}

static inline void
inc_nr_runnable(unsigned int cpu)
{
    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));
    CSCHED_PCPU(cpu)->nr_runnable++;

}

static inline void
dec_nr_runnable(unsigned int cpu)
{
    ASSERT(spin_is_locked(per_cpu(schedule_data, cpu).schedule_lock));
    ASSERT(CSCHED_PCPU(cpu)->nr_runnable >= 1);
    CSCHED_PCPU(cpu)->nr_runnable--;
}

static inline void
__runq_insert(struct csched_vcpu *svc)
{
    unsigned int cpu = svc->vcpu->processor;
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;

    BUG_ON( __vcpu_on_runq(svc) );

    list_for_each( iter, runq )
    {
        const struct csched_vcpu * const iter_svc = __runq_elem(iter);
        if ( svc->pri > iter_svc->pri )
            break;
    }

    /* If the vcpu yielded, try to put it behind one lower-priority
     * runnable vcpu if we can.  The next runq_sort will bring it forward
     * within 30ms if the queue too long. */
    if ( test_bit(CSCHED_FLAG_VCPU_YIELD, &svc->flags)
         && __runq_elem(iter)->pri > CSCHED_PRI_IDLE )
    {
        iter=iter->next;

        /* Some sanity checks */
        BUG_ON(iter == runq);
    }

    list_add_tail(&svc->runq_elem, iter);
}

static inline void
runq_insert(struct csched_vcpu *svc)
{
    __runq_insert(svc);
    inc_nr_runnable(svc->vcpu->processor);
}

static inline void
__runq_remove(struct csched_vcpu *svc)
{
    BUG_ON( !__vcpu_on_runq(svc) );
    list_del_init(&svc->runq_elem);
}

static inline void
runq_remove(struct csched_vcpu *svc)
{
    dec_nr_runnable(svc->vcpu->processor);
    __runq_remove(svc);
}

static void burn_credits(struct csched_vcpu *svc, s_time_t now)
{
    s_time_t delta;
    uint64_t val;
    unsigned int credits;

    /* Assert svc is current */
    ASSERT( svc == CSCHED_VCPU(curr_on_cpu(svc->vcpu->processor)) );

    if ( (delta = now - svc->start_time) <= 0 )
        return;

    val = delta * CSCHED_CREDITS_PER_MSEC + svc->residual;
    svc->residual = do_div(val, MILLISECS(1));
    credits = val;
    ASSERT(credits == val); /* make sure we haven't truncated val */
    atomic_sub(credits, &svc->credit);
    svc->start_time += (credits * MILLISECS(1)) / CSCHED_CREDITS_PER_MSEC;
}

static bool_t __read_mostly opt_tickle_one_idle = 1;
boolean_param("tickle_one_idle_cpu", opt_tickle_one_idle);

DEFINE_PER_CPU(unsigned int, last_tickle_cpu);

static inline void __runq_tickle(struct csched_vcpu *new)
{
    unsigned int cpu = new->vcpu->processor;
    struct csched_vcpu * const cur = CSCHED_VCPU(curr_on_cpu(cpu));
    struct csched_private *prv = CSCHED_PRIV(per_cpu(scheduler, cpu));
    cpumask_t mask, idle_mask, *online;
    int balance_step, idlers_empty;

    ASSERT(cur);
    cpumask_clear(&mask);

    online = cpupool_domain_cpumask(new->sdom->dom);
    cpumask_and(&idle_mask, prv->idlers, online);
    idlers_empty = cpumask_empty(&idle_mask);

    /*
     * Exclusive pinning is when a vcpu has hard-affinity with only one
     * cpu, and there is no other vcpu that has hard-affinity with that
     * same cpu. This is infrequent, but if it happens, is for achieving
     * the most possible determinism, and least possible overhead for
     * the vcpus in question.
     *
     * Try to identify the vast majority of these situations, and deal
     * with them quickly.
     */
    if ( unlikely(test_bit(CSCHED_FLAG_VCPU_PINNED, &new->flags) &&
                  cpumask_test_cpu(cpu, &idle_mask)) )
    {
        ASSERT(cpumask_cycle(cpu, new->vcpu->cpu_hard_affinity) == cpu);
        SCHED_STAT_CRANK(tickled_idle_cpu_excl);
        __cpumask_set_cpu(cpu, &mask);
        goto tickle;
    }

    /*
     * If the pcpu is idle, or there are no idlers and the new
     * vcpu is a higher priority than the old vcpu, run it here.
     *
     * If there are idle cpus, first try to find one suitable to run
     * new, so we can avoid preempting cur.  If we cannot find a
     * suitable idler on which to run new, run it here, but try to
     * find a suitable idler on which to run cur instead.
     */
    if ( cur->pri == CSCHED_PRI_IDLE
         || (idlers_empty && new->pri > cur->pri) )
    {
        if ( cur->pri != CSCHED_PRI_IDLE )
            SCHED_STAT_CRANK(tickled_busy_cpu);
        else
            SCHED_STAT_CRANK(tickled_idle_cpu);
        __cpumask_set_cpu(cpu, &mask);
    }
    else if ( !idlers_empty )
    {
        /*
         * Soft and hard affinity balancing loop. For vcpus without
         * a useful soft affinity, consider hard affinity only.
         */
        for_each_affinity_balance_step( balance_step )
        {
            int new_idlers_empty;

            if ( balance_step == BALANCE_SOFT_AFFINITY
                 && !has_soft_affinity(new->vcpu) )
                continue;

            /* Are there idlers suitable for new (for this balance step)? */
            affinity_balance_cpumask(new->vcpu, balance_step,
                                     cpumask_scratch_cpu(cpu));
            cpumask_and(cpumask_scratch_cpu(cpu),
                        cpumask_scratch_cpu(cpu), &idle_mask);
            new_idlers_empty = cpumask_empty(cpumask_scratch_cpu(cpu));

            /*
             * Let's not be too harsh! If there aren't idlers suitable
             * for new in its soft affinity mask, make sure we check its
             * hard affinity as well, before taking final decisions.
             */
            if ( new_idlers_empty
                 && balance_step == BALANCE_SOFT_AFFINITY )
                continue;

            /*
             * If there are no suitable idlers for new, and it's higher
             * priority than cur, check whether we can migrate cur away.
             * We have to do it indirectly, via _VPF_migrating (instead
             * of just tickling any idler suitable for cur) because cur
             * is running.
             *
             * If there are suitable idlers for new, no matter priorities,
             * leave cur alone (as it is running and is, likely, cache-hot)
             * and wake some of them (which is waking up and so is, likely,
             * cache cold anyway).
             */
            if ( new_idlers_empty && new->pri > cur->pri )
            {
                if ( cpumask_intersects(cur->vcpu->cpu_hard_affinity,
                                        &idle_mask) )
                {
                    SCHED_VCPU_STAT_CRANK(cur, kicked_away);
                    SCHED_VCPU_STAT_CRANK(cur, migrate_r);
                    SCHED_STAT_CRANK(migrate_kicked_away);
                    set_bit(_VPF_migrating, &cur->vcpu->pause_flags);
                }
                /* Tickle cpu anyway, to let new preempt cur. */
                SCHED_STAT_CRANK(tickled_busy_cpu);
                __cpumask_set_cpu(cpu, &mask);
            }
            else if ( !new_idlers_empty )
            {
                /* Which of the idlers suitable for new shall we wake up? */
                SCHED_STAT_CRANK(tickled_idle_cpu);
                if ( opt_tickle_one_idle )
                {
                    this_cpu(last_tickle_cpu) =
                        cpumask_cycle(this_cpu(last_tickle_cpu),
                                      cpumask_scratch_cpu(cpu));
                    __cpumask_set_cpu(this_cpu(last_tickle_cpu), &mask);
                }
                else
                    cpumask_or(&mask, &mask, cpumask_scratch_cpu(cpu));
            }

            /* Did we find anyone? */
            if ( !cpumask_empty(&mask) )
                break;
        }
    }

 tickle:
    if ( !cpumask_empty(&mask) )
    {
        if ( unlikely(tb_init_done) )
        {
            /* Avoid TRACE_*: saves checking !tb_init_done each step */
            for_each_cpu(cpu, &mask)
                __trace_var(TRC_CSCHED_TICKLE, 1, sizeof(cpu), &cpu);
        }

        /*
         * Mark the designated CPUs as busy and send them all the scheduler
         * interrupt. We need the for_each_cpu for dealing with the
         * !opt_tickle_one_idle case. We must use cpumask_clear_cpu() and
         * can't use cpumask_andnot(), because prv->idlers needs atomic access.
         *
         * In the default (and most common) case, when opt_rickle_one_idle is
         * true, the loop does only one step, and only one bit is cleared.
         */
        for_each_cpu(cpu, &mask)
            cpumask_clear_cpu(cpu, prv->idlers);
        cpumask_raise_softirq(&mask, SCHEDULE_SOFTIRQ);
    }
    else
        SCHED_STAT_CRANK(tickled_no_cpu);
}

static void
csched_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct csched_private *prv = CSCHED_PRIV(ops);

    /*
     * pcpu either points to a valid struct csched_pcpu, or is NULL, if we're
     * beeing called from CPU_UP_CANCELLED, because bringing up a pCPU failed
     * very early. xfree() does not really mind, but we want to be sure that,
     * when we get here, either init_pdata has never been called, or
     * deinit_pdata has been called already.
     */
    ASSERT(!cpumask_test_cpu(cpu, prv->cpus));

    xfree(pcpu);
}

static void
csched_deinit_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct csched_private *prv = CSCHED_PRIV(ops);
    struct csched_pcpu *spc = pcpu;
    unsigned int node = cpu_to_node(cpu);
    unsigned long flags;

    /*
     * Scheduler specific data for this pCPU must still be there and and be
     * valid. In fact, if we are here:
     *  1. alloc_pdata must have been called for this cpu, and free_pdata
     *     must not have been called on it before us,
     *  2. init_pdata must have been called on this cpu, and deinit_pdata
     *     (us!) must not have been called on it already.
     */
    ASSERT(spc && cpumask_test_cpu(cpu, prv->cpus));

    spin_lock_irqsave(&prv->lock, flags);

    prv->credit -= prv->credits_per_tslice;
    prv->ncpus--;
    cpumask_clear_cpu(cpu, prv->idlers);
    cpumask_clear_cpu(cpu, prv->cpus);
    if ( (prv->master == cpu) && (prv->ncpus > 0) )
    {
        prv->master = cpumask_first(prv->cpus);
        migrate_timer(&prv->master_ticker, prv->master);
    }
    if ( prv->balance_bias[node] == cpu )
    {
        cpumask_and(cpumask_scratch, prv->cpus, &node_to_cpumask(node));
        if ( !cpumask_empty(cpumask_scratch) )
            prv->balance_bias[node] =  cpumask_first(cpumask_scratch);
    }
    kill_timer(&spc->ticker);
    if ( prv->ncpus == 0 )
        kill_timer(&prv->master_ticker);

    spin_unlock_irqrestore(&prv->lock, flags);
}

static void *
csched_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct csched_pcpu *spc;

    /* Allocate per-PCPU info */
    spc = xzalloc(struct csched_pcpu);
    if ( spc == NULL )
        return ERR_PTR(-ENOMEM);

    return spc;
}

static void
init_pdata(struct csched_private *prv, struct csched_pcpu *spc, int cpu)
{
    ASSERT(spin_is_locked(&prv->lock));
    /* cpu data needs to be allocated, but STILL uninitialized. */
    ASSERT(spc && spc->runq.next == NULL && spc->runq.prev == NULL);

    /* Initialize/update system-wide config */
    prv->credit += prv->credits_per_tslice;
    prv->ncpus++;
    cpumask_set_cpu(cpu, prv->cpus);
    if ( prv->ncpus == 1 )
    {
        prv->master = cpu;
        init_timer(&prv->master_ticker, csched_acct, prv, cpu);
        set_timer(&prv->master_ticker, NOW() + prv->tslice);
    }

    cpumask_and(cpumask_scratch, prv->cpus, &node_to_cpumask(cpu_to_node(cpu)));
    if ( cpumask_weight(cpumask_scratch) == 1 )
        prv->balance_bias[cpu_to_node(cpu)] = cpu;

    init_timer(&spc->ticker, csched_tick, (void *)(unsigned long)cpu, cpu);
    set_timer(&spc->ticker, NOW() + MICROSECS(prv->tick_period_us) );

    INIT_LIST_HEAD(&spc->runq);
    spc->runq_sort_last = prv->runq_sort;
    spc->idle_bias = nr_cpu_ids - 1;

    /* Start off idling... */
    BUG_ON(!is_idle_vcpu(curr_on_cpu(cpu)));
    cpumask_set_cpu(cpu, prv->idlers);
    spc->nr_runnable = 0;

    spc->cpu = cpu;
    spc->nr_running = 0;

    init_cfs_rq(spc);
}

static void
csched_init_pdata(const struct scheduler *ops, void *pdata, int cpu)
{
    unsigned long flags;
    struct csched_private *prv = CSCHED_PRIV(ops);
    struct schedule_data *sd = &per_cpu(schedule_data, cpu);

    /*
     * This is called either during during boot, resume or hotplug, in
     * case Credit1 is the scheduler chosen at boot. In such cases, the
     * scheduler lock for cpu is already pointing to the default per-cpu
     * spinlock, as Credit1 needs it, so there is no remapping to be done.
     */
    ASSERT(sd->schedule_lock == &sd->_lock && !spin_is_locked(&sd->_lock));

    spin_lock_irqsave(&prv->lock, flags);
    init_pdata(prv, pdata, cpu);
    spin_unlock_irqrestore(&prv->lock, flags);
}

/* Change the scheduler of cpu to us (Credit). */
static void
csched_switch_sched(struct scheduler *new_ops, unsigned int cpu,
                    void *pdata, void *vdata)
{
    struct schedule_data *sd = &per_cpu(schedule_data, cpu);
    struct csched_private *prv = CSCHED_PRIV(new_ops);
    struct csched_vcpu *svc = vdata;

    ASSERT(svc && is_idle_vcpu(svc->vcpu));

    idle_vcpu[cpu]->sched_priv = vdata;

    /*
     * We are holding the runqueue lock already (it's been taken in
     * schedule_cpu_switch()). It actually may or may not be the 'right'
     * one for this cpu, but that is ok for preventing races.
     */
    ASSERT(!local_irq_is_enabled());
    spin_lock(&prv->lock);
    init_pdata(prv, pdata, cpu);
    spin_unlock(&prv->lock);

    per_cpu(scheduler, cpu) = new_ops;
    per_cpu(schedule_data, cpu).sched_priv = pdata;

    /*
     * (Re?)route the lock to the per pCPU lock as /last/ thing. In fact,
     * if it is free (and it can be) we want that anyone that manages
     * taking it, finds all the initializations we've done above in place.
     */
    smp_mb();
    sd->schedule_lock = &sd->_lock;
}

#ifndef NDEBUG
static inline void
__csched_vcpu_check(struct vcpu *vc)
{
    struct csched_vcpu * const svc = CSCHED_VCPU(vc);
    struct csched_dom * const sdom = svc->sdom;

    BUG_ON( svc->vcpu != vc );
    BUG_ON( sdom != CSCHED_DOM(vc->domain) );
    if ( sdom )
    {
        BUG_ON( is_idle_vcpu(vc) );
        BUG_ON( sdom->dom != vc->domain );
    }
    else
    {
        BUG_ON( !is_idle_vcpu(vc) );
    }

    SCHED_STAT_CRANK(vcpu_check);
}
#define CSCHED_VCPU_CHECK(_vc)  (__csched_vcpu_check(_vc))
#else
#define CSCHED_VCPU_CHECK(_vc)
#endif

/*
 * Delay, in microseconds, between migrations of a VCPU between PCPUs.
 * This prevents rapid fluttering of a VCPU between CPUs, and reduces the
 * implicit overheads such as cache-warming. 1ms (1000) has been measured
 * as a good value.
 */
static unsigned int vcpu_migration_delay_us;
integer_param("vcpu_migration_delay", vcpu_migration_delay_us);

static inline bool
__csched_vcpu_is_cache_hot(const struct csched_private *prv, struct vcpu *v)
{
    bool hot = prv->vcpu_migr_delay &&
               (NOW() - v->last_run_time) < prv->vcpu_migr_delay;

    if ( hot )
        SCHED_STAT_CRANK(vcpu_hot);

    return hot;
}

static inline int
__csched_vcpu_is_migrateable(const struct csched_private *prv, struct vcpu *vc,
                             int dest_cpu, cpumask_t *mask)
{
    /*
     * Don't pick up work that's hot on peer PCPU, or that can't (or
     * would prefer not to) run on cpu.
     *
     * The caller is supposed to have already checked that vc is also
     * not running.
     */
    ASSERT(!vc->is_running);

    return !__csched_vcpu_is_cache_hot(prv, vc) &&
           cpumask_test_cpu(dest_cpu, mask);
}

static int
_csched_cpu_pick(const struct scheduler *ops, struct vcpu *vc, bool_t commit)
{
    /* We must always use vc->procssor's scratch space */
    cpumask_t *cpus = cpumask_scratch_cpu(vc->processor);
    cpumask_t idlers;
    cpumask_t *online = cpupool_domain_cpumask(vc->domain);
    struct csched_pcpu *spc = NULL;
    int cpu = vc->processor;
    int balance_step;

    for_each_affinity_balance_step( balance_step )
    {
        affinity_balance_cpumask(vc, balance_step, cpus);
        cpumask_and(cpus, online, cpus);
        /*
         * We want to pick up a pcpu among the ones that are online and
         * can accommodate vc. As far as hard affinity is concerned, there
         * always will be at least one of these pcpus in the scratch cpumask,
         * hence, the calls to cpumask_cycle() and cpumask_test_cpu() below
         * are ok.
         *
         * On the other hand, when considering soft affinity, it is possible
         * that the mask is empty (for instance, if the domain has been put
         * in a cpupool that does not contain any of the pcpus in its soft
         * affinity), which would result in the ASSERT()-s inside cpumask_*()
         * operations triggering (in debug builds).
         *
         * Therefore, if that is the case, we just skip the soft affinity
         * balancing step all together.
         */
        if ( balance_step == BALANCE_SOFT_AFFINITY &&
             (!has_soft_affinity(vc) || cpumask_empty(cpus)) )
            continue;

        /* If present, prefer vc's current processor */
        cpu = cpumask_test_cpu(vc->processor, cpus)
                ? vc->processor : cpumask_cycle(vc->processor, cpus);
        ASSERT(cpumask_test_cpu(cpu, cpus));

        /*
         * Try to find an idle processor within the above constraints.
         *
         * In multi-core and multi-threaded CPUs, not all idle execution
         * vehicles are equal!
         *
         * We give preference to the idle execution vehicle with the most
         * idling neighbours in its grouping. This distributes work across
         * distinct cores first and guarantees we don't do something stupid
         * like run two VCPUs on co-hyperthreads while there are idle cores
         * or sockets.
         *
         * Notice that, when computing the "idleness" of cpu, we may want to
         * discount vc. That is, iff vc is the currently running and the only
         * runnable vcpu on cpu, we add cpu to the idlers.
         */
        cpumask_and(&idlers, &cpu_online_map, CSCHED_PRIV(ops)->idlers);
        if ( vc->processor == cpu && is_runq_idle(cpu) )
            __cpumask_set_cpu(cpu, &idlers);
        cpumask_and(cpus, &idlers, cpus);

        /*
         * It is important that cpu points to an idle processor, if a suitable
         * one exists (and we can use cpus to check and, possibly, choose a new
         * CPU, as we just &&-ed it with idlers). In fact, if we are on SMT, and
         * cpu points to a busy thread with an idle sibling, both the threads
         * will be considered the same, from the "idleness" calculation point
         * of view", preventing vcpu from being moved to the thread that is
         * actually idle.
         *
         * Notice that cpumask_test_cpu() is quicker than cpumask_empty(), so
         * we check for it first.
         */
        if ( !cpumask_test_cpu(cpu, cpus) && !cpumask_empty(cpus) )
            cpu = cpumask_cycle(cpu, cpus);
        __cpumask_clear_cpu(cpu, cpus);

        while ( !cpumask_empty(cpus) )
        {
            cpumask_t cpu_idlers;
            cpumask_t nxt_idlers;
            int nxt, weight_cpu, weight_nxt;
            int migrate_factor;

            nxt = cpumask_cycle(cpu, cpus);

            if ( cpumask_test_cpu(cpu, per_cpu(cpu_core_mask, nxt)) )
            {
                /* We're on the same socket, so check the busy-ness of threads.
                 * Migrate if # of idlers is less at all */
                ASSERT( cpumask_test_cpu(nxt, per_cpu(cpu_core_mask, cpu)) );
                migrate_factor = 1;
                cpumask_and(&cpu_idlers, &idlers, per_cpu(cpu_sibling_mask,
                            cpu));
                cpumask_and(&nxt_idlers, &idlers, per_cpu(cpu_sibling_mask,
                            nxt));
            }
            else
            {
                /* We're on different sockets, so check the busy-ness of cores.
                 * Migrate only if the other core is twice as idle */
                ASSERT( !cpumask_test_cpu(nxt, per_cpu(cpu_core_mask, cpu)) );
                migrate_factor = 2;
                cpumask_and(&cpu_idlers, &idlers, per_cpu(cpu_core_mask, cpu));
                cpumask_and(&nxt_idlers, &idlers, per_cpu(cpu_core_mask, nxt));
            }

            weight_cpu = cpumask_weight(&cpu_idlers);
            weight_nxt = cpumask_weight(&nxt_idlers);
            /* smt_power_savings: consolidate work rather than spreading it */
            if ( sched_smt_power_savings ?
                 weight_cpu > weight_nxt :
                 weight_cpu * migrate_factor < weight_nxt )
            {
                cpumask_and(&nxt_idlers, &nxt_idlers, cpus);
                spc = CSCHED_PCPU(nxt);
                cpu = cpumask_cycle(spc->idle_bias, &nxt_idlers);
                cpumask_andnot(cpus, cpus, per_cpu(cpu_sibling_mask, cpu));
            }
            else
            {
                cpumask_andnot(cpus, cpus, &nxt_idlers);
            }
        }

        /* Stop if cpu is idle */
        if ( cpumask_test_cpu(cpu, &idlers) )
            break;
    }

    if ( commit && spc )
       spc->idle_bias = cpu;

    TRACE_3D(TRC_CSCHED_PICKED_CPU, vc->domain->domain_id, vc->vcpu_id, cpu);

    return cpu;
}

static int
csched_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_vcpu *svc = CSCHED_VCPU(vc);

    /*
     * We have been called by vcpu_migrate() (in schedule.c), as part
     * of the process of seeing if vc can be migrated to another pcpu.
     * We make a note about this in svc->flags so that later, in
     * csched_vcpu_wake() (still called from vcpu_migrate()) we won't
     * get boosted, which we don't deserve as we are "only" migrating.
     */
    set_bit(CSCHED_FLAG_VCPU_MIGRATING, &svc->flags);
    return _csched_cpu_pick(ops, vc, 1);
}

static inline void
__csched_vcpu_acct_start(struct csched_private *prv, struct csched_vcpu *svc)
{
    struct csched_dom * const sdom = svc->sdom;
    unsigned long flags;

    spin_lock_irqsave(&prv->lock, flags);

    if ( list_empty(&svc->active_vcpu_elem) )
    {
        SCHED_VCPU_STAT_CRANK(svc, state_active);
        SCHED_STAT_CRANK(acct_vcpu_active);

        sdom->active_vcpu_count++;
        list_add(&svc->active_vcpu_elem, &sdom->active_vcpu);
        /* Make weight per-vcpu */
        prv->weight += sdom->weight;
        if ( list_empty(&sdom->active_sdom_elem) )
        {
            list_add(&sdom->active_sdom_elem, &prv->active_sdom);
        }
    }

    TRACE_3D(TRC_CSCHED_ACCOUNT_START, sdom->dom->domain_id,
             svc->vcpu->vcpu_id, sdom->active_vcpu_count);

    spin_unlock_irqrestore(&prv->lock, flags);
}

static inline void
__csched_vcpu_acct_stop_locked(struct csched_private *prv,
    struct csched_vcpu *svc)
{
    struct csched_dom * const sdom = svc->sdom;

    BUG_ON( list_empty(&svc->active_vcpu_elem) );

    SCHED_VCPU_STAT_CRANK(svc, state_idle);
    SCHED_STAT_CRANK(acct_vcpu_idle);

    BUG_ON( prv->weight < sdom->weight );
    sdom->active_vcpu_count--;
    list_del_init(&svc->active_vcpu_elem);
    prv->weight -= sdom->weight;
    if ( list_empty(&sdom->active_vcpu) )
    {
        list_del_init(&sdom->active_sdom_elem);
    }

    //dequeue_task_fair(NULL, svc, 0);

    TRACE_3D(TRC_CSCHED_ACCOUNT_STOP, sdom->dom->domain_id,
             svc->vcpu->vcpu_id, sdom->active_vcpu_count);
}

static void
csched_vcpu_acct(struct csched_private *prv, unsigned int cpu)
{
    struct csched_vcpu * const svc = CSCHED_VCPU(current);
    const struct scheduler *ops = per_cpu(scheduler, cpu);

    ASSERT( current->processor == cpu );
    ASSERT( svc->sdom != NULL );
    ASSERT( !is_idle_vcpu(svc->vcpu) );

    /*
     * If this VCPU's priority was boosted when it last awoke, reset it.
     * If the VCPU is found here, then it's consuming a non-negligeable
     * amount of CPU resources and should no longer be boosted.
     */
    if ( svc->pri == CSCHED_PRI_TS_BOOST )
    {
        svc->pri = CSCHED_PRI_TS_UNDER;
        TRACE_2D(TRC_CSCHED_BOOST_END, svc->sdom->dom->domain_id,
                 svc->vcpu->vcpu_id);
    }

    /*
     * Update credits
     */
    burn_credits(svc, NOW());

    /*
     * Put this VCPU and domain back on the active list if it was
     * idling.
     */
    if ( list_empty(&svc->active_vcpu_elem) )
    {
        __csched_vcpu_acct_start(prv, svc);
    }
    else
    {
        unsigned int new_cpu;
        unsigned long flags;
        spinlock_t *lock = vcpu_schedule_lock_irqsave(current, &flags);

        /*
         * If it's been active a while, check if we'd be better off
         * migrating it to run elsewhere (see multi-core and multi-thread
         * support in csched_cpu_pick()).
         */
        new_cpu = _csched_cpu_pick(ops, current, 0);

        vcpu_schedule_unlock_irqrestore(lock, flags, current);

        if ( new_cpu != cpu )
        {
            SCHED_VCPU_STAT_CRANK(svc, migrate_r);
            SCHED_STAT_CRANK(migrate_running);
            set_bit(_VPF_migrating, &current->pause_flags);
            /*
             * As we are about to tickle cpu, we should clear its bit in
             * idlers. But, if we are here, it means there is someone running
             * on it, and hence the bit must be zero already.
             */
            ASSERT(!cpumask_test_cpu(cpu,
                                     CSCHED_PRIV(per_cpu(scheduler, cpu))->idlers));
            cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
        }
    }
}

static void *
csched_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct csched_vcpu *svc;

    /* Allocate per-VCPU info */
    svc = xzalloc(struct csched_vcpu);
    if ( svc == NULL )
        return NULL;

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->active_vcpu_elem);
    svc->sdom = dd;
    svc->vcpu = vc;
    svc->pri = is_idle_domain(vc->domain) ?
        CSCHED_PRI_IDLE : CSCHED_PRI_TS_UNDER;
    SCHED_VCPU_STATS_RESET(svc);
    SCHED_STAT_CRANK(vcpu_alloc);

    svc->on_rq = 0;

    return svc;
}

static void
csched_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_vcpu *svc = vc->sched_priv;
    spinlock_t *lock;

    BUG_ON( is_idle_vcpu(vc) );

    /* csched_cpu_pick() looks in vc->processor's runq, so we need the lock. */
    lock = vcpu_schedule_lock_irq(vc);

    vc->processor = csched_cpu_pick(ops, vc);

    spin_unlock_irq(lock);

    svc->load.weight = svc->sdom->load.weight;
    svc->load.inv_weight = svc->sdom->load.inv_weight;

    lock = vcpu_schedule_lock_irq(vc);

    if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running ) 
    {
        runq_insert(svc);
        enqueue_task_fair(NULL, svc, 0);
    }

    vcpu_schedule_unlock_irq(lock, vc);

    SCHED_STAT_CRANK(vcpu_insert);
}

static void
csched_free_vdata(const struct scheduler *ops, void *priv)
{
    struct csched_vcpu *svc = priv;

    BUG_ON( !list_empty(&svc->runq_elem) );

    xfree(svc);
}

static void
csched_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_private *prv = CSCHED_PRIV(ops);
    struct csched_vcpu * const svc = CSCHED_VCPU(vc);
    struct csched_dom * const sdom = svc->sdom;

    SCHED_STAT_CRANK(vcpu_remove);

    ASSERT(!__vcpu_on_runq(svc));

    if ( test_and_clear_bit(CSCHED_FLAG_VCPU_PARKED, &svc->flags) )
    {
        SCHED_STAT_CRANK(vcpu_unpark);
        vcpu_unpause(svc->vcpu);
    }

    spin_lock_irq(&prv->lock);

    if ( !list_empty(&svc->active_vcpu_elem) )
        __csched_vcpu_acct_stop_locked(prv, svc);

    spin_unlock_irq(&prv->lock);

    BUG_ON( sdom == NULL );
}

static void
csched_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_vcpu * const svc = CSCHED_VCPU(vc);
    unsigned int cpu = vc->processor;

    SCHED_STAT_CRANK(vcpu_sleep);

    BUG_ON( is_idle_vcpu(vc) );

    if ( curr_on_cpu(cpu) == vc )
    {
        /*
         * We are about to tickle cpu, so we should clear its bit in idlers.
         * But, we are here because vc is going to sleep while running on cpu,
         * so the bit must be zero already.
         */
        ASSERT(!cpumask_test_cpu(cpu, CSCHED_PRIV(per_cpu(scheduler, cpu))->idlers));
        cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
    }
    else if ( __vcpu_on_runq(svc) )
    {
        runq_remove(svc);
        dequeue_task_fair(NULL, svc, DEQUEUE_SLEEP);
    }
}

static void
csched_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_vcpu * const svc = CSCHED_VCPU(vc);
    bool_t migrating;

    BUG_ON( is_idle_vcpu(vc) );

    if ( unlikely(curr_on_cpu(vc->processor) == vc) )
    {
        SCHED_STAT_CRANK(vcpu_wake_running);
        return;
    }
    if ( unlikely(__vcpu_on_runq(svc)) )
    {
        SCHED_STAT_CRANK(vcpu_wake_onrunq);
        return;
    }

    if ( likely(vcpu_runnable(vc)) )
        SCHED_STAT_CRANK(vcpu_wake_runnable);
    else
        SCHED_STAT_CRANK(vcpu_wake_not_runnable);

    /*
     * We temporarly boost the priority of awaking VCPUs!
     *
     * If this VCPU consumes a non negligeable amount of CPU, it
     * will eventually find itself in the credit accounting code
     * path where its priority will be reset to normal.
     *
     * If on the other hand the VCPU consumes little CPU and is
     * blocking and awoken a lot (doing I/O for example), its
     * priority will remain boosted, optimizing it's wake-to-run
     * latencies.
     *
     * This allows wake-to-run latency sensitive VCPUs to preempt
     * more CPU resource intensive VCPUs without impacting overall 
     * system fairness.
     *
     * There are two cases, when we don't want to boost:
     *  - VCPUs that are waking up after a migration, rather than
     *    after having block;
     *  - VCPUs of capped domains unpausing after earning credits
     *    they had overspent.
     */
    migrating = test_and_clear_bit(CSCHED_FLAG_VCPU_MIGRATING, &svc->flags);

    if ( !migrating && svc->pri == CSCHED_PRI_TS_UNDER &&
         !test_bit(CSCHED_FLAG_VCPU_PARKED, &svc->flags) )
    {
        TRACE_2D(TRC_CSCHED_BOOST_START, vc->domain->domain_id, vc->vcpu_id);
        SCHED_STAT_CRANK(vcpu_boost);
        svc->pri = CSCHED_PRI_TS_BOOST;
    }

    /* Put the VCPU on the runq and tickle CPUs */
    runq_insert(svc);
    enqueue_task_fair(NULL, svc, ENQUEUE_WAKEUP | (migrating? ENQUEUE_MIGRATED:0));
    entity_tick(CSCHED_PCPU(svc->vcpu->processor), svc, 0);
    __runq_tickle(svc);
}

static void
csched_vcpu_yield(const struct scheduler *ops, struct vcpu *vc)
{
    struct csched_vcpu * const svc = CSCHED_VCPU(vc);

    /* Let the scheduler know that this vcpu is trying to yield */
    set_bit(CSCHED_FLAG_VCPU_YIELD, &svc->flags);
}


static int
csched_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
    struct xen_domctl_scheduler_op *op)
{
    struct csched_dom * const sdom = CSCHED_DOM(d);
    struct csched_private *prv = CSCHED_PRIV(ops);
    unsigned long flags;
    int rc = 0;

    /* Protect both get and put branches with the pluggable scheduler
     * lock. Runq lock not needed anywhere in here. */
    spin_lock_irqsave(&prv->lock, flags);

    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_getinfo:
        op->u.credit.weight = sdom->weight;
        op->u.credit.cap = sdom->cap;
        break;
    case XEN_DOMCTL_SCHEDOP_putinfo:
        if ( op->u.credit.weight != 0 )
        {
            if ( !list_empty(&sdom->active_sdom_elem) )
            {
                prv->weight -= sdom->weight * sdom->active_vcpu_count;
                prv->weight += op->u.credit.weight * sdom->active_vcpu_count;
            }
            sdom->weight = op->u.credit.weight;
            if(sdom->weight <= 39)
            {
                (sdom->load).weight = scale_load(sched_prio_to_weight[sdom->weight]);
	            (sdom->load).inv_weight = sched_prio_to_wmult[sdom->weight];
            }
            else
            {
                (sdom->load).weight = scale_load(sched_prio_to_weight[19]);
                (sdom->load).inv_weight = sched_prio_to_wmult[19];
            }
        }

        if ( op->u.credit.cap != (uint16_t)~0U )
            sdom->cap = op->u.credit.cap;
        break;
    default:
        rc = -EINVAL;
        break;
    }

    spin_unlock_irqrestore(&prv->lock, flags);

    return rc;
}

static void
csched_aff_cntl(const struct scheduler *ops, struct vcpu *v,
                const cpumask_t *hard, const cpumask_t *soft)
{
    struct csched_vcpu *svc = CSCHED_VCPU(v);

    if ( !hard )
        return;

    /* Are we becoming exclusively pinned? */
    if ( cpumask_weight(hard) == 1 )
        set_bit(CSCHED_FLAG_VCPU_PINNED, &svc->flags);
    else
        clear_bit(CSCHED_FLAG_VCPU_PINNED, &svc->flags);
}

static inline void
__csched_set_tslice(struct csched_private *prv, unsigned int timeslice_ms)
{
    prv->tslice = MILLISECS(timeslice_ms);
    prv->ticks_per_tslice = CSCHED_TICKS_PER_TSLICE;
    if ( timeslice_ms < prv->ticks_per_tslice )
        prv->ticks_per_tslice = 1;
    prv->tick_period_us = timeslice_ms * 1000 / prv->ticks_per_tslice;
    prv->credits_per_tslice = CSCHED_CREDITS_PER_MSEC * timeslice_ms;
    prv->credit = prv->credits_per_tslice * prv->ncpus;
}

static int
csched_sys_cntl(const struct scheduler *ops,
                        struct xen_sysctl_scheduler_op *sc)
{
    int rc = -EINVAL;
    struct xen_sysctl_credit_schedule *params = &sc->u.sched_credit;
    struct csched_private *prv = CSCHED_PRIV(ops);
    unsigned long flags;

    switch ( sc->cmd )
    {
    case XEN_SYSCTL_SCHEDOP_putinfo:
        if ( params->tslice_ms > XEN_SYSCTL_CSCHED_TSLICE_MAX
             || params->tslice_ms < XEN_SYSCTL_CSCHED_TSLICE_MIN
             || (params->ratelimit_us
                 && (params->ratelimit_us > XEN_SYSCTL_SCHED_RATELIMIT_MAX
                     || params->ratelimit_us < XEN_SYSCTL_SCHED_RATELIMIT_MIN))
             || MICROSECS(params->ratelimit_us) > MILLISECS(params->tslice_ms)
             || params->vcpu_migr_delay_us > XEN_SYSCTL_CSCHED_MGR_DLY_MAX_US )
                goto out;

        spin_lock_irqsave(&prv->lock, flags);
        __csched_set_tslice(prv, params->tslice_ms);
        if ( !prv->ratelimit && params->ratelimit_us )
            printk(XENLOG_INFO "Enabling context switch rate limiting\n");
        else if ( prv->ratelimit && !params->ratelimit_us )
            printk(XENLOG_INFO "Disabling context switch rate limiting\n");
        prv->ratelimit = MICROSECS(params->ratelimit_us);
        prv->vcpu_migr_delay = MICROSECS(params->vcpu_migr_delay_us);
        spin_unlock_irqrestore(&prv->lock, flags);

        /* FALLTHRU */
    case XEN_SYSCTL_SCHEDOP_getinfo:
        params->tslice_ms = prv->tslice / MILLISECS(1);
        params->ratelimit_us = prv->ratelimit / MICROSECS(1);
        params->vcpu_migr_delay_us = prv->vcpu_migr_delay / MICROSECS(1);
        rc = 0;
        break;
    }
    out:
    return rc;
}

static void *
csched_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    struct csched_dom *sdom;

    sdom = xzalloc(struct csched_dom);
    if ( sdom == NULL )
        return ERR_PTR(-ENOMEM);

    /* Initialize credit and weight */
    INIT_LIST_HEAD(&sdom->active_vcpu);
    INIT_LIST_HEAD(&sdom->active_sdom_elem);
    sdom->dom = dom;
    sdom->weight = CSCHED_DEFAULT_WEIGHT;

    // set default weight to 1024
    (sdom->load).weight = scale_load(sched_prio_to_weight[19]);
	(sdom->load).inv_weight = sched_prio_to_wmult[19];

    return sdom;
}

static void
csched_free_domdata(const struct scheduler *ops, void *data)
{
    xfree(data);
}

/*
 * This is a O(n) optimized sort of the runq.
 *
 * Time-share VCPUs can only be one of two priorities, UNDER or OVER. We walk
 * through the runq and move up any UNDERs that are preceded by OVERS. We
 * remember the last UNDER to make the move up operation O(1).
 */
static void
csched_runq_sort(struct csched_private *prv, unsigned int cpu)
{
    struct csched_pcpu * const spc = CSCHED_PCPU(cpu);
    struct list_head *runq, *elem, *next, *last_under;
    struct csched_vcpu *svc_elem;
    spinlock_t *lock;
    unsigned long flags;
    int sort_epoch;

    sort_epoch = prv->runq_sort;
    if ( sort_epoch == spc->runq_sort_last )
        return;

    spc->runq_sort_last = sort_epoch;

    lock = pcpu_schedule_lock_irqsave(cpu, &flags);

    runq = &spc->runq;
    elem = runq->next;
    last_under = runq;

    while ( elem != runq )
    {
        next = elem->next;
        svc_elem = __runq_elem(elem);

        if ( svc_elem->pri >= CSCHED_PRI_TS_UNDER )
        {
            /* does elem need to move up the runq? */
            if ( elem->prev != last_under )
            {
                list_del(elem);
                list_add(elem, last_under);
            }
            last_under = elem;
        }

        elem = next;
    }

    pcpu_schedule_unlock_irqrestore(lock, flags, cpu);
}

static void
csched_acct(void* dummy)
{
    struct csched_private *prv = dummy;
    unsigned long flags;
    struct list_head *iter_vcpu, *next_vcpu;
    struct list_head *iter_sdom, *next_sdom;
    struct csched_vcpu *svc;
    struct csched_dom *sdom;
    uint32_t credit_total;
    uint32_t weight_total;
    uint32_t weight_left;
    uint32_t credit_fair;
    uint32_t credit_peak;
    uint32_t credit_cap;
    int credit_balance;
    int credit_xtra;
    int credit;


    spin_lock_irqsave(&prv->lock, flags);

    weight_total = prv->weight;
    credit_total = prv->credit;

    /* Converge balance towards 0 when it drops negative */
    if ( prv->credit_balance < 0 )
    {
        credit_total -= prv->credit_balance;
        SCHED_STAT_CRANK(acct_balance);
    }

    if ( unlikely(weight_total == 0) )
    {
        prv->credit_balance = 0;
        spin_unlock_irqrestore(&prv->lock, flags);
        SCHED_STAT_CRANK(acct_no_work);
        goto out;
    }

    SCHED_STAT_CRANK(acct_run);

    weight_left = weight_total;
    credit_balance = 0;
    credit_xtra = 0;
    credit_cap = 0U;

    list_for_each_safe( iter_sdom, next_sdom, &prv->active_sdom )
    {
        sdom = list_entry(iter_sdom, struct csched_dom, active_sdom_elem);

        BUG_ON( is_idle_domain(sdom->dom) );
        BUG_ON( sdom->active_vcpu_count == 0 );
        BUG_ON( sdom->weight == 0 );
        BUG_ON( (sdom->weight * sdom->active_vcpu_count) > weight_left );

        weight_left -= ( sdom->weight * sdom->active_vcpu_count );

        /*
         * A domain's fair share is computed using its weight in competition
         * with that of all other active domains.
         *
         * At most, a domain can use credits to run all its active VCPUs
         * for one full accounting period. We allow a domain to earn more
         * only when the system-wide credit balance is negative.
         */
        credit_peak = sdom->active_vcpu_count * prv->credits_per_tslice;
        if ( prv->credit_balance < 0 )
        {
            credit_peak += ( ( -prv->credit_balance
                               * sdom->weight
                               * sdom->active_vcpu_count) +
                             (weight_total - 1)
                           ) / weight_total;
        }

        if ( sdom->cap != 0U )
        {
            credit_cap = ((sdom->cap * prv->credits_per_tslice) + 99) / 100;
            if ( credit_cap < credit_peak )
                credit_peak = credit_cap;

            /* FIXME -- set cap per-vcpu as well...? */
            credit_cap = ( credit_cap + ( sdom->active_vcpu_count - 1 )
                         ) / sdom->active_vcpu_count;
        }

        credit_fair = ( ( credit_total
                          * sdom->weight
                          * sdom->active_vcpu_count )
                        + (weight_total - 1)
                      ) / weight_total;

        if ( credit_fair < credit_peak )
        {
            credit_xtra = 1;
        }
        else
        {
            if ( weight_left != 0U )
            {
                /* Give other domains a chance at unused credits */
                credit_total += ( ( ( credit_fair - credit_peak
                                    ) * weight_total
                                  ) + ( weight_left - 1 )
                                ) / weight_left;
            }

            if ( credit_xtra )
            {
                /*
                 * Lazily keep domains with extra credits at the head of
                 * the queue to give others a chance at them in future
                 * accounting periods.
                 */
                SCHED_STAT_CRANK(acct_reorder);
                list_del(&sdom->active_sdom_elem);
                list_add(&sdom->active_sdom_elem, &prv->active_sdom);
            }

            credit_fair = credit_peak;
        }

        /* Compute fair share per VCPU */
        credit_fair = ( credit_fair + ( sdom->active_vcpu_count - 1 )
                      ) / sdom->active_vcpu_count;


        list_for_each_safe( iter_vcpu, next_vcpu, &sdom->active_vcpu )
        {
            svc = list_entry(iter_vcpu, struct csched_vcpu, active_vcpu_elem);
            BUG_ON( sdom != svc->sdom );

            /* Increment credit */
            atomic_add(credit_fair, &svc->credit);
            credit = atomic_read(&svc->credit);

            /*
             * Recompute priority or, if VCPU is idling, remove it from
             * the active list.
             */
            if ( credit < 0 )
            {
                svc->pri = CSCHED_PRI_TS_OVER;

                /* Park running VCPUs of capped-out domains */
                if ( sdom->cap != 0U &&
                     credit < -credit_cap &&
                     !test_and_set_bit(CSCHED_FLAG_VCPU_PARKED, &svc->flags) )
                {
                    SCHED_STAT_CRANK(vcpu_park);
                    vcpu_pause_nosync(svc->vcpu);
                }

                /* Lower bound on credits */
                if ( credit < -prv->credits_per_tslice )
                {
                    SCHED_STAT_CRANK(acct_min_credit);
                    credit = -prv->credits_per_tslice;
                    atomic_set(&svc->credit, credit);
                }
            }
            else
            {
                svc->pri = CSCHED_PRI_TS_UNDER;

                /* Unpark any capped domains whose credits go positive */
                if ( test_and_clear_bit(CSCHED_FLAG_VCPU_PARKED, &svc->flags) )
                {
                    /*
                     * It's important to unset the flag AFTER the unpause()
                     * call to make sure the VCPU's priority is not boosted
                     * if it is woken up here.
                     */
                    SCHED_STAT_CRANK(vcpu_unpark);
                    vcpu_unpause(svc->vcpu);
                }

                /* Upper bound on credits means VCPU stops earning */
                if ( credit > prv->credits_per_tslice )
                {
                    __csched_vcpu_acct_stop_locked(prv, svc);
                    /* Divide credits in half, so that when it starts
                     * accounting again, it starts a little bit "ahead" */
                    credit /= 2;
                    atomic_set(&svc->credit, credit);
                }
            }

            SCHED_VCPU_STAT_SET(svc, credit_last, credit);
            SCHED_VCPU_STAT_SET(svc, credit_incr, credit_fair);
            credit_balance += credit;
        }
    }

    prv->credit_balance = credit_balance;

    spin_unlock_irqrestore(&prv->lock, flags);

    /* Inform each CPU that its runq needs to be sorted */
    prv->runq_sort++;

out:
    set_timer( &prv->master_ticker, NOW() + prv->tslice);
}

static void
csched_tick(void *_cpu)
{
    unsigned int cpu = (unsigned long)_cpu;
    struct csched_pcpu *spc = CSCHED_PCPU(cpu);
    struct csched_private *prv = CSCHED_PRIV(per_cpu(scheduler, cpu));

    spc->tick++;

    /*
     * Accounting for running VCPU
     */
    if ( !is_idle_vcpu(current) ){
        csched_vcpu_acct(prv, cpu);
        entity_tick(CSCHED_PCPU(cpu), CSCHED_PCPU(cpu)->curr, 1);
    }

    /*
     * Check if runq needs to be sorted
     *
     * Every physical CPU resorts the runq after the accounting master has
     * modified priorities. This is a special O(n) sort and runs at most
     * once per accounting period (currently 30 milliseconds).
     */
    csched_runq_sort(prv, cpu);

    set_timer(&spc->ticker, NOW() + MICROSECS(prv->tick_period_us) );
}

static struct csched_vcpu *
csched_runq_steal(int peer_cpu, int cpu, int pri, int balance_step)
{
    const struct csched_private * const prv = CSCHED_PRIV(per_cpu(scheduler, cpu));
    const struct csched_pcpu * const peer_pcpu = CSCHED_PCPU(peer_cpu);
    struct csched_vcpu *speer;
    struct list_head *iter;
    struct vcpu *vc;

    ASSERT(peer_pcpu != NULL);

    /*
     * Don't steal from an idle CPU's runq because it's about to
     * pick up work from it itself.
     */
    if ( unlikely(is_idle_vcpu(curr_on_cpu(peer_cpu))) )
        goto out;

    list_for_each( iter, &peer_pcpu->runq )
    {
        speer = __runq_elem(iter);

        /*
         * If next available VCPU here is not of strictly higher
         * priority than ours, this PCPU is useless to us.
         */
        if ( speer->pri <= pri )
            break;

        /* Is this VCPU runnable on our PCPU? */
        vc = speer->vcpu;
        BUG_ON( is_idle_vcpu(vc) );

        /*
         * If the vcpu is still in peer_cpu's scheduling tail, or if it
         * has no useful soft affinity, skip it.
         *
         * In fact, what we want is to check if we have any "soft-affine
         * work" to steal, before starting to look at "hard-affine work".
         *
         * Notice that, if not even one vCPU on this runq has a useful
         * soft affinity, we could have avoid considering this runq for
         * a soft balancing step in the first place. This, for instance,
         * can be implemented by taking note of on what runq there are
         * vCPUs with useful soft affinities in some sort of bitmap
         * or counter.
         */
        if ( vc->is_running || (balance_step == BALANCE_SOFT_AFFINITY &&
                                !has_soft_affinity(vc)) )
            continue;

        affinity_balance_cpumask(vc, balance_step, cpumask_scratch);
        if ( __csched_vcpu_is_migrateable(prv, vc, cpu, cpumask_scratch) )
        {
            /* We got a candidate. Grab it! */
            TRACE_3D(TRC_CSCHED_STOLEN_VCPU, peer_cpu,
                     vc->domain->domain_id, vc->vcpu_id);
            SCHED_VCPU_STAT_CRANK(speer, migrate_q);
            SCHED_STAT_CRANK(migrate_queued);
            WARN_ON(vc->is_urgent);
            runq_remove(speer);
            dequeue_task_fair(NULL, speer, 0);
            vc->processor = cpu;
            /*
             * speer will start executing directly on cpu, without having to
             * go through runq_insert(). So we must update the runnable count
             * for cpu here.
             */
            inc_nr_runnable(cpu);
            return speer;
        }
    }
 out:
    SCHED_STAT_CRANK(steal_peer_idle);
    return NULL;
}

static struct csched_vcpu *
csched_load_balance(struct csched_private *prv, int cpu,
    struct csched_vcpu *snext, bool_t *stolen)
{
    struct cpupool *c = per_cpu(cpupool, cpu);
    struct csched_vcpu *speer;
    cpumask_t workers;
    cpumask_t *online;
    int peer_cpu, first_cpu, peer_node, bstep;
    int node = cpu_to_node(cpu);

    BUG_ON( cpu != snext->vcpu->processor );
    online = cpupool_online_cpumask(c);

    /*
     * If this CPU is going offline, or is not (yet) part of any cpupool
     * (as it happens, e.g., during cpu bringup), we shouldn't steal work.
     */
    if ( unlikely(!cpumask_test_cpu(cpu, online) || c == NULL) )
        goto out;

    if ( snext->pri == CSCHED_PRI_IDLE )
        SCHED_STAT_CRANK(load_balance_idle);
    else if ( snext->pri == CSCHED_PRI_TS_OVER )
        SCHED_STAT_CRANK(load_balance_over);
    else
        SCHED_STAT_CRANK(load_balance_other);

    /*
     * Let's look around for work to steal, taking both hard affinity
     * and soft affinity into account. More specifically, we check all
     * the non-idle CPUs' runq, looking for:
     *  1. any "soft-affine work" to steal first,
     *  2. if not finding anything, any "hard-affine work" to steal.
     */
    for_each_affinity_balance_step( bstep )
    {
        /*
         * We peek at the non-idling CPUs in a node-wise fashion. In fact,
         * it is more likely that we find some affine work on our same
         * node, not to mention that migrating vcpus within the same node
         * could well expected to be cheaper than across-nodes (memory
         * stays local, there might be some node-wide cache[s], etc.).
         */
        peer_node = node;
        do
        {
            /* Select the pCPUs in this node that have work we can steal. */
            cpumask_andnot(&workers, online, prv->idlers);
            cpumask_and(&workers, &workers, &node_to_cpumask(peer_node));
            __cpumask_clear_cpu(cpu, &workers);

            first_cpu = cpumask_cycle(prv->balance_bias[peer_node], &workers);
            if ( first_cpu >= nr_cpu_ids )
                goto next_node;
            peer_cpu = first_cpu;
            do
            {
                spinlock_t *lock;

                /*
                 * If there is only one runnable vCPU on peer_cpu, it means
                 * there's no one to be stolen in its runqueue, so skip it.
                 *
                 * Checking this without holding the lock is racy... But that's
                 * the whole point of this optimization!
                 *
                 * In more details:
                 * - if we race with dec_nr_runnable(), we may try to take the
                 *   lock and call csched_runq_steal() for no reason. This is
                 *   not a functional issue, and should be infrequent enough.
                 *   And we can avoid that by re-checking nr_runnable after
                 *   having grabbed the lock, if we want;
                 * - if we race with inc_nr_runnable(), we skip a pCPU that may
                 *   have runnable vCPUs in its runqueue, but that's not a
                 *   problem because:
                 *   + if racing with csched_vcpu_insert() or csched_vcpu_wake(),
                 *     __runq_tickle() will be called afterwords, so the vCPU
                 *     won't get stuck in the runqueue for too long;
                 *   + if racing with csched_runq_steal(), it may be that a
                 *     vCPU that we could have picked up, stays in a runqueue
                 *     until someone else tries to steal it again. But this is
                 *     no worse than what can happen already (without this
                 *     optimization), it the pCPU would schedule right after we
                 *     have taken the lock, and hence block on it.
                 */
                if ( CSCHED_PCPU(peer_cpu)->nr_runnable <= 1 )
                {
                    TRACE_2D(TRC_CSCHED_STEAL_CHECK, peer_cpu, /* skipp'n */ 0);
                    goto next_cpu;
                }

                /*
                 * Get ahold of the scheduler lock for this peer CPU.
                 *
                 * Note: We don't spin on this lock but simply try it. Spinning
                 * could cause a deadlock if the peer CPU is also load
                 * balancing and trying to lock this CPU.
                 */
                lock = pcpu_schedule_trylock(peer_cpu);
                SCHED_STAT_CRANK(steal_trylock);
                if ( !lock )
                {
                    SCHED_STAT_CRANK(steal_trylock_failed);
                    TRACE_2D(TRC_CSCHED_STEAL_CHECK, peer_cpu, /* skip */ 0);
                    goto next_cpu;
                }

                TRACE_2D(TRC_CSCHED_STEAL_CHECK, peer_cpu, /* checked */ 1);

                /* Any work over there to steal? */
                speer = cpumask_test_cpu(peer_cpu, online) ?
                    csched_runq_steal(peer_cpu, cpu, snext->pri, bstep) : NULL;
                pcpu_schedule_unlock(lock, peer_cpu);

                /* As soon as one vcpu is found, balancing ends */
                if ( speer != NULL )
                {
                    *stolen = 1;
                    /*
                     * Next time we'll look for work to steal on this node, we
                     * will start from the next pCPU, with respect to this one,
                     * so we don't risk stealing always from the same ones.
                     */
                    prv->balance_bias[peer_node] = peer_cpu;
                    return speer;
                }

 next_cpu:
                peer_cpu = cpumask_cycle(peer_cpu, &workers);

            } while( peer_cpu != first_cpu );

 next_node:
            peer_node = cycle_node(peer_node, node_online_map);
        } while( peer_node != node );
    }

 out:
    /* Failed to find more important work elsewhere... */
    __runq_remove(snext);
    // if(CSCHED_PCPU(snext->vcpu->processor)->curr != snext)
    //     __dequeue_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
    // if(CSCHED_PCPU(snext->vcpu->processor)->curr == snext)
    // {
    //     //dequeue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr, 0);
    //     set_next_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
    // }
    return snext;
}

/*
 * This function is in the critical path. It is designed to be simple and
 * fast for the common case.
 */
static struct task_slice
csched_schedule(
    const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct list_head * const runq = RUNQ(cpu);
    struct csched_vcpu * const scurr = CSCHED_VCPU(current);
    struct csched_private *prv = CSCHED_PRIV(ops);
    struct csched_vcpu *snext;
    struct task_slice ret;
    s_time_t runtime, tslice;

    SCHED_STAT_CRANK(schedule);
    CSCHED_VCPU_CHECK(current);

    /*
     * Here in Credit1 code, we usually just call TRACE_nD() helpers, and
     * don't care about packing. But scheduling happens very often, so it
     * actually is important that the record is as small as possible.
     */
    if ( unlikely(tb_init_done) )
    {
        struct {
            unsigned cpu:16, tasklet:8, idle:8;
        } d;
        d.cpu = cpu;
        d.tasklet = tasklet_work_scheduled;
        d.idle = is_idle_vcpu(current);
        __trace_var(TRC_CSCHED_SCHEDULE, 1, sizeof(d),
                    (unsigned char *)&d);
    }

    runtime = now - current->runstate.state_entry_time;
    if ( runtime < 0 ) /* Does this ever happen? */
        runtime = 0;

    if ( !is_idle_vcpu(scurr->vcpu) )
    {
        /* Update credits of a non-idle VCPU. */
        burn_credits(scurr, now);
        scurr->start_time -= now;
    }
    else
    {
        /* Re-instate a boosted idle VCPU as normal-idle. */
        scurr->pri = CSCHED_PRI_IDLE;
    }

    /* Choices, choices:
     * - If we have a tasklet, we need to run the idle vcpu no matter what.
     * - If sched rate limiting is in effect, and the current vcpu has
     *   run for less than that amount of time, continue the current one,
     *   but with a shorter timeslice and return it immediately
     * - Otherwise, chose the one with the highest priority (which may
     *   be the one currently running)
     * - If the currently running one is TS_OVER, see if there
     *   is a higher priority one waiting on the runqueue of another
     *   cpu and steal it.
     */

    /*
     * If we have schedule rate limiting enabled, check to see
     * how long we've run for.
     *
     * If scurr is yielding, however, we don't let rate limiting kick in.
     * In fact, it may be the case that scurr is about to spin, and there's
     * no point forcing it to do so until rate limiting expires.
     */
    if ( !test_bit(CSCHED_FLAG_VCPU_YIELD, &scurr->flags)
         && !tasklet_work_scheduled
         && prv->ratelimit
         && vcpu_runnable(current)
         && !is_idle_vcpu(current)
         && runtime < prv->ratelimit )
    {
        snext = scurr;
        snext->start_time += now;
        //yield_task_fair();
        perfc_incr(delay_ms);
        /*
         * Next timeslice must last just until we'll have executed for
         * ratelimit. However, to avoid setting a really short timer, which
         * will most likely be inaccurate and counterproductive, we never go
         * below CSCHED_MIN_TIMER.
         */
        tslice = prv->ratelimit - runtime;
        if ( unlikely(runtime < CSCHED_MIN_TIMER) )
            tslice = CSCHED_MIN_TIMER;
        if ( unlikely(tb_init_done) )
        {
            struct {
                unsigned vcpu:16, dom:16;
                unsigned runtime;
            } d;
            d.dom = scurr->vcpu->domain->domain_id;
            d.vcpu = scurr->vcpu->vcpu_id;
            d.runtime = runtime;
            __trace_var(TRC_CSCHED_RATELIMIT, 1, sizeof(d),
                        (unsigned char *)&d);
        }

        ret.migrated = 0;
        goto out;
    }
    tslice = prv->tslice;

    /*
     * Select next runnable local VCPU (ie top of local runq)
     */
    if ( vcpu_runnable(current) )
    {
        __runq_insert(scurr);
        put_prev_task_fair(NULL, scurr);
        // if(CSCHED_PCPU(scurr->vcpu->processor)->curr != scurr)
        //     __enqueue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr);
    }
    else
    {
        BUG_ON( is_idle_vcpu(current) || list_empty(runq) );
        /* Current has blocked. Update the runnable counter for this cpu. */
        dec_nr_runnable(cpu);
    }

    //snext = pick_next_task_fair(CSCHED_PCPU(cpu), scurr);
    snext = pick_next_entity(CSCHED_PCPU(cpu), NULL);
    // if(CSCHED_PCPU(snext->vcpu->processor)->curr == snext)
    // {
    //     //dequeue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr, 0);
    //     set_next_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
    // }
    snext = __runq_elem(runq->next);
    ret.migrated = 0;

    // if(CSCHED_PCPU(scurr->vcpu->processor)->curr == scurr)
    // {
    //     //dequeue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr, 0);
    //     set_next_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr);
    // }

    /* Tasklet work (which runs in idle VCPU context) overrides all else. */
    if ( tasklet_work_scheduled )
    {
        TRACE_0D(TRC_CSCHED_SCHED_TASKLET);
        snext = CSCHED_VCPU(idle_vcpu[cpu]);
        snext->pri = CSCHED_PRI_TS_BOOST;
    }

    /*
     * Clear YIELD flag before scheduling out
     */
    clear_bit(CSCHED_FLAG_VCPU_YIELD, &scurr->flags);

    /*
     * SMP Load balance:
     *
     * If the next highest priority local runnable VCPU has already eaten
     * through its credits, look on other PCPUs to see if we have more
     * urgent work... If not, csched_load_balance() will return snext, but
     * already removed from the runq.
     */
    if ( snext->pri > CSCHED_PRI_TS_OVER )
    {
        __runq_remove(snext);
        // if(CSCHED_PCPU(snext->vcpu->processor)->curr != snext)
        //     __dequeue_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
        // if(CSCHED_PCPU(snext->vcpu->processor)->curr == snext)
        // {
        //     //dequeue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr, 0);
        //     set_next_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
        // }
    }
    else
        snext = csched_load_balance(prv, cpu, snext, &ret.migrated);

    //if(CSCHED_PCPU(snext->vcpu->processor)->curr == snext)
    //{
        //dequeue_entity(CSCHED_PCPU(scurr->vcpu->processor), scurr, 0);
        set_next_entity(CSCHED_PCPU(snext->vcpu->processor), snext);
    //}

    /*
     * Update idlers mask if necessary. When we're idling, other CPUs
     * will tickle us when they get extra work.
     */
    if ( !tasklet_work_scheduled && snext->pri == CSCHED_PRI_IDLE )
    {
        if ( !cpumask_test_cpu(cpu, prv->idlers) )
            cpumask_set_cpu(cpu, prv->idlers);
    }
    else if ( cpumask_test_cpu(cpu, prv->idlers) )
    {
        cpumask_clear_cpu(cpu, prv->idlers);
    }

    if ( !is_idle_vcpu(snext->vcpu) )
        snext->start_time += now;

out:
    /*
     * Return task to run next...
     */
    ret.time = (is_idle_vcpu(snext->vcpu) ?
                -1 : tslice);
    ret.task = snext->vcpu;

    CSCHED_VCPU_CHECK(ret.task);
    return ret;
}

static void
csched_dump_vcpu(struct csched_vcpu *svc)
{
    struct csched_dom * const sdom = svc->sdom;

    printk("[%i.%i] pri=%i flags=%x cpu=%i",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->pri,
            svc->flags,
            svc->vcpu->processor);

    if ( sdom )
    {
        printk(" credit=%i [w=%u,cap=%u]", atomic_read(&svc->credit),
                sdom->weight, sdom->cap);
#ifdef CSCHED_STATS
        printk(" (%d+%u) {a/i=%u/%u m=%u+%u (k=%u)}",
                svc->stats.credit_last,
                svc->stats.credit_incr,
                svc->stats.state_active,
                svc->stats.state_idle,
                svc->stats.migrate_q,
                svc->stats.migrate_r,
                svc->stats.kicked_away);
#endif
    }

    printk("\n");
}

static void
csched_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct list_head *runq, *iter;
    struct csched_private *prv = CSCHED_PRIV(ops);
    struct csched_pcpu *spc;
    struct csched_vcpu *svc;
    spinlock_t *lock;
    unsigned long flags;
    int loop;
#define cpustr keyhandler_scratch

    /*
     * We need both locks:
     * - csched_dump_vcpu() wants to access domains' scheduling
     *   parameters, which are protected by the private scheduler lock;
     * - we scan through the runqueue, so we need the proper runqueue
     *   lock (the one of the runqueue of this cpu).
     */
    spin_lock_irqsave(&prv->lock, flags);
    lock = pcpu_schedule_lock(cpu);

    spc = CSCHED_PCPU(cpu);
    runq = &spc->runq;

    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_sibling_mask, cpu));
    printk("CPU[%02d] nr_run=%d, sort=%d, sibling=%s, ",
           cpu, spc->nr_runnable, spc->runq_sort_last, cpustr);
    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_core_mask, cpu));
    printk("core=%s\n", cpustr);

    /* current VCPU (nothing to say if that's the idle vcpu). */
    svc = CSCHED_VCPU(curr_on_cpu(cpu));
    if ( svc && !is_idle_vcpu(svc->vcpu) )
    {
        printk("\trun: ");
        csched_dump_vcpu(svc);
    }

    loop = 0;
    list_for_each( iter, runq )
    {
        svc = __runq_elem(iter);
        if ( svc )
        {
            printk("\t%3d: ", ++loop);
            csched_dump_vcpu(svc);
        }
    }

    pcpu_schedule_unlock(lock, cpu);
    spin_unlock_irqrestore(&prv->lock, flags);
#undef cpustr
}

static void
csched_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc;
    struct csched_private *prv = CSCHED_PRIV(ops);
    int loop;
    unsigned long flags;

    spin_lock_irqsave(&prv->lock, flags);

#define idlers_buf keyhandler_scratch

    printk("info:\n"
           "\tncpus              = %u\n"
           "\tmaster             = %u\n"
           "\tcredit             = %u\n"
           "\tcredit balance     = %d\n"
           "\tweight             = %u\n"
           "\trunq_sort          = %u\n"
           "\tdefault-weight     = %d\n"
           "\ttslice             = %"PRI_stime"ms\n"
           "\tratelimit          = %"PRI_stime"us\n"
           "\tcredits per msec   = %d\n"
           "\tticks per tslice   = %d\n"
           "\tmigration delay    = %"PRI_stime"us\n",
           prv->ncpus,
           prv->master,
           prv->credit,
           prv->credit_balance,
           prv->weight,
           prv->runq_sort,
           CSCHED_DEFAULT_WEIGHT,
           prv->tslice / MILLISECS(1),
           prv->ratelimit / MICROSECS(1),
           CSCHED_CREDITS_PER_MSEC,
           prv->ticks_per_tslice,
           prv->vcpu_migr_delay/ MICROSECS(1));

    cpumask_scnprintf(idlers_buf, sizeof(idlers_buf), prv->idlers);
    printk("idlers: %s\n", idlers_buf);

    printk("active vcpus:\n");
    loop = 0;
    list_for_each( iter_sdom, &prv->active_sdom )
    {
        struct csched_dom *sdom;
        sdom = list_entry(iter_sdom, struct csched_dom, active_sdom_elem);

        list_for_each( iter_svc, &sdom->active_vcpu )
        {
            struct csched_vcpu *svc;
            spinlock_t *lock;

            svc = list_entry(iter_svc, struct csched_vcpu, active_vcpu_elem);
            lock = vcpu_schedule_lock(svc->vcpu);

            printk("\t%3d: ", ++loop);
            csched_dump_vcpu(svc);

            vcpu_schedule_unlock(lock, svc->vcpu);
        }
    }
#undef idlers_buf

    spin_unlock_irqrestore(&prv->lock, flags);
}

static int
csched_init(struct scheduler *ops)
{
    struct csched_private *prv;

    prv = xzalloc(struct csched_private);
    if ( prv == NULL )
        return -ENOMEM;

    prv->balance_bias = xzalloc_array(uint32_t, MAX_NUMNODES);
    if ( prv->balance_bias == NULL )
    {
        xfree(prv);
        return -ENOMEM;
    }

    if ( !zalloc_cpumask_var(&prv->cpus) ||
         !zalloc_cpumask_var(&prv->idlers) )
    {
        free_cpumask_var(prv->cpus);
        xfree(prv->balance_bias);
        xfree(prv);
        return -ENOMEM;
    }

    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->active_sdom);
    prv->master = UINT_MAX;

    if ( sched_credit_tslice_ms > XEN_SYSCTL_CSCHED_TSLICE_MAX
         || sched_credit_tslice_ms < XEN_SYSCTL_CSCHED_TSLICE_MIN )
    {
        printk("WARNING: sched_credit_tslice_ms outside of valid range [%d,%d].\n"
               " Resetting to default %u\n",
               XEN_SYSCTL_CSCHED_TSLICE_MIN,
               XEN_SYSCTL_CSCHED_TSLICE_MAX,
               CSCHED_DEFAULT_TSLICE_MS);
        sched_credit_tslice_ms = CSCHED_DEFAULT_TSLICE_MS;
    }

    __csched_set_tslice(prv, sched_credit_tslice_ms);

    if ( MICROSECS(sched_ratelimit_us) > MILLISECS(sched_credit_tslice_ms) )
    {
        printk("WARNING: sched_ratelimit_us >" 
               "sched_credit_tslice_ms is undefined\n"
               "Setting ratelimit to tslice\n");
        prv->ratelimit = prv->tslice;
    }
    else
        prv->ratelimit = MICROSECS(sched_ratelimit_us);

    if ( vcpu_migration_delay_us > XEN_SYSCTL_CSCHED_MGR_DLY_MAX_US )
    {
        vcpu_migration_delay_us = 0;
        printk("WARNING: vcpu_migration_delay outside of valid range [0,%d]us.\n"
               "Resetting to default: %u\n",
               XEN_SYSCTL_CSCHED_MGR_DLY_MAX_US, vcpu_migration_delay_us);
    }
    prv->vcpu_migr_delay = MICROSECS(vcpu_migration_delay_us);

    return 0;
}

static void test(void){
    //update_curr_fair(NULL);
    dequeue_task_fair(NULL, NULL, 0);
    wakeup_preempt_entity(NULL, NULL);
    set_last_buddy(NULL);
    //yield_to_task_fair(NULL, NULL, 0);
    //detach_task_cfs_rq(NULL);
    //attach_task_cfs_rq(NULL);
    //set_curr_task_fair(NULL);
    check_preempt_wakeup(NULL, NULL, 0);
    yield_task_fair(NULL);
    task_tick_fair(NULL, NULL, 0);
    enqueue_task_fair(NULL, NULL, 0);
    pick_next_task_fair(NULL, NULL);
    check_preempt_tick(NULL, NULL);
}

static void
csched_deinit(struct scheduler *ops)
{
    struct csched_private *prv;

    prv = CSCHED_PRIV(ops);
    if ( prv != NULL )
    {
        ops->sched_data = NULL;
        free_cpumask_var(prv->cpus);
        free_cpumask_var(prv->idlers);
        xfree(prv->balance_bias);
        xfree(prv);
    }

    return;

    test();
}

static void csched_tick_suspend(const struct scheduler *ops, unsigned int cpu)
{
    struct csched_pcpu *spc;

    spc = CSCHED_PCPU(cpu);

    stop_timer(&spc->ticker);
}

static void csched_tick_resume(const struct scheduler *ops, unsigned int cpu)
{
    struct csched_private *prv;
    struct csched_pcpu *spc;
    uint64_t now = NOW();

    spc = CSCHED_PCPU(cpu);

    prv = CSCHED_PRIV(ops);

    set_timer(&spc->ticker, now + MICROSECS(prv->tick_period_us)
            - now % MICROSECS(prv->tick_period_us) );
}

static const struct scheduler sched_credit_def = {
    .name           = "SMP Credit Scheduler",
    .opt_name       = "credit",
    .sched_id       = XEN_SCHEDULER_CREDIT,
    .sched_data     = NULL,

    .insert_vcpu    = csched_vcpu_insert,
    .remove_vcpu    = csched_vcpu_remove,

    .sleep          = csched_vcpu_sleep,
    .wake           = csched_vcpu_wake,
    .yield          = csched_vcpu_yield,

    .adjust         = csched_dom_cntl,
    .adjust_affinity= csched_aff_cntl,
    .adjust_global  = csched_sys_cntl,

    .pick_cpu       = csched_cpu_pick,
    .do_schedule    = csched_schedule,

    .dump_cpu_state = csched_dump_pcpu,
    .dump_settings  = csched_dump,
    .init           = csched_init,
    .deinit         = csched_deinit,
    .alloc_vdata    = csched_alloc_vdata,
    .free_vdata     = csched_free_vdata,
    .alloc_pdata    = csched_alloc_pdata,
    .init_pdata     = csched_init_pdata,
    .deinit_pdata   = csched_deinit_pdata,
    .free_pdata     = csched_free_pdata,
    .switch_sched   = csched_switch_sched,
    .alloc_domdata  = csched_alloc_domdata,
    .free_domdata   = csched_free_domdata,

    .tick_suspend   = csched_tick_suspend,
    .tick_resume    = csched_tick_resume,
};

REGISTER_SCHEDULER(sched_credit_def);