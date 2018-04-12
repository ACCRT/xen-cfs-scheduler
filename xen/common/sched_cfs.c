#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/timer.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <asm/div64.h>
#include <xen/errno.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>
#include <xen/err.h>
#include <xen/err.h>
#include <xen/guest_access.h>

/*
* Basic constants
*/
#define CFS_DEFAULT_LATENCY			0
#define CFS_TICKS_PER_TSLICE		3
#define CFS_DEFAULT_MIN_RUNTIME		30
/* Default timeslice: 30ms */
#define CFS_DEFAULT_TSLICE_MS		30
//#define CFS_CREDITS_PER_MSEC		10
/* Never set a timer shorter than this value. */
#define CFS_MIN_TIMER				XEN_SYSCTL_SCHED_RATELIMIT_MIN

/*
* Priorities
*/
#define CFS_PRI_TS_BOOST		0      /* time-share waking up */
#define CFS_PRI_IDLE			-64    /* idle */

/*
* Flags
*
* Note that svc->flags (where these flags live) is protected by an
* inconsistent set of locks. Therefore atomic-safe bit operations must
* be used for accessing it.
*/
//#define CFS_FLAG_VCPU_PARKED		0x0  /* VCPU over capped credits */
#define CFS_FLAG_VCPU_YIELD			0x1  /* VCPU yielding */
#define CFS_FLAG_VCPU_MIGRATING		0x2  /* VCPU may have moved to a new pcpu */
#define CFS_FLAG_VCPU_PINNED		0x4  /* VCPU can run only on 1 pcpu */

/*
 * Useful macros
 */
#define CFS_PRIV(_ops)   \
    ((struct cfs_private *)((_ops)->sched_data))
#define CFS_VCPU(_vcpu)  ((struct cfs_vcpu *) (_vcpu)->sched_priv)

struct cfs_pcpu {
	struct load_weight load;
	unsigned long nr_running;

	struct cfs_rb_root tasks_timeline;
	struct cfs_rb_node *cfs_rb_leftmost;
	
	struct sched_entity *curr, *next, *last;
};

struct cfs_vcpu {
	/* Up-pointers */
    struct cfs_dom *sdom;
    struct vcpu *vcpu;

	unsigned flags;

	struct load_weight	load;		/* for load-balancing */
	struct cfs_rb_node	run_node;
	//struct list_head	group_node;
	unsigned int		on_rq;

	u64			exec_start;
	u64			sum_exec_runtime;
	s_time_t	vruntime;
	u64			prev_sum_exec_runtime;

	u64			nr_migrations;
};

struct cfs_dom {
	struct domain *dom;
};

struct cfs_private {
	spinlock_t lock;        /* scheduler lock; nests inside cpupool_lock */

	u64 exec_clock;
	s_time_t min_vruntime;

	cpumask_var_t idlers;
    cpumask_var_t cpus;

	struct list_head active_sdom;	/* list for active domains */

	unsigned int master;
	struct timer master_ticker;
};

struct cfs_rb_root {
	struct cfs_rb_node *cfs_rb_node;
};

struct cfs_rb_node {
	unsigned long rb_parent_color;
#define	CFS_RB_RED		0
#define	CFS_RB_BLACK	1
	struct cfs_rb_node *cfs_rb_left;
	struct cfs_rb_node *cfs_rb_right;
};

struct load_weight {
	unsigned long weight, inv_weight;
};

/*struct sched_entity {
	struct load_weight	load;		// for load-balancing
	struct cfs_rb_node	run_node;
	//struct list_head	group_node;
	unsigned int		on_rq;

	u64			exec_start;
	u64			sum_exec_runtime;
	u64			vruntime;
	u64			prev_sum_exec_runtime;

	u64			nr_migrations;
};

struct sched_rt_entity {
	struct list_head run_list;
	unsigned long timeout;
	unsigned int time_slice;
	int nr_cpus_allowed;

	struct sched_rt_entity *back;
};*/

static int
cfs_init(struct scheduler *ops)
{
    struct cfs_private *prv;

    prv = xzalloc(struct cfs_private);
    if ( prv == NULL )
        return -ENOMEM;

    /*prv->balance_bias = xzalloc_array(uint32_t, MAX_NUMNODES);
    if ( prv->balance_bias == NULL )
    {
        xfree(prv);
        return -ENOMEM;
    }*/

    if ( !zalloc_cpumask_var(&prv->cpus) ||
         !zalloc_cpumask_var(&prv->idlers) )
    {
        free_cpumask_var(prv->cpus);
        //xfree(prv->balance_bias);
        xfree(prv);
        return -ENOMEM;
    }

    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->active_sdom);
    prv->master = UINT_MAX;

	// ???
	prv->min_vruntime = MILLISECS(CFS_DEFAULT_MIN_RUNTIME);

	// no time slice in cfs
	/*
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
    prv->vcpu_migr_delay = MICROSECS(vcpu_migration_delay_us);*/

    return 0;
}

static void
cfs_deinit(struct scheduler *ops)
{
    struct cfs_private *prv;

    prv = CFS_PRIV(ops);
    if ( prv != NULL )
    {
        ops->sched_data = NULL;
        free_cpumask_var(prv->cpus);
        free_cpumask_var(prv->idlers);
        //xfree(prv->balance_bias);
        xfree(prv);
    }
}

static void *
cfs_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct cfs_pcpu *spc;

    /* Allocate per-PCPU info */
    spc = xzalloc(struct cfs_pcpu);
    if ( spc == NULL )
        return ERR_PTR(-ENOMEM);

    return spc;
}

static void *
cfs_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct cfs_vcpu *svc;

    /* Allocate per-VCPU info */
    svc = xzalloc(struct cfs_vcpu);
    if ( svc == NULL )
        return NULL;

    // INIT_LIST_HEAD(&svc->runq_elem);
    // INIT_LIST_HEAD(&svc->active_vcpu_elem);
    svc->sdom = dd;
    svc->vcpu = vc;
    //svc->pri = is_idle_domain(vc->domain) ?
    //    CSCHED_PRI_IDLE : CSCHED_PRI_TS_UNDER;

	svc->vruntime = MILLISECS(0);
	(svc->run_node).cfs_rb_left = NULL;
	(svc->run_node).cfs_rb_right = NULL;

	// init load

    SCHED_VCPU_STATS_RESET(svc);
    SCHED_STAT_CRANK(vcpu_alloc);
    return svc;
}

static void
cfs_free_vdata(const struct scheduler *ops, void *priv)
{
    struct cfs_vcpu *svc = priv;

    BUG_ON( !((svc->run_node).cfs_rb_left == NULL && (svc->run_node).cfs_rb_right == NULL) );

    xfree(svc);
}

static void
cfs_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct cfs_vcpu *svc = vc->sched_priv;
    spinlock_t *lock;

    BUG_ON( is_idle_vcpu(vc) );

    /* csched_cpu_pick() looks in vc->processor's runq, so we need the lock. */
    lock = vcpu_schedule_lock_irq(vc);

    vc->processor = cfs_cpu_pick(ops, vc);

    spin_unlock_irq(lock);

    lock = vcpu_schedule_lock_irq(vc);

    if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )
        runq_insert(svc);

    vcpu_schedule_unlock_irq(lock, vc);

    SCHED_STAT_CRANK(vcpu_insert);
}

static int
cfs_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    struct cfs_vcpu *svc = CFS_VCPU(vc);

    /*
     * We have been called by vcpu_migrate() (in schedule.c), as part
     * of the process of seeing if vc can be migrated to another pcpu.
     * We make a note about this in svc->flags so that later, in
     * csched_vcpu_wake() (still called from vcpu_migrate()) we won't
     * get boosted, which we don't deserve as we are "only" migrating.
     */
    set_bit(CFS_FLAG_VCPU_MIGRATING, &svc->flags);
    return _cfs_cpu_pick(ops, vc, 1);
}

static const struct scheduler sched_cfs_def = {
	.name				= "SMP CFS Scheduler",
	.opt_name			= "cfs",
	.sched_id			= XEN_SCHEDULER_CFS,
	.sched_data			= NULL,

	.insert_vcpu		= cfs_vcpu_insert,
	.remove_vcpu		= cfs_vcpu_remove,

	.sleep				= cfs_vcpu_sleep,
	.wake				= cfs_vcpu_wake,
	.yield				= cfs_vcpu_yield,

	.adjust				= cfs_dom_cntl,
	.adjust_affinity	= cfs_aff_cntl,
	.adjust_global		= cfs_sys_cntl,

	.pick_cpu			= cfs_cpu_pick,
	.do_schedule		= cfs_schedule,

	.dump_cpu_state		= cfs_dump_pcpu,
	.dump_settings		= cfs_dump,
	.init				= cfs_init,
	.deinit				= cfs_deinit,
	.alloc_vdata		= cfs_alloc_vdata,
	.free_vdata			= cfs_free_vdata,
	.alloc_pdata		= cfs_alloc_pdata,
	.init_pdata			= cfs_init_pdata,
	.deinit_pdata		= cfs_deinit_pdata,
	.free_pdata			= cfs_free_pdata,
	.switch_sched		= cfs_switch_sched,
	.alloc_domdata		= cfs_alloc_domdata,
	.free_domdata		= cfs_free_domdata,

	.tick_suspend		= cfs_tick_suspend,
	.tick_resume		= cfs_tick_resume,
};

REGISTER_SCHEDULER(sched_cfs_def);