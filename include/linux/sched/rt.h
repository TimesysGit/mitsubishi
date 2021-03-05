#ifndef _SCHED_RT_H
#define _SCHED_RT_H

#include <linux/sched/prio.h>

static inline int rt_prio(int prio)
{
#if 1 /* Priority handling modification */
	if (unlikely(prio < (MAX_RT_PRIO+1)))
#else
	if (unlikely(prio < MAX_RT_PRIO))
#endif
		return 1;
	return 0;
}

static inline int rt_task(struct task_struct *p)
{
	return rt_prio(p->prio);
}

#ifdef CONFIG_RT_MUTEXES
/*
 * Must hold either p->pi_lock or task_rq(p)->lock.
 */
static inline struct task_struct *rt_mutex_get_top_task(struct task_struct *p)
{
	return p->pi_top_task;
}
extern void rt_mutex_setprio(struct task_struct *p, struct task_struct *pi_task);
extern void rt_mutex_adjust_pi(struct task_struct *p);
static inline bool tsk_is_pi_blocked(struct task_struct *tsk)
{
	return tsk->pi_blocked_on != NULL;
}
#else
static inline struct task_struct *rt_mutex_get_top_task(struct task_struct *task)
{
	return NULL;
}
# define rt_mutex_adjust_pi(p)		do { } while (0)
static inline bool tsk_is_pi_blocked(struct task_struct *tsk)
{
	return false;
}
#endif

extern void normalize_rt_tasks(void);


/*
 * default timeslice is 100 msecs (used only for SCHED_RR tasks).
 * Timeslices get refilled after they expire.
 */
#define RR_TIMESLICE		(100 * HZ / 1000)

#endif /* _SCHED_RT_H */
