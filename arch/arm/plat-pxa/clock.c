/*
 *  linux/arch/arm/plat-pxa/clock.c
 *
 *  based on arch/arm/mach-tegra/clock.c
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/regs-apbc.h>
#include <plat/clock.h>


static DEFINE_MUTEX(clock_list_lock);
static LIST_HEAD(clocks);


static inline bool clk_cansleep(struct clk *c)
{
	return c->cansleep;
}

#define clk_lock_save(c, flags)						\
	do {								\
		if (clk_cansleep(c)) {					\
			flags = 0;					\
			mutex_lock(&c->mutex);				\
		} else {						\
			spin_lock_irqsave(&c->spinlock, flags);		\
		}							\
	} while (0)

#define clk_unlock_restore(c, flags)					\
	do {								\
		if (clk_cansleep(c))					\
			mutex_unlock(&c->mutex);			\
		else							\
			spin_unlock_irqrestore(&c->spinlock, flags);	\
	} while (0)

#ifdef CONFIG_LOCKDEP

#define clock_set_lockdep_class(c, lock)				\
	do {								\
		lockdep_set_class(lock, &c->lockdep_key);		\
	} while (0)

#else

#define clock_set_lockdep_class(c, lock)	do {} while (0)

#endif

static inline void clk_lock_init(struct clk *c)
{
	mutex_init(&c->mutex);
	clock_set_lockdep_class(c, &c->mutex);
	spin_lock_init(&c->spinlock);
	clock_set_lockdep_class(c, &c->spinlock);
}

static void __clk_set_cansleep(struct clk *c)
{
	struct clk *child;
	BUG_ON(mutex_is_locked(&c->mutex));
	BUG_ON(spin_is_locked(&c->spinlock));

	list_for_each_entry(child, &clocks, node) {
		if (child->parent != c)
			continue;

		WARN(child->ops && child->ops->set_parent,
			"can't make child clock %s of %s "
			"sleepable if it's parent could change",
			child->name, c->name);

		__clk_set_cansleep(child);
	}

	c->cansleep = true;
}

/* Must be called before any clk_get calls */
void clk_set_cansleep(struct clk *c)
{

	mutex_lock(&clock_list_lock);
	__clk_set_cansleep(c);
	mutex_unlock(&clock_list_lock);
}

int clk_reparent(struct clk *c, struct clk *parent)
{
	c->parent = parent;
	return 0;
}

void clk_init(struct clk *c)
{
	clk_lock_init(c);

	if (c->ops && c->ops->init)
		c->ops->init(c);

	if (!c->ops || !c->ops->enable)
		c->refcnt++;

	mutex_lock(&clock_list_lock);
	list_add(&c->node, &clocks);
	mutex_unlock(&clock_list_lock);
}

/* Must be called with clk_lock(c) held */
static unsigned long clk_predict_rate_from_parent(struct clk *c, struct clk *p)
{
	u64 rate;

	rate = clk_get_rate(p);

	if (c->mul != 0 && c->div != 0) {
		rate *= c->mul;
		do_div(rate, c->div);
	}

	return rate;
}

/* Must be called with clk_lock(c) held */
unsigned long clk_get_rate_locked(struct clk *c)
{
	unsigned long rate;

	if (c->ops && c->ops->getrate)
		rate = c->ops->getrate(c);
	else if (c->parent)
		rate = clk_predict_rate_from_parent(c, c->parent);
	else
		rate = c->rate;

	return rate;
}

int clk_enable(struct clk *c)
{
	int ret = 0;
	unsigned long flags;
	int i = 0;
	u32 dependence_count = c->dependence_count;

	clk_lock_save(c, flags);

	if (c->refcnt == 0) {
		while (dependence_count != 0) {
			dependence_count--;
			ret = clk_enable(c->dependence[i]);
			if (ret) {
				while (i > 0)
					clk_disable(c->dependence[--i]);
				goto out;
			}
			i++;
		}

		if (c->parent) {
			ret = clk_enable(c->parent);
			if (ret)
				goto disable_depend;
		}

		if (c->ops && c->ops->enable) {
			ret = c->ops->enable(c);
			if (ret) {
				if (c->parent)
					clk_disable(c->parent);
				goto disable_depend;
			}
		}
	}
	c->refcnt++;

	clk_unlock_restore(c, flags);
	return 0;

disable_depend:
	while (dependence_count != 0)
		clk_disable(c->dependence[--dependence_count]);
out:
	clk_unlock_restore(c, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *c)
{
	unsigned long flags;
	u32 dependence_count = c->dependence_count;

	clk_lock_save(c, flags);

	if (c->refcnt == 0) {
		clk_unlock_restore(c, flags);
		return;
	}
	if (c->refcnt == 1) {
		if (c->ops && c->ops->disable)
			c->ops->disable(c);

		if (c->parent)
			clk_disable(c->parent);

		while (dependence_count != 0)
			clk_disable(c->dependence[--dependence_count]);
	}
	c->refcnt--;

	clk_unlock_restore(c, flags);
}
EXPORT_SYMBOL(clk_disable);

int clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret = 0;
	unsigned long flags;
	long new_rate;

	clk_lock_save(c, flags);

	if ((c->refcnt != 0) && !c->dynamic_change) {
		ret = -EBUSY;
		goto out;
	}

	if (!c->ops || !c->ops->setrate) {
		ret = -ENOSYS;
		goto out;
	}

	if (c->ops && c->ops->round_rate) {
		new_rate = c->ops->round_rate(c, rate);

		if (new_rate < 0) {
			ret = new_rate;
			goto out;
		}

		rate = new_rate;
	}

	ret = c->ops->setrate(c, rate);
	if (ret)
		goto out;
	else
		c->rate = rate;
out:
	clk_unlock_restore(c, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

unsigned long clk_get_rate(struct clk *c)
{
	unsigned long flags;
	unsigned long rate;

	clk_lock_save(c, flags);

	rate = clk_get_rate_locked(c);

	clk_unlock_restore(c, flags);

	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_parent(struct clk *c, struct clk *parent)
{
	int ret = 0;
	unsigned long flags;

	clk_lock_save(c, flags);

	if (!c->ops || !c->ops->set_parent) {
		ret = -ENOSYS;
		goto out;
	}

	ret = c->ops->set_parent(c, parent);
	if (ret)
		goto out;

out:
	clk_unlock_restore(c, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *c)
{
	return c->parent;
}
EXPORT_SYMBOL(clk_get_parent);

long clk_round_rate(struct clk *c, unsigned long rate)
{
	unsigned long flags;
	long ret;

	clk_lock_save(c, flags);

	if (!c->ops || !c->ops->round_rate) {
		ret = -ENOSYS;
		goto out;
	}

	ret = c->ops->round_rate(c, rate);

out:
	clk_unlock_restore(c, flags);
	return ret;
}
EXPORT_SYMBOL(clk_round_rate);

