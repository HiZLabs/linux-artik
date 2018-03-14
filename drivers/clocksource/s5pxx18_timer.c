/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Youngbok, Park <ybpark@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/list.h>

#define CLK_SOURCE_HZ (10 * 1000000) /* or 1MHZ */
#define CLK_EVENT_HZ (10 * 1000000)  /* or 1MHZ */

/* timer register */
#define REG_TCFG0 (0x00)
#define REG_TCFG1 (0x04)
#define REG_TCON (0x08)
#define REG_TCNTB0 (0x0C)
#define REG_TCMPB0 (0x10)
#define REG_TCNT0 (0x14)
#define REG_CSTAT (0x44)

#define TCON_BIT_AUTO (1 << 3)
#define TCON_BIT_INVT (1 << 2)
#define TCON_BIT_UP (1 << 1)
#define TCON_BIT_RUN (1 << 0)
#define TCFG0_BIT_CH(ch) (ch == 0 || ch == 1 ? 0 : 8)
#define TCFG1_BIT_CH(ch) (ch * 4)
#define TCON_BIT_CH(ch) (ch ? ch * 4 + 4 : 0)
#define TINT_CSTAT_BIT_CH(ch) (ch + 5)
#define TINT_CSTAT_MASK (0x1F)
#define TIMER_TCNT_OFFS (0xC)

static void timer_source_suspend(struct clocksource *cs);
static void timer_source_resume(struct clocksource *cs);
static int timer_event_shutdown(struct clock_event_device *evt);
static int timer_event_set_periodic(struct clock_event_device *evt);
static int timer_event_set_oneshot(struct clock_event_device *evt);
static int timer_event_shutdown(struct clock_event_device *evt);
static int timer_event_set_next(unsigned long delta,	struct clock_event_device *evt);
static void timer_event_resume(struct clock_event_device *evt);
static cycle_t timer_source_read(struct clocksource *cs);
static irqreturn_t timer_event_handler(int irq, void *dev_id);

/* timer data structs */
struct timer_info {
	int channel;
	int interrupt;
	const char *clock_name;
	struct clk *clk;
	unsigned long request;
	unsigned long rate;
	int tmux;
	int prescale;
	unsigned int tcount;
	unsigned int rcount;
};

struct timer_of_dev {
	void __iomem *base;
	struct clk *pclk;
	char source_name[20];
	char event_name[20];
	char irq_name [25];
	struct timer_info timer_source;
	struct timer_info timer_event;
	struct clocksource timer_clocksource;
	struct clock_event_device timer_clock_event;
	struct irqaction timer_event_irqaction;
	struct list_head list;
};

static LIST_HEAD(devs);
static int num_instances = 0;

static struct timer_of_dev* get_dev_for_clocksource(const struct clocksource *cs) {
	struct timer_of_dev *dev;

	list_for_each_entry(dev, &devs, list) {
		if(&dev->timer_clocksource == cs)
			return dev;
	}

	return NULL;
}

static struct timer_of_dev *get_dev_for_clock_event_device(const struct clock_event_device *ce) {
	struct timer_of_dev *dev;

	list_for_each_entry(dev, &devs, list) {
		if(&dev->timer_clock_event == ce)
			return dev;
	}

	return NULL;
}



static struct timer_of_dev *create_timer_dev(void) {
	struct timer_of_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL | __GFP_NOFAIL);

	sprintf(dev->source_name, "source timer %d", num_instances);
	sprintf(dev->event_name, "event timer %d", num_instances);
	sprintf(dev->irq_name, "Event Timer %d IRQ", num_instances);
	num_instances++;

	dev->timer_clocksource.name = dev->source_name;
	dev->timer_clocksource.rating = 300;
	dev->timer_clocksource.read = timer_source_read;
	dev->timer_clocksource.mask = CLOCKSOURCE_MASK(32);
	dev->timer_clocksource.shift = 20;
	dev->timer_clocksource.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	dev->timer_clocksource.suspend = timer_source_suspend;
	dev->timer_clocksource.resume = timer_source_resume;
	dev->timer_clock_event.name = dev->event_name;
	dev->timer_clock_event.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	dev->timer_clock_event.set_state_shutdown = timer_event_shutdown;
	dev->timer_clock_event.set_state_periodic = timer_event_set_periodic;
	dev->timer_clock_event.set_state_oneshot = timer_event_set_oneshot;
	dev->timer_clock_event.tick_resume = timer_event_shutdown;
	dev->timer_clock_event.set_next_event = timer_event_set_next;
	dev->timer_clock_event.resume = timer_event_resume;
	dev->timer_clock_event.rating = 150; /* Lower than dummy timer (for 6818) */
	dev->timer_event_irqaction.name = dev->irq_name;
	dev->timer_event_irqaction.flags = IRQF_TIMER; /* removed IRQF_DISABLED kernel 4.1.15 */
	dev->timer_event_irqaction.handler = timer_event_handler;
	dev->timer_event_irqaction.dev_id = dev;
	INIT_LIST_HEAD(&dev->list);

	list_add(&dev->list, &devs);

	return dev;
}

static struct timer_of_dev *get_first_timer_dev(void) {
	return list_first_entry_or_null(&devs, struct timer_of_dev, list);
}


static inline void timer_periph_reset(int id) { return; }

static inline void timer_clock(void __iomem *base, int ch, int mux, int scl)
{
	u32 val = readl(base + REG_TCFG0) & ~(0xFF << TCFG0_BIT_CH(ch));

	writel(val | ((scl - 1) << TCFG0_BIT_CH(ch)), base + REG_TCFG0);
	val = readl(base + REG_TCFG1) & ~(0xF << TCFG1_BIT_CH(ch));
	writel(val | (mux << TCFG1_BIT_CH(ch)), base + REG_TCFG1);
}

static inline void timer_count(void __iomem *base, int ch, unsigned int cnt)
{
	writel((cnt - 1), base + REG_TCNTB0 + (TIMER_TCNT_OFFS * ch));
	writel((cnt - 1), base + REG_TCMPB0 + (TIMER_TCNT_OFFS * ch));
}

static inline void timer_start(void __iomem *base, int ch, int irqon)
{
	int on = irqon ? 1 : 0;
	u32 val = readl(base + REG_CSTAT) & ~(TINT_CSTAT_MASK << 5 | 0x1 << ch);

	writel(val | (0x1 << TINT_CSTAT_BIT_CH(ch) | on << ch),
	       base + REG_CSTAT);
	val = readl(base + REG_TCON) & ~(0xE << TCON_BIT_CH(ch));
	writel(val | (TCON_BIT_UP << TCON_BIT_CH(ch)), base + REG_TCON);

	val &= ~(TCON_BIT_UP << TCON_BIT_CH(ch));
	val |= ((TCON_BIT_AUTO | TCON_BIT_RUN) << TCON_BIT_CH(ch));
	writel(val, base + REG_TCON);
}

static inline void timer_stop(void __iomem *base, int ch, int irqon)
{
	int on = irqon ? 1 : 0;
	u32 val = readl(base + REG_CSTAT) & ~(TINT_CSTAT_MASK << 5 | 0x1 << ch);

	writel(val | (0x1 << TINT_CSTAT_BIT_CH(ch) | on << ch),
	       base + REG_CSTAT);
	val = readl(base + REG_TCON) & ~(TCON_BIT_RUN << TCON_BIT_CH(ch));
	writel(val, base + REG_TCON);
}

static inline unsigned int timer_read(void __iomem *base, int ch)
{
	return readl(base + REG_TCNT0 + (TIMER_TCNT_OFFS * ch));
}

static inline u32 timer_read_count_dev(struct timer_of_dev *dev)
{
	struct timer_info *info;

	if (NULL == dev || NULL == dev->base)
		return 0;

	info = &dev->timer_source;

	info->rcount = (info->tcount - timer_read(dev->base, info->channel));
	return (cycle_t)info->rcount;
}

static inline u32 timer_read_count(void) {
	return timer_read_count_dev(get_first_timer_dev());
}

/*
 * Timer clock source
 */
static void timer_clock_select(struct timer_of_dev *dev, struct timer_info *info)
{
	unsigned long rate, tout = 0;
	unsigned long mout, thz, delt = (-1UL);
	unsigned long frequency = info->request;
	int tscl = 0, tmux = 5, smux = 0, pscl = 0;
	int from_tclk = 0;

	if (dev->pclk) {
		rate = clk_get_rate(dev->pclk);
		for (smux = 0; 5 > smux; smux++) {
			mout = rate / (1 << smux), pscl = mout / frequency;
			thz = mout / (pscl ? pscl : 1);
			if (!(mout % frequency) && 256 > pscl) {
				tout = thz, tmux = smux, tscl = pscl;
				break;
			}
			if (pscl > 256)
				continue;
			if (abs(frequency - thz) >= delt)
				continue;
			tout = thz, tmux = smux, tscl = pscl;
			delt = abs(frequency - thz);
		}
	}

	if (tout != frequency) {
		rate = clk_round_rate(info->clk, frequency);
		if (abs(frequency - tout) >= abs(frequency - rate)) {
			clk_set_rate(info->clk, rate);
			clk_prepare_enable(info->clk);
			tout = rate, tmux = 5, tscl = 1, from_tclk = 1;
		}
	}

	if (dev->pclk && !from_tclk) {
		clk_put(info->clk);
		info->clk = NULL;
		rate = clk_get_rate(dev->pclk); /* restore pclk */
	}

	info->tmux = tmux;
	info->prescale = tscl;
	info->tcount = tout / HZ;
	info->rate = tout;

	pr_debug("%s (ch:%d, mux=%d, scl=%d, rate=%ld, %s)\n", __func__,
		 info->channel, tmux, tscl, tout, from_tclk ? "TCLK" : "PCLK");
}

static void timer_source_suspend(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_dev_for_clocksource(cs);
	struct timer_info *info = &dev->timer_source;
	void __iomem *base = dev->base;
	int ch = info->channel;

	if (info->clk) {
		clk_disable_unprepare(info->clk);
	}

	info->rcount = (info->tcount - timer_read(base, ch));
	timer_stop(base, ch, 0);
}

static void timer_source_resume(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_dev_for_clocksource(cs);
	struct timer_info *info = &dev->timer_source;
	void __iomem *base = dev->base;
	int ch = info->channel;
	ulong flags;

	pr_debug("%s (ch:%d, mux:%d, scale:%d cnt:0x%x,0x%x)\n", __func__, ch,
		 info->tmux, info->prescale, info->rcount, info->tcount);

	local_irq_save(flags);

	if (info->clk) {
		clk_set_rate(info->clk, info->rate);
		clk_prepare_enable(info->clk);
	}

	timer_stop(base, ch, 0);
	timer_clock(base, ch, info->tmux, info->prescale);
	timer_count(base, ch, info->rcount + 1); /* restore count */
	timer_start(base, ch, 0);
	timer_count(base, ch, info->tcount + 1); /* next count */

	local_irq_restore(flags);
}

static cycle_t timer_source_read(struct clocksource *cs)
{
	return (cycle_t)timer_read_count_dev(get_first_timer_dev());
}

static int __init timer_source_of_init(struct device_node *node, struct timer_of_dev *dev)
{
	struct timer_info *info = &dev->timer_source;
	struct clocksource *cs = &dev->timer_clocksource;
	void __iomem *base = dev->base;
	int ch = info->channel;

	info->request = CLK_SOURCE_HZ;

	timer_clock_select(dev, info);

	/* reset tcount */
	info->tcount = 0xFFFFFFFF;

	if(dev != list_first_entry(&devs, struct timer_of_dev, list))
		cs->rating = 50;

	clocksource_register_hz(cs, info->rate);

	timer_stop(base, ch, 0);
	timer_clock(base, ch, info->tmux, info->prescale);
	timer_count(base, ch, 0);
	timer_start(base, ch, 0);

	pr_debug("timer.%d: source, %9lu(HZ:%d), mult:%u\n", ch, info->rate, HZ,
	       cs->mult);
	return 0;
}

/*
 * Timer clock event
 */
static void timer_event_resume(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_dev_for_clock_event_device(evt);
	struct timer_info *info = &dev->timer_event;
	void __iomem *base = dev->base;
	int ch = info->channel;

	pr_debug("%s (ch:%d, mux:%d, scale:%d)\n", __func__, ch, info->tmux,
		 info->prescale);

	timer_stop(base, ch, 1);
	timer_clock(base, ch, info->tmux, info->prescale);
}

static int timer_event_shutdown(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_dev_for_clock_event_device(evt);
	struct timer_info *info = &dev->timer_event;
	void __iomem *base = dev->base;
	int ch = info->channel;

	timer_stop(base, ch, 0);

	return 0;
}

static int timer_event_set_oneshot(struct clock_event_device *evt)
{
	return 0;
}

static int timer_event_set_periodic(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_dev_for_clock_event_device(evt);
	struct timer_info *info = &dev->timer_event;
	void __iomem *base = dev->base;
	int ch = info->channel;
	unsigned long cnt = info->tcount;

	timer_stop(base, ch, 0);
	timer_count(base, ch, cnt);
	timer_start(base, ch, 1);

	return 0;
}

static int timer_event_set_next(unsigned long delta, struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_dev_for_clock_event_device(evt);
	struct timer_info *info = &dev->timer_event;
	void __iomem *base = dev->base;
	int ch = info->channel;
	ulong flags;

	raw_local_irq_save(flags);

	timer_stop(base, ch, 0);
	timer_count(base, ch, delta);
	timer_start(base, ch, 1);

	raw_local_irq_restore(flags);
	return 0;
}

static irqreturn_t timer_event_handler(int irq, void *dev_id)
{
	struct timer_of_dev *dev = (struct timer_of_dev *)dev_id;
	struct clock_event_device *evt = &dev->timer_clock_event;
	struct timer_info *info = &dev->timer_event;
	void __iomem *base = dev->base;
	int ch = info->channel;
	u32 val;

	/* clear status */
	val = readl(base + REG_CSTAT) & ~(TINT_CSTAT_MASK << 5);
	val |= (0x1 << TINT_CSTAT_BIT_CH(ch));
	writel(val, base + REG_CSTAT);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

#ifdef CONFIG_ARM64
/*
 * to __delay , refer to arch_timer.h and  arm64 lib delay.c
 */
u64 arch_counter_get_cntvct(void) { return timer_read_count_dev(get_first_timer_dev()); }
EXPORT_SYMBOL(arch_counter_get_cntvct);

int arch_timer_arch_init(void) { return 0; }
#endif

static int __init timer_event_of_init(struct device_node *node, struct timer_of_dev *dev)
{
	struct timer_info *info = &dev->timer_event;
	struct clock_event_device *evt = &dev->timer_clock_event;
	void __iomem *base = dev->base;
	int ch = info->channel;

	info->request = CLK_EVENT_HZ;

	timer_clock_select(dev, info);
	timer_stop(base, ch, 1);
	timer_clock(base, ch, info->tmux, info->prescale);

	setup_irq(info->interrupt, &dev->timer_event_irqaction);
	clockevents_calc_mult_shift(evt, info->rate, 5);
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(0x7, evt);
	evt->cpumask = cpumask_of(0);
	evt->irq = info->interrupt;

	if(dev != list_first_entry(&devs, struct timer_of_dev, list))
		evt->rating = 50;

	clockevents_register_device(evt);

	pr_debug("timer.%d: event , %9lu(HZ:%d), mult:%u\n",
		 ch, info->rate, HZ, evt->mult);
	return 0;
}

static int __init
timer_get_device_data(struct device_node *node, struct timer_of_dev *dev)
{
	struct timer_info *tsrc = &dev->timer_source;
	struct timer_info *tevt = &dev->timer_event;

	dev->base = of_iomap(node, 0);
	if (!dev->base) {
		pr_err("Can't map registers for timer!");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "clksource", &tsrc->channel)) {
		pr_err("timer node is missing 'clksource'\n");
		return -EINVAL;
	}

	if (of_property_read_u32(node, "clkevent", &tevt->channel)) {
		pr_err("timer node is missing 'clkevent'\n");
		return -EINVAL;
	}
	tevt->interrupt = irq_of_parse_and_map(node, 0);

	tsrc->clk = of_clk_get(node, 0);
	if (IS_ERR(tsrc->clk)) {
		pr_err("failed timer tsrc clock\n");
		return -EINVAL;
	}

	tevt->clk = of_clk_get(node, 1);
	if (IS_ERR(tevt->clk)) {
		pr_err("failed timer event clock\n");
		return -EINVAL;
	}

	dev->pclk = of_clk_get(node, 2);
	if (IS_ERR(dev->pclk))
		dev->pclk = NULL;

	pr_debug("%s : ch %d,%d irq %d\n", node->name, tsrc->channel,
		 tevt->channel, tevt->interrupt);

	return 0;
}

#ifdef CONFIG_ARM
static struct delay_timer nxp_delay_timer = {
	.freq = CLK_SOURCE_HZ,
	.read_current_timer = (unsigned long (*)(void))timer_read_count,
};
#endif

static void __init timer_of_init_dt(struct device_node *node)
{
	struct timer_of_dev *dev;
	dev = create_timer_dev();

	if (timer_get_device_data(node, dev))
		panic("unable to map timer cpu !!!\n");

	timer_source_of_init(node, dev);
	timer_event_of_init(node, dev);

#ifdef CONFIG_ARM
	register_current_timer_delay(&nxp_delay_timer);
#endif
}

CLOCKSOURCE_OF_DECLARE(s5p6818, "nexell,s5p6818-timer", timer_of_init_dt);
