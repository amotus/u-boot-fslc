#include <common.h>
#include <watchdog.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/dma.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>

#ifdef CONFIG_MXS_WATCHDOG
#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 60000
#endif

/**
 * stmp3xxx_wdt_set_timeout - configure the watchdog inside the STMP3xxx RTC
 * @timeout: the desired value for the timeout register of the watchdog.
 */
unsigned long stmp3xxx_wdt_set_timeout(unsigned long timeout)
{
	struct mxs_rtc_regs *rtc_regs =
		(struct mxs_rtc_regs *)MXS_RTC_BASE;

    if (timeout){
        /* AUTO_RESET(b17) - automatically power up approximately 180 ms after powering down. */
        writel(RTC_PERSISTENT0_AUTO_RESTART, &rtc_regs->hw_rtc_persistent0_set);
        /* WATCHDOGEN + ONEMSEC_IRQ */
        writel(RTC_CTRL_WATCHDOGEN|RTC_CTRL_ONEMSEC_IRQ, &rtc_regs->hw_rtc_ctrl_set);
        /* watchdog timer register - takes msec */
        writel(timeout, &rtc_regs->hw_rtc_watchdog);
    }
}

void hw_watchdog_reset(void)
{
    stmp3xxx_wdt_set_timeout(CONFIG_WATCHDOG_TIMEOUT_MSECS);
}

void hw_watchdog_init(void)
{
    stmp3xxx_wdt_set_timeout(CONFIG_WATCHDOG_TIMEOUT_MSECS);
}
#endif