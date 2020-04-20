// SPDX-License-Identifier: GPL-2.0+
/*
 * DIGI Connected Board ccardimx28js Support
 * Copyright (C) 2020 Amotus Solutions
 * Author: Walter Bonetti <bonettiw@amotus.ca>
 *
 * Based on m28evk.c:
 * (C) Copyright 2011 Freescale Semiconductor, Inc.
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Based on m28evk.c:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Functions
 */
int board_early_init_f(void)
{
    /* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

#ifdef	CONFIG_CMD_MMC
	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
#endif

#ifdef	CONFIG_CMD_USB
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS1__USB1_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_RX__GPIO_3_8 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_RX__GPIO_3_8, 1);
#endif
	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC
static int mx28evk_mmc_wp(int id)
{
	if (id != 0) {
		printf("MXS MMC: Invalid card selected (card id = %d)\n", id);
		return 1;
	}

	return gpio_get_value(MX28_PAD_SSP1_SCK__GPIO_2_12);
}

int board_mmc_init(bd_t *bis)
{
    return mxsmmc_initialize(bis, 0, mx28evk_mmc_wp, NULL); 
}
#endif

#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC1_INIT_ONLY_MAC
#define CONFIG_ENET0_PHY_RESET_GPIO	MX28_PAD_PWM4__GPIO_3_29

#define KSZ80x1_ANEG_ADVERT		0x04
#define NEXT_PAGE			(1 << 15)

#define KSZ80x1_OPMODE_STRAPOV		0x16
#define BCAST_OFF			(1 << 9)

#define MII_PHY_CTRL2			0x1f
#define HP_AUTO_MDI			(1 << 15)
#define ENABLE_JABBER_COUNTER		(1 << 8)
#define RMII_50MHZ_CLOCK		(1 << 7)
#define CONFIG_DIGI_EVK_PHY

#ifdef CONFIG_DIGI_EVK_PHY 
int fecmxc_mii_postcall(int phy)
{
	unsigned short val;
	char phyname[4];

	sprintf(phyname, "FEC%d", phy ? 1 : 0);
	miiphy_write(phyname, phy, MII_PHY_CTRL2,
		     HP_AUTO_MDI | ENABLE_JABBER_COUNTER |
		     RMII_50MHZ_CLOCK);

	/*
	 * Set bit 9 of register 0x16 (undocumented) to work
	 * around Micrel PHY bug that causes the second PHY, with
	 * address=3, to also respond to reads/writes addressed
	 * to the first PHY, which has address=0.
	 * The setting of this bit for platforms having only
	 * one PHY at address 0 is harmless.
	 */
	if (!miiphy_read(phyname, phy, KSZ80x1_OPMODE_STRAPOV, &val))
		miiphy_write(phyname, phy, KSZ80x1_OPMODE_STRAPOV,
			     val | BCAST_OFF);

	/* Clear Next Page capable bit (set by default on KSZ8081RNA) */
	if (!miiphy_read(phyname, phy, KSZ80x1_ANEG_ADVERT, &val))
		miiphy_write(phyname, phy, KSZ80x1_ANEG_ADVERT,
			     val & ~NEXT_PAGE);

	return 0;
}
#endif /* CONFIG_DIGI_EVK_PHY */

int board_eth_init(bd_t *bis)
{
#ifdef CONFIG_DIGI_EVK_PHY 
struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_PWM4__GPIO_3_29, 0);
	udelay(10300);  // from datasheet: tvr + tsr
	gpio_set_value(MX28_PAD_PWM4__GPIO_3_29, 1);
    /* Ensure recommended delay of 100us in PHY datasheet is met before
	 * accessing the MDIO interface */

    udelay(100);
	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	ret = fecmxc_initialize_multi(bis, 1, 3, MXS_ENET1_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC1\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

    ret = fecmxc_register_mii_postcall(dev, fecmxc_mii_postcall);
	if (ret) {
		printf("FEC MXS: Unable to register FEC0 mii postcall\n");
		return ret;
	}

#ifndef CONFIG_FEC1_INIT_ONLY_MAC
	dev = eth_get_dev_by_name("FEC1");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC1 device entry\n");
		return -EINVAL;
	}

    ret = fecmxc_register_mii_postcall(dev, fecmxc_mii_postcall);
	if (ret) {
		printf("FEC MXS: Unable to register FEC0 mii postcall\n");
		return ret;
	}
#endif
	return ret;
#else
    return 0;
#endif /* CONFIG_DIGI_EVK_PHY */
}

#endif

int board_late_init(void)
{
	int fet_active = 1;	/* default polarity */

	/*
	 * Set the FET off, then on to make sure the peripherals are
	 * properly reset
	 */
	gpio_direction_output(MX28_PAD_LCD_RS__GPIO_1_26, !fet_active);
	mxs_iomux_setup_pad(MX28_PAD_LCD_RS__GPIO_1_26 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_PULLUP);
	udelay(50);
	gpio_direction_output(MX28_PAD_LCD_RS__GPIO_1_26, fet_active);

    /*
     * Customer Watchdog Timer
     * WDT Early Feed Once
     * GPIO 2_25
    */
	gpio_direction_output(MX28_PAD_SSP3_MOSI__GPIO_2_25, 1);
	udelay(1000);
	gpio_direction_output(MX28_PAD_SSP3_MOSI__GPIO_2_25, 0);
	udelay(1000);
	gpio_set_value(MX28_PAD_SSP3_MOSI__GPIO_2_25, 1);

	return 0;
}

/*
 * This function returns the size of available RAM for doing a TFTP transfer.
 * This size depends on:
 *   - The total RAM available
 *   - The loadaddr
 * U-Boot is assumed to be loaded at a very low address (below loadaddr) so
 * it is not a variable in this calculation
 */
unsigned int get_tftp_available_ram(unsigned int loadaddr)
{
	return loadaddr - gd->bd->bi_dram[0].size;
}