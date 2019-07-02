/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2015 Variscite Ltd. All Rights Reserved.
 * Maintainer: Ron Donio <ron.d@variscite.com>
 * Configuration settings for the Variscite  i.MX6UL DART board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ull_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>

#include "../common/eeprom.h"

int eeprom_revision __attribute__ ((section ("sram")));
static long sdram_size __attribute__ ((section ("sram")));

DECLARE_GLOBAL_DATA_PTR;

#define DDR0_CS0_END 0x021b0040

#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |			\
		       PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
		       PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |			\
			PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |	\
			PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |	\
		       PAD_CTL_SPEED_HIGH   |			\
		       PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
		      PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
		      PAD_CTL_DSE_40ohm | PAD_CTL_HYS |		\
		      PAD_CTL_ODE)

#define MDIO_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |		\
		       PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |			\
			  PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define LCD_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
		      PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)

#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)

#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#ifdef CONFIG_SYS_I2C_MXC

/* I2C1 38 54 55 */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		/* conflict with usb_otg2_pwr */
		.i2c_mode  = MX6_PAD_UART4_TX_DATA__I2C1_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 28),
	},
	.sda = {
		/* conflict with usb_otg2_oc */
		.i2c_mode  = MX6_PAD_UART4_RX_DATA__I2C1_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 29),
	},
};

/* I2C2 1A 50 51 */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode  = MX6_PAD_UART5_TX_DATA__I2C2_SCL   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 30),
	},
	.sda = {
		/* conflict with usb_otg2_oc */
		.i2c_mode  = MX6_PAD_UART5_RX_DATA__I2C2_SDA   | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART5_RX_DATA__GPIO1_IO31 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 31),
	},
};

#endif	/* CONFIG_SYS_I2C_MXC */

int dram_init(void)
{
	unsigned int volatile *const port1 = (unsigned int *)PHYS_SDRAM;
	unsigned int volatile *port2;
	unsigned int volatile *ddr_cs0_end = (unsigned int *)DDR0_CS0_END;

	/* Set the sdram_size to the actually configured one */
	sdram_size = ((*ddr_cs0_end) - 63) * 32;
	do {
		port2 = (unsigned int volatile *) (PHYS_SDRAM + ((sdram_size * 1024 * 1024) / 2));
		*port2 = 0;		// write zero to start of second half of memory.
		*port1 = 0x3f3f3f3f;	// write pattern to start of memory.

		if ((0x3f3f3f3f == *port2) && (sdram_size > 128))
			sdram_size = sdram_size / 2;	// Next step devide size by half
		else
			if (0 == *port2) break;		// Done actual size found.

	} while (sdram_size > 128);

	gd->ram_size = ((ulong)sdram_size * 1024 * 1024);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* uSD slot */
#ifndef CONFIG_SPL_BUILD
static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};
#endif

/* eMMC soldered */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__USDHC2_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__USDHC2_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__USDHC2_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__USDHC2_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{ .esdhc_base = USDHC1_BASE_ADDR, .sdhc_clk = 0, .max_bus_width = 4 },
	{ .esdhc_base = USDHC2_BASE_ADDR, .sdhc_clk = 0, .max_bus_width = 8 },
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret, i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)   (Attached Physical Device)
	 * MMC0                    USDHC1            uSD
	 * MMC1                    USDHC2            eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) than supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	/* in spl, always boot from MMC1 (eMMC on USDHC2) */
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	usdhc_cfg[0].max_bus_width = 8;
	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

#ifdef CONFIG_USB_EHCI_MX6

#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* At default the 3v3 enables the MIC2026 for VBUS power */
static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return USB_INIT_DEVICE;
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET + port * 4);

	/* Set power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

#endif	/* CONFIG_USB_EHCI_MX6 */

#ifdef CONFIG_FEC_MXC

/*
 * Pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2.
 */

static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO        | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN      | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1  | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER      | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN      | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

int board_eth_init(bd_t *bis)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(fec1_pads,
					 ARRAY_SIZE(fec1_pads));

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				      CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);

#if defined(CONFIG_CI_UDC) && defined(CONFIG_USB_ETHER)
	/* USB Ethernet Gadget */
	usb_eth_initialize(bis);
#endif
	return ret;
}

static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(CONFIG_FEC_ENET_DEV, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#endif	/* CONFIG_FEC_MXC */

#ifdef CONFIG_TARGET_VARISCITE_DART6UL_IDE

#define CARRIER_EEPROM_ADDR 0x52
#define CARRIER_EEPROM_BUS  0x1
#define CARRIER_MAGIC 0xcb1de445
#define HDR_NAME_LEN   64
#define HDR_SERIAL_LEN 64
#define HDR_CONFIG_LEN 64

struct board_eecfg {
	unsigned int magic;
	unsigned int rev;
	char name[HDR_NAME_LEN];
	char serial[HDR_SERIAL_LEN];
	char config[HDR_CONFIG_LEN];
};

enum board_rev {
	IDE_REVA,
	IDE_REVB,
	IDE_UNKNOWN,
};

#define HDR_SIZE 200 		/* 4 + 4 + 64 + 64 + 64 */

int get_board_eecfg(struct board_eecfg *h)
{
	uchar buf[HDR_SIZE];
	int prevbus = i2c_get_bus_num();

	i2c_set_bus_num(CARRIER_EEPROM_BUS);

	if (i2c_probe(CARRIER_EEPROM_ADDR))
		return -ENODEV;

	if (i2c_read(CARRIER_EEPROM_ADDR, 0, 1, buf, sizeof(buf)))
		return -EIO;

	i2c_set_bus_num(prevbus);

	/* deserialize */
	h->magic = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];
	h->rev = buf[7] << 24 | buf[6] << 16 | buf[5] << 8 | buf[4];
	memcpy(h->name, &buf[8], HDR_NAME_LEN);
	memcpy(h->serial, &buf[72], HDR_SERIAL_LEN);
	memcpy(h->config, &buf[136], HDR_CONFIG_LEN);

	if (h->magic != CARRIER_MAGIC)
		return -EINVAL;

	return 0;
}

enum board_rev get_board_rev(void)
{
	struct board_eecfg cfg;
	int err;

	err = get_board_eecfg(&cfg);
	if (!err) {
		switch (cfg.rev) {
		case 0x0: return IDE_REVA;
		case 0x1: return IDE_REVB;
		default:
			printf("Carrier: unknown revision 0x%x, assuming revA\n", cfg.rev);
			return IDE_REVA;
		}
	}
	switch (err) {
	case -ENODEV:
		puts("Carrier: no i2c device detected, assuming revA\n");
		break;
	case -EINVAL:
		puts("Carrier: wrong magic, assuming revA\n");
		break;
	default:
		puts("Carrier: io error reading eeprom, assuming revA\n");
		break;
	}
	return IDE_REVA;
}

#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif

	/* Vraiment besoin ETH? */
#ifdef	CONFIG_FEC_MXC
	setup_fec();
#endif

	/* Vraiment besoin USB dans Bootloader? */
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE

static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,	 0},
};

#endif	/* CONFIG_CMD_BMODE */

static struct eeprom_config eeprom_cfg;

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "MX6UL_VAR_DART");

	var_eeprom_v2_read_struct(&eeprom_cfg);

	switch ((eeprom_cfg.som_info >> 3) & 0x3) {
	case 0x0:
		env_set("som_rev", "1");
		break;
	case 0x1:
		env_set("som_rev", "2");
		break;
	default:
		env_set("som_rev", "unknown");
		break;
	}
#endif

#ifdef CONFIG_USB_GADGET_DOWNLOAD
	/* the usb download gadget is only used for board production */
	env_set("mfgboot", "yes");
#else
	env_set("mfgboot", "no");
#endif

#ifdef CONFIG_TARGET_VARISCITE_DART6UL_IDE
	if (get_board_rev() == IDE_REVA)
		env_set("fdt_file", "imx6ull-dart6ul-ide-revA.dtb");
	else
		env_set("fdt_file", "imx6ull-dart6ul-ide-revB.dtb");
#endif

	/* Set i2c mux to allow communication with rtc */
	i2c_set_bus_num(1);

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6UL Variscite DART\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD

#include <spl.h>
#include <linux/libfdt.h>
#include <asm/arch/mx6-ddr.h>

static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000c0000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000008,
	.dram_sdqs0 = 0x00000038,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0   = 0x414C0158,
	.p0_mprddlctl   = 0x40403A3A,
	.p0_mpwrdlctl   = 0x40405A56,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 1,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
        /* Enable Audio Clock for SOM codec */
	writel(0x01130100, (long *)CCM_CCOSR);
}

static void spl_dram_init(void)
{
	mx6ul_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&ddr_sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

/*
 * Second phase ddr init. Use eeprom values.
 */
static int spl_dram_init_v2(void)
{
	struct eeprom_config cfg;
	int ret;

	/* Add here: Read EEPROM and parse Variscite struct */
	memset(&cfg, 0x00, sizeof(cfg));

	ret = var_eeprom_v2_read_struct(&cfg);

	if (ret)
		return -1;

	/* Test for VAR2 in the header. */
	if (cfg.variscite_magic != 0x32524156)
		return -1;

	handle_eeprom_data(&cfg);

	sdram_size = cfg.ddr_size * 128;

	return 0;
}

void board_dram_init(void)
{
	int spl_status;

	/* Initialize DDR based on eeprom if exist */
	spl_status = spl_dram_init_v2();
	if (spl_status < 0) {
		spl_dram_init();
		eeprom_revision=0;
	} else
		eeprom_revision=2;
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	i2c_set_bus_num(1);

	/* DDR initialization */
	board_dram_init();

	/* Clear the BSS */
	memset(__bss_start, 0, __bss_end - __bss_start);

	if (eeprom_revision == 2) {
		var_eeprom_v2_read_struct(&eeprom_cfg);

		eeprom_cfg.part_number[sizeof(eeprom_cfg.part_number)-1] = (u8)0x00;
		eeprom_cfg.Assembly[sizeof(eeprom_cfg.Assembly)-1] = (u8)0x00;
		eeprom_cfg.date[sizeof(eeprom_cfg.date)-1] = (u8)0x00;

		printf("Part number: %s\n", (char *)eeprom_cfg.part_number);
		printf("Assembly: %s\n", (char *)eeprom_cfg.Assembly);
		printf("Date of production: %s\n", (char *)eeprom_cfg.date);
		printf("DART-6UL configuration: ");
		switch (eeprom_cfg.som_info & 0x3) {
		case 0x00:
			printf("SDCARD Only ");
			break;
		case 0x01:
			printf("NAND ");
			break;
		case 0x02:
			printf("eMMC ");
			break;
		case 0x03:
			printf("Ilegal !!! ");
			break;
		}

		if (eeprom_cfg.som_info & 0x04)
			printf("WIFI\n");
		else
			printf("\n");

		switch ((eeprom_cfg.som_info >> 3) & 0x3) {
		case 0x0:
			printf("SOM rev: 1\n");
			break;
		case 0x1:
			printf("SOM rev: 2\n");
			break;
		default:
			printf("SOM rev: unknown\n");
			break;
		}

	} else {
		printf("DDR LEGACY configuration\n");
	}

	dram_init();
	printf("Ram size: %ld\n", sdram_size);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}

#endif	/* CONFIG_SPL_BUILD */
