// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Marc Ferland, Amotus Solutions Inc., <ferlandm@amotus.ca>
 */

#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <dm.h>
#include <fsl_esdhc_imx.h>
#include <i2c_eeprom.h>
#include <linux/bitops.h>
#include <malloc.h>
#include <miiphy.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (fec_id != 0)
		return -1;	/* not supported */

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/*
	 * Defaults + Enable status LEDs (LED1: Activity, LED0: Link) & select
	 * 50 MHz RMII clock mode.
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif /* CONFIG_FEC_MXC */

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

	return 0;
}

/* length of strings stored in the eeprom */
#define DART6UL_PN_LEN   16
#define DART6UL_ASSY_LEN 16
#define DART6UL_DATE_LEN 12

/* eeprom content, 512 bytes */
struct dart6ul_info {
	u32 magic;
	u8 partnumber[DART6UL_PN_LEN];
	u8 assy[DART6UL_ASSY_LEN];
	u8 date[DART6UL_DATE_LEN];
	u32 custom_addr_val[32];
	struct cmd {
		u8 addr;
		u8 index;
	} custom_cmd[150];
	u8 res[33];
	u8 som_info;
	u8 ddr_size;
	u8 crc;
} __attribute__ ((__packed__));

#define DART6UL_INFO_STORAGE_GET(n) ((n) & 0x3)
#define DART6UL_INFO_WIFI_GET(n)    ((n) >> 2 & 0x1)
#define DART6UL_INFO_REV_GET(n)     ((n) >> 3 & 0x3)
#define DART6UL_DDRSIZE(n)          ((n) * SZ_128M)
#define DART6UL_INFO_MAGIC          0x32524156

static const char *som_info_storage_to_str(u8 som_info)
{
	switch (DART6UL_INFO_STORAGE_GET(som_info)) {
	case 0x0: return "none (SD only)";
	case 0x1: return "NAND";
	case 0x2: return "eMMC";
	default: return "unknown";
	}
}

static const char *som_info_rev_to_str(u8 som_info)
{
	switch (DART6UL_INFO_REV_GET(som_info)) {
	case 0x0: return "2.4G";
	case 0x1: return "5G";
	default: return "unknown";
	}
}

static int get_eeprom_device(const char *path, struct udevice **dev)
{
	int ret, off;

	off = fdt_path_offset(gd->fdt_blob, path);
	if (off < 0) {
		printf("%s: fdt_path_offset() failed: %d\n", __func__, off);
		return off;
	}

	ret = uclass_get_device_by_of_offset(UCLASS_I2C_EEPROM, off, dev);
	if (ret) {
		printf("%s: uclass_get_device_by_of_offset() failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int checkboard(void)
{
	const char *path = "eeprom0";
	struct dart6ul_info *info;
	struct udevice *dev;
	int ret;

	ret = get_eeprom_device(path, &dev);
	if (ret)
		return ret;

	info = malloc(sizeof(struct dart6ul_info));
	if (!info)
		return -ENOMEM;

	ret = i2c_eeprom_read(dev, 0, (uint8_t *)info,
			      sizeof(struct dart6ul_info));
	if (ret) {
		printf("%s: i2c_eeprom_read() failed: %d\n", __func__, ret);
		free(info);
		return ret;
	}

	if (info->magic != DART6UL_INFO_MAGIC) {
		printf("Board: Invalid board info magic: 0x%08x, expected 0x%08x\n",
		       info->magic, DART6UL_INFO_MAGIC);
		/* do not fail if the content is invalid */
		free(info);
		return 0;
	}

	/* make sure strings are null terminated */
	info->partnumber[DART6UL_PN_LEN - 1] = '\0';
	info->assy[DART6UL_ASSY_LEN - 1] = '\0';
	info->date[DART6UL_DATE_LEN - 1] = '\0';

	printf("Board: PN: %s, Assy: %s, Date: %s\n"
	       "       Storage: %s, Wifi: %s, DDR: %d MiB, Rev: %s\n",
	       info->partnumber,
	       info->assy,
	       info->date,
	       som_info_storage_to_str(info->som_info),
	       DART6UL_INFO_WIFI_GET(info->som_info) ? "yes" : "no",
	       DART6UL_DDRSIZE(info->ddr_size) / SZ_1M,
	       som_info_rev_to_str(info->som_info));

	free(info);

	return 0;
}

#define CARRIER_EEPROM_ADDR 0x52
#define CARRIER_EEPROM_BUS  0x1
#define CARRIER_MAGIC 0xcb1de445
#define HDR_NAME_LEN   64
#define HDR_SERIAL_LEN 64
#define HDR_CONFIG_LEN 64

/* board config, 200 bytes (4 + 4 + 64 + 64 + 64) */
struct board_eecfg {
	unsigned int magic;
	unsigned int rev;
	char name[HDR_NAME_LEN];
	char serial[HDR_SERIAL_LEN];
	char config[HDR_CONFIG_LEN];
}  __attribute__ ((__packed__));

enum board_rev {
	IDE_REVA,
	IDE_REVB,
	IDE_UNKNOWN,
};

static int get_board_eecfg(struct board_eecfg *h)
{
	const char *path = "eeprom1";
	struct udevice *dev;
	int ret;

	ret = get_eeprom_device(path, &dev);
	if (ret)
		return ret;

	ret = i2c_eeprom_read(dev, 0, (uint8_t *)h,
			      sizeof(struct board_eecfg));
	if (ret) {
		printf("%s: i2c_eeprom_read() failed: %d\n", __func__, ret);
		return ret;
	}

	if (h->magic != CARRIER_MAGIC)
		return -EINVAL;

	return 0;
}

static enum board_rev get_board_rev(void)
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
		puts("Carrier: wrong magic, assuming revB\n");
		return IDE_REVB;
	default:
		puts("Carrier: io error reading eeprom, assuming revA\n");
		break;
	}
	return IDE_REVA;
}

int board_late_init(void)
{
	const char *name = get_board_rev() == IDE_REVA ? \
		"imx6ull-dart6ul-ide-pmc-reva.dtb" : "imx6ull-dart6ul-ide-pmc-revb.dtb";
	return env_set("fdt_file", name);
}
