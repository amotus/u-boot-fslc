/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Board configuration file for IDE Systems DistroPMC boards.
 */

#ifndef __IDE_DISTROPMC
#define __IDE_DISTROPMC

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>


/* SPL options */
#include "imx6_spl.h"

#define CONFIG_SC_TIMER_CLK 8000000 /* 8Mhz */
#define COUNTER_FREQUENCY CONFIG_SC_TIMER_CLK
#define CONFIG_BOARD_POSTCLK_INIT
#define CONFIG_MXC_GPT_HCLK
/* Manufacturing needs a rather big initramfs, make sure we have
 * enough memory available. */
#define CONFIG_SYS_BOOTM_LEN            SZ_128M
#define CONFIG_SYS_FSL_CLK
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_LOADADDR		0x82000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
/* Miscellaneous configurable options */
#define CONFIG_SYS_CBSIZE	512
#define CONFIG_SYS_MAXARGS	32
#ifndef CONFIG_MX6
#define CONFIG_MX6
#endif

#define CONFIG_SYS_FSL_USDHC_NUM        2

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_DRIVERS_MISC_SUPPORT
#endif

#ifdef CONFIG_CMD_NET
#define CONFIG_FEC_ENET_DEV		0
#define CONFIG_ETHPRIME			"eth0"
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

/* Environment settings */

/* Environment in SD */
#define MMC_ROOTFS_DEV			1
#define MMC_ROOTFS_PART			2

/* Console configs */
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */

#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
#define CONFIG_SUPPORT_EMMC_BOOT

/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif

/* Miscellaneous configurable options */

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_512M

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* USB Configs */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2

#define ENV_MMC \
	"mmcdev=" __stringify(MMC_ROOTFS_DEV) "\0" \
	"mmcpart=" __stringify(MMC_ROOTFS_PART) "\0" \
	"fitpart=1\0" \
	"bootdelay=3\0" \
	"silent=1\0" \
	"optargs=rw rootwait\0" \
	"mmcautodetect=yes\0" \
	"mmcrootfstype=ext4\0" \
	"mmcfit_name=fitImage\0" \
	"mmcloadfit=fatload mmc ${mmcdev}:${fitpart} ${fit_addr} " \
		    "${mmcfit_name}\0" \
	"mmcargs=setenv bootargs " \
		"root=/dev/mmcblk${mmcdev}p${mmcpart} ${optargs} " \
		"rootfstype=${mmcrootfstype}\0" \
	"mmc_mmc_fit=run mmcloadfit;run mmcargs addcon; bootm ${fit_addr}#conf-${fdt_file}\0" \

#define ENV_MFG \
	"mfg_fit=bootm ${fit_addr}#conf-${fdt_file}\0" \

/* Default environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"console=ttymxc0,115200n8\0" \
	"addcon=setenv bootargs ${bootargs} console=${console}\0" \
	"fit_addr=0x82000000\0" \
	ENV_MMC \
	ENV_MFG

#endif	/* __IDE_DISTROPMC */
