
/* Copyright 2021 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/pwm.h>
#include <drivers/lnv_system.h>
#include <cmsis_core.h>
#include <soc.h>

LOG_MODULE_REGISTER(lnv_system, LOG_LEVEL_ERR);

/* Driver config */
struct lnv_system_npcx_config {
	/* hardware module base address */
	uintptr_t base_scfg;
	uintptr_t base_glue;
	uintptr_t base_twd;
	uintptr_t base_mswc;
};

/* Driver data */
struct lnv_system_npcx_data {
	int reset; /* reset cause */
};

/* Driver convenience defines */
#define DRV_CONFIG(dev) ((const struct lnv_system_npcx_config *)(dev)->config)

#define HAL_SCFG_INST(dev) (struct scfg_reg *)(DRV_CONFIG(dev)->base_scfg)
#define HAL_GLUE_INST(dev) (struct glue_reg *)(DRV_CONFIG(dev)->base_glue)
#define HAL_TWD_INST(dev) (struct twd_reg *)(DRV_CONFIG(dev)->base_twd)
#define HAL_MSWC_INST(dev) (struct mswc_reg *)(DRV_CONFIG(dev)->base_mswc)

#define DRV_DATA(dev) ((struct lnv_system_npcx_data *)(dev)->data)

#define SYSTEM_DT_NODE_SOC_ID_CONFIG DT_INST(0, nuvoton_npcx_soc_id)

/* Chip info devicetree data */
#define NPCX_FAMILY_ID DT_PROP(SYSTEM_DT_NODE_SOC_ID_CONFIG, family_id)

#define NPCX_CHIP_ID DT_PROP(SYSTEM_DT_NODE_SOC_ID_CONFIG, chip_id)

#define NPCX_DEVICE_ID DT_PROP(SYSTEM_DT_NODE_SOC_ID_CONFIG, device_id)

#define NPCX_REVISION_ADDR \
	DT_PROP_BY_IDX(SYSTEM_DT_NODE_SOC_ID_CONFIG, revision_reg, 0)
#define NPCX_REVISION_LEN \
	DT_PROP_BY_IDX(SYSTEM_DT_NODE_SOC_ID_CONFIG, revision_reg, 1)

/* RAM block size in npcx family (Unit: bytes) */
#define NPCX_RAM_BLOCK_SIZE (32 * 1024)
/* RAM block number in npcx7 series */

/* Calculate the number of RAM blocks:
 * total RAM size = code ram + data ram + extra 2K for ROM functions
 * divided by the block size 32k.
 */
#if DT_NODE_EXISTS(DT_NODELABEL(bootloader_ram))
#define BT_RAM_SIZE DT_REG_SIZE(DT_NODELABEL(bootloader_ram))
#else
#define BT_RAM_SIZE 0
#endif
#define DATA_RAM_SIZE DT_REG_SIZE(DT_NODELABEL(sram0))
#define CODE_RAM_SIZE DT_REG_SIZE(DT_NODELABEL(flash0))
#define NPCX_RAM_BLOCK_COUNT \
	((DATA_RAM_SIZE + CODE_RAM_SIZE + BT_RAM_SIZE) / NPCX_RAM_BLOCK_SIZE)

/* Valid bit-depth of RAM block Power-Down control (RAM_PD) registers. Use its
 * mask to power down all unnecessary RAM blocks before hibernating.
 */
#define NPCX_RAM_PD_DEPTH DT_PROP(DT_NODELABEL(pcc), ram_pd_depth)
#define NPCX_RAM_BLOCK_PD_MASK (BIT(NPCX_RAM_PD_DEPTH) - 1)

/* Get saved reset flag address in battery-backed ram */
#define BBRAM_SAVED_RESET_FLAG_ADDR                    \
	(DT_REG_ADDR(DT_INST(0, nuvoton_npcx_bbram)) + \
	 BBRAM_REGION_OFFSET(saved_reset_flags))

/* Soc specific system local functions */
#define PSL_NODE DT_INST(0, npcx_hibernate_psl)
#define PSL_OUT_GPIO_DRIVEN	0
#define PSL_OUT_FW_CTRL_DRIVEN	1
#define PINCTRL_STATE_HIBERNATE 1

#if DT_NODE_HAS_STATUS(PSL_NODE, okay)
PINCTRL_DT_DEFINE(PSL_NODE);
static int lnv_system_npcx_configure_psl_in(void)
{
	const struct pinctrl_dev_config *pcfg =
		PINCTRL_DT_DEV_CONFIG_GET(PSL_NODE);

	return pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
}

static void lnv_system_npcx_psl_out_inactive(void)
{
#if (DT_ENUM_IDX(PSL_NODE, psl_driven_type) == PSL_OUT_GPIO_DRIVEN)
	struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(PSL_NODE, enable_gpios);

	gpio_pin_set_dt(&enable, 1);
#elif (DT_ENUM_IDX(PSL_NODE, psl_driven_type) == PSL_OUT_FW_CTRL_DRIVEN)
	const struct pinctrl_dev_config *pcfg =
		PINCTRL_DT_DEV_CONFIG_GET(PSL_NODE);

	/* Set PSL_OUT to inactive */
	pinctrl_apply_state(pcfg, PINCTRL_STATE_HIBERNATE);
#endif
}
#else
static int lnv_system_npcx_configure_psl_in(void)
{
	return -EINVAL;
}

static void lnv_system_npcx_psl_out_inactive(void)
{
}
#endif

static void system_npcx_hibernate_by_psl(const struct device *dev,
					 uint32_t seconds,
					 uint32_t microseconds)
{
	ARG_UNUSED(dev);
	int ret;

	/*
	 * RTC wake-up in PSL mode only support in npcx9
	 * series. Nuvoton will introduce CLs for it later.
	 */
	ARG_UNUSED(seconds);
	ARG_UNUSED(microseconds);

	/* Configure detection settings of PSL_IN pads first */
	ret = lnv_system_npcx_configure_psl_in();
	if (ret < 0) {
		LOG_ERR("PSL_IN pinctrl setup failed (%d)", ret);
		return;
	}

	/*
	 * A transition from 0 to 1 of specific IO (GPIO85) data-out bit
	 * set PSL_OUT to inactive state. Then, it will turn Core Domain
	 * power supply (VCC1) off for better power consumption.
	 */
	lnv_system_npcx_psl_out_inactive();
}

/* lenovo system interface API functions */
static int lnv_system_npcx_get_reset_cause(const struct device *dev)
{
	struct lnv_system_npcx_data *data = DRV_DATA(dev);

	return data->reset;
}

static int lnv_system_npcx_soc_reset(const struct device *dev)
{
	const struct device *wdt_dev =
		DEVICE_DT_GET(DT_NODELABEL(twd0));
	struct wdt_timeout_cfg wdt_cfg0;
	int err;

	if (!device_is_ready(wdt_dev)) {
		LOG_ERR("Error: device %s is not ready", wdt_dev->name);
		return -ENODEV;
	}

	/* Stop the watchdog */
	wdt_disable(wdt_dev);

	/* Initialize watchdog for reset */
	wdt_cfg0.callback = NULL;
	wdt_cfg0.flags = WDT_FLAG_RESET_SOC;
	wdt_cfg0.window.max = 100; /* 100 ms */
	wdt_cfg0.window.min = 0;
	err = wdt_install_timeout(wdt_dev, &wdt_cfg0);
	if (err < 0) {
		LOG_ERR("Watchdog install error\n");
	}

	/* Start watchdog */
	err = wdt_setup(wdt_dev, 0);
	if (err < 0) {
		LOG_ERR("Watchdog setup error\n");
	}

	/* Disable interrupt */
	__asm__("cpsid i");

	/* Wait for the soc reset. */
	while (1) {
		;
	}

	/* should never return */
	return 0;
}

static int lnv_system_npcx_hibernate(const struct device *dev,
				      uint32_t seconds, uint32_t microseconds)
{
	const struct device *wdt_dev =
		DEVICE_DT_GET(DT_NODELABEL(twd0));
	if (!device_is_ready(wdt_dev)) {
		LOG_ERR("Error: device %s is not ready", wdt_dev->name);
		return -ENODEV;
	}

	/* Stop the watchdog */
	wdt_disable(wdt_dev);

	/* Disable interrupt first */
	__asm__("cpsid i");

	/* Enter hibernate mode */
	system_npcx_hibernate_by_psl(dev, seconds, microseconds);

	return 0;
}

static const char *lnv_system_npcx_get_chip_name(const struct device *dev)
{
	struct mswc_reg *const inst_mswc = HAL_MSWC_INST(dev);
	static char str[13] = "Unknown-XXXX";
	char *p = str + 8;
	uint8_t chip_id = inst_mswc->SRID_CR;
	uint8_t device_id = inst_mswc->DEVICE_ID_CR;

	if (chip_id == NPCX_CHIP_ID && device_id == NPCX_DEVICE_ID) {
		return CONFIG_SOC;
	}

	hex2char(chip_id >> 4, p++);
	hex2char(chip_id & 0xf, p++);
	hex2char(device_id >> 4, p++);
	hex2char(device_id & 0xf, p);
	return str;
}

/* lenovo system driver instances */
static int lnv_system_npcx_init(const struct device *dev)
{
	struct scfg_reg *const inst_scfg = HAL_SCFG_INST(dev);
	struct twd_reg *const inst_twd = HAL_TWD_INST(dev);
	struct lnv_system_npcx_data *data = DRV_DATA(dev);

	/* check reset cause */
	data->reset = 0;
	/* Use scratch bit to check power on reset or VCC1_RST reset. */
	if (!IS_BIT_SET(inst_scfg->RSTCTL, NPCX_RSTCTL_VCC1_RST_SCRATCH)) {
		bool is_vcc1_rst =
			IS_BIT_SET(inst_scfg->RSTCTL, NPCX_RSTCTL_VCC1_RST_STS);
		struct glue_reg *const inst_glue = HAL_GLUE_INST(dev);
		uint8_t psl_evt = inst_glue->PSL_CTS & NPCX_RESET_POWERUP_PSL_MASK;

		data->reset = is_vcc1_rst ? NPCX_RESET_VCC1_RST_PIN : NPCX_RESET_POWERUP;

		/* Check PSL_IN event */
		if (psl_evt != 0) {
			data->reset |= psl_evt;
			/* Clear all event status bits */
			inst_glue->PSL_CTS = psl_evt;
			inst_glue->PSL_IN_POS = psl_evt;
			inst_glue->PSL_IN_POS = psl_evt;
		}
	}

	/*
	 * Set scratch bit to distinguish VCC1_RST# is asserted again
	 * or not. This bit will be clear automatically when VCC1_RST#
	 * is asserted or power-on reset occurs.
	 */
	inst_scfg->RSTCTL |= BIT(NPCX_RSTCTL_VCC1_RST_SCRATCH);

	if (IS_BIT_SET(inst_scfg->RSTCTL, NPCX_RSTCTL_DBGRST_STS)) {
		data->reset = NPCX_RESET_DEBUG_RST;
		/* Clear debugger reset status initially */
		inst_scfg->RSTCTL |= BIT(NPCX_RSTCTL_DBGRST_STS);
	}
	if (IS_BIT_SET(inst_twd->T0CSR, NPCX_T0CSR_WDRST_STS)) {
		data->reset = NPCX_RESET_WATCHDOG_RST;
		/* Clear watchdog reset status initially */
		inst_twd->T0CSR |= BIT(NPCX_T0CSR_WDRST_STS);
	}

	return 0;
}

static struct lnv_system_npcx_data lnv_system_npcx_dev_data;

static const struct lnv_system_npcx_config lnv_system_dev_cfg = {
	.base_scfg = DT_REG_ADDR(DT_INST(0, nuvoton_npcx_scfg)),
	.base_glue = DT_REG_ADDR_BY_NAME(DT_INST(0, nuvoton_npcx_scfg), glue),
	.base_twd = DT_REG_ADDR(DT_INST(0, nuvoton_npcx_watchdog)),
	.base_mswc =
		DT_REG_ADDR_BY_NAME(DT_INST(0, nuvoton_npcx_host_sub), mswc),
};

static const struct lnv_system_driver_api lnv_system_driver_npcx_api = {
	.get_reset_cause = lnv_system_npcx_get_reset_cause,
	.soc_reset = lnv_system_npcx_soc_reset,
	.hibernate = lnv_system_npcx_hibernate,
	.chip_name = lnv_system_npcx_get_chip_name,
};

DEVICE_DEFINE(lnv_system_npcx_0, "LNV_SYSTEM", lnv_system_npcx_init, NULL,
	      &lnv_system_npcx_dev_data, &lnv_system_dev_cfg, PRE_KERNEL_1,
	      CONFIG_LNV_SYSTEM_NPCX_INIT_PRIORITY,
	      &lnv_system_driver_npcx_api);
