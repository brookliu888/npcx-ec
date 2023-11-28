/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/gpio/nuvoton-npcx-gpio.h>

#include <drivers/lnv_system.h>
#include <stdlib.h>
#include <soc_dt.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define NODE_OUTPUT	DT_ALIAS(gpio_out)
#define NODE_INPUT	DT_ALIAS(gpio_in)
#define NODE_OBSERVE	DT_ALIAS(gpio_obs)

/* POC GPIO objects */
struct gpio_poc_spec {
	/* A npcx gpio port device */
	const struct device *dev;
	/* A npcx gpio port number */
	int port;
	/* Bit number of pin within a npcx gpio port */
	gpio_pin_t pin;
	/* GPIO net name */
	const char *name;
	/** The pin's configuration flags as specified in devicetree */
	int flags;
};

/* define your configurations from board dts file */
#define POC_GPIO_OBJS_GET(node)						\
	{								\
		.dev = DEVICE_DT_GET(DT_GPIO_CTLR(node, gpios)),	\
		.port = DT_PROP(DT_GPIO_CTLR(node, gpios), index),	\
		.pin = DT_GPIO_PIN(node, gpios),			\
		.name = DT_NODE_FULL_NAME(node),			\
		.flags = DT_GPIO_FLAGS(node, gpios),			\
	}

static const struct gpio_poc_spec gpio_objs[] = {
	DT_FOREACH_CHILD_SEP(DT_PATH(poc_gpio), POC_GPIO_OBJS_GET, (,))
};

static const struct pwm_dt_spec pwm_objs[] = {
	DT_FOREACH_CHILD_SEP(DT_PATH(poc_leds_pwm), PWM_DT_SPEC_GET, (,))
};

struct gpio_callback gpio_cb;

static void gpio_isr(const struct device *port, struct gpio_callback *cb,
			  gpio_port_pins_t pins)
{
	const struct device *const dev_obs =
		DEVICE_DT_GET(DT_GPIO_CTLR(NODE_OBSERVE, gpios));
	gpio_pin_t pin_obs =
		DT_GPIO_PIN(NODE_OBSERVE, gpios);

	/* drive observe pin to low */
	gpio_pin_set_raw(dev_obs, pin_obs, 0);

	/* Never use shell_print in ISR */
	LOG_INF("%08x is pressed", pins);
	/* drive observe pin to high */
	gpio_pin_set_raw(dev_obs, pin_obs, 1);
}

static int cmd_gpio_list_all(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Print header */
	shell_print(sh, "\tIDX|LVL| GPIO | Name");
	shell_print(sh, "\t---+---+------+----------");

	/* List all GPIOs in 'poc-gpios' and 'unused_pins' DT nodes */
	for (int i = 0; i < ARRAY_SIZE(gpio_objs); i++) {
		shell_print(sh, "\t%02d | %s | io%x%x | %s",
			    i,
			    gpio_pin_get(gpio_objs[i].dev, gpio_objs[i].pin) == 1 ? "H" : "L",
			    gpio_objs[i].port,
			    gpio_objs[i].pin, gpio_objs[i].name);
	}

	return 0;
}

static int cmd_gpio_trig(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *const dev_out =
		DEVICE_DT_GET(DT_GPIO_CTLR(NODE_OUTPUT, gpios));
	gpio_pin_t pin_out = DT_GPIO_PIN(NODE_OUTPUT, gpios);
	const struct device *const dev_in =
		DEVICE_DT_GET(DT_GPIO_CTLR(NODE_INPUT, gpios));
	gpio_pin_t pin_in = DT_GPIO_PIN(NODE_INPUT, gpios);
	int ret;

	ret = gpio_pin_interrupt_configure(dev_in, pin_in, GPIO_INT_EDGE_RISING);
	if (ret) {
		shell_error(sh, "Configure interrupt fail.\n");
		return ret;
	}

	gpio_init_callback(&gpio_cb, gpio_isr, BIT(pin_in));
	ret = gpio_add_callback(dev_in, &gpio_cb);
	if (ret) {
		shell_error(sh, "Configure callback fail.\n");
		return ret;
	}

	/* drive to low */
	gpio_pin_set_raw(dev_out, pin_out, 0);

	/* drive to high later */
	gpio_pin_set_raw(dev_out, pin_out, 1);

	return 0;
}

static int cmd_pwm_list_all(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Print header */
	shell_print(sh, "\tIDX| CHN | Period  | Name");
	shell_print(sh, "\t---+-----+---------+----------");

	/* List all GPIOs in 'poc-leds-pwm' and 'unused_pins' DT nodes */
	for (int i = 0; i < ARRAY_SIZE(pwm_objs); i++) {
		shell_print(sh, "\t%02d | %02d | %d ms  | %s",
			    i,
			    pwm_objs[i].channel,
			    pwm_objs[i].period/1000000,
			    pwm_objs[i].dev->name);
	}

	return 0;
}

static int cmd_system_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *sys_dev = device_get_binding("LNV_SYSTEM");
	int chip_reset_cause = lnv_system_get_reset_cause(sys_dev);

	shell_print(sh, "\tChip: \t%s", lnv_system_chip_name(sys_dev));

	if ((chip_reset_cause & NPCX_RESET_POWERUP) != 0) {
		int psl_evt = chip_reset_cause & NPCX_RESET_POWERUP_PSL_MASK;

		shell_print(sh, "\tReset Cause: \tPower-on restart");

		while (psl_evt) {
			int bit = find_lsb_set(psl_evt) - 1;

			shell_print(sh, "\t\t\tPSL_IN%d triggered!", bit - 1);
			psl_evt &= ~(1U << bit);
		}
	} else if ((chip_reset_cause & NPCX_RESET_VCC1_RST_PIN) != 0) {
		shell_print(sh, "\tReset Cause: \tReset-Pin pressed");
	} else if ((chip_reset_cause & NPCX_RESET_DEBUG_RST) != 0) {
		shell_print(sh, "\tReset Cause: \tReset by debugger");
	} else if ((chip_reset_cause & NPCX_RESET_WATCHDOG_RST) != 0) {
		shell_print(sh, "\tReset Cause: \tWatchdog reset");
	} else {
		shell_print(sh, "\tReset Cause: \tUnknown reset");
	}

	return 0;
}

static int cmd_system_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *sys_dev = device_get_binding("LNV_SYSTEM");

	shell_print(sh, "\tSystem reset now!");
	k_msleep(100);
	lnv_system_soc_reset(sys_dev);

	return 0;
}

static int cmd_system_hibernate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *sys_dev = device_get_binding("LNV_SYSTEM");

	shell_print(sh, "\tSystem hibernating!");
	k_msleep(100);
	lnv_system_hibernate(sys_dev, 0, 0);

	return 0;
}



/* Main entry */
int main(void)
{
	/* Configure all GPIOs from `poc-gpio` dt node */
	for (int i = 0; i <ARRAY_SIZE(gpio_objs); i++) {
		gpio_pin_configure(gpio_objs[i].dev,
				   gpio_objs[i].pin,
				   gpio_objs[i].flags);
	}

	/* Configure all PWMs from `poc-leds-pwm` dt node as 50% duty cycle */
	for (int i = 0; i <ARRAY_SIZE(pwm_objs); i++) {
		uint32_t pulse_width = pwm_objs[i].period / 2;

		pwm_set_pulse_dt(&pwm_objs[i], pulse_width);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_gpio,
	SHELL_CMD_ARG(list, NULL, "List all GPIOs used on platform by index",
		      cmd_gpio_list_all, 1, 0),
	SHELL_CMD_ARG(go, NULL, "Trigger a GPIO event and observe interrupt latency",
		      cmd_gpio_trig, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(gpio, &sub_gpio, "gpio commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pwm,
	SHELL_CMD_ARG(list, NULL, "List all LEDs used on platform by index",
		      cmd_pwm_list_all, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(pwm, &sub_pwm, "pwm commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_system,
	SHELL_CMD_ARG(info, NULL, "List all system information",
		      cmd_system_info, 1, 0),
	SHELL_CMD_ARG(reset, NULL, "System reset now",
		      cmd_system_reset, 1, 0),
	SHELL_CMD_ARG(hibernate, NULL, "System hibernating",
		      cmd_system_hibernate, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(sys, &sub_system, "system commands", NULL);
