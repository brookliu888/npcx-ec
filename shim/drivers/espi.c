/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <i8042_protocol.h>

LOG_MODULE_REGISTER(espi_shim, CONFIG_ESPI_LOG_LEVEL);

/**
 * @file
 * @brief Nuvoton shimmed eSPI driver
 *
 * This file contains the drivers of NPCX Host Sub-Modules that serve as an
 * interface between the Host and Core domains. Please refer the block diagram.
 *
 *                                        +------------+
 *                                        |   Serial   |---> TXD
 *                                  +<--->|    Port    |<--- RXD
 *                                  |     |            |<--> ...
 *                                  |     +------------+
 *                                  |     +------------+     |
 *                +------------+    |<--->|  KBC & PM  |<--->|
 *   eSPI_CLK --->|  eSPI Bus  |    |     |  Channels  |     |
 *   eSPI_RST --->| Controller |    |     +------------+     |
 * eSPI_IO3-0 <-->|            |<-->|     +------------+     |
 *    eSPI_CS --->| (eSPI mode)|    |     |   Shared   |     |
 * eSPI_ALERT <-->|            |    |<--->|   Memory   |<--->|
 *                +------------+    |     +------------+     |
 *                                  |     +------------+     |
 *                                  |<--->|    MSWC    |<--->|
 *                                  |     +------------+     |
 *                                  |     +------------+     |
 *                                  |     |    Core    |     |
 *                                  |<--->|   to Host  |<--->|
 *                                  |     |   Access   |     |
 *                                  |     +------------+     |
 *                                HMIB                       | Core Bus
 *                     (Host Modules Internal Bus)           +------------
 *
 * @code{.dts}
 *     host_sub {
 *            reg = <0x400c1000 0x2000
 *                   0x40010000 0x2000
 *                   0x4000e000 0x2000
 *                   0x400c7000 0x2000
 *                   0x400c9000 0x2000
 *                   0x400cb000 0x2000>;
 *                    cache-level = <2>;
 *            reg-names = "mswc", "shm", "c2h", "kbc", "pm_acpi", "pm_hcmd";
 * };
 * @endcode
 *
 * This driver implements host sub-modules via eSPI bus. It includes:
 *
 * 1. Keyboard and Mouse Controller (KBC) interface:
 *   ● Kconfig option: CONFIG_ESPI_PERIPHERAL_8042_KBC
 *   ● DT node: host_sub.reg_names: "kbc"
 *   ● Intel 8051SL-compatible Host interface
 *     — 8042 KBD standard interface (ports 60h, 64h)
 *     — Legacy IRQ: IRQ1 (KBD) and IRQ12 (mouse) support
 *   ● Configured by two logical devices: Keyboard and Mouse (LDN 0x06/0x05)
 *
 * 2. Power Management Channel for Advanced Configuration and Power Interface (ACPI)
 *   ● Kconfig option: CONFIG_ESPI_PERIPHERAL_HOST_IO
 *   ● DT node: host_sub.reg_names: "pm_acpi" (ie. PM1)
 *   ● PM channel registers
 *     — Command/Status register
 *     — Data register
 *       channel 1: legacy 62h, 66h; channel 2: legacy 68h, 6Ch;
 *       channel 3: legacy 6Ah, 6Eh; channel 4: legacy 6Bh, 6Fh;
 *   ● PM interrupt using:
 *     — Serial IRQ
 *     — SMI
 *     — EC_SCI
 *   ● Configured by four logical devices: PM1/2/3/4 (LDN 0x11/0x12/0x17/0x1E)
 *
 * 3. Port 80 debug Mechanism
 *   It is supported by Shared Memory mechanism (SHM) which also allows sharing
 *   of the on-chip RAM by both Core and the Host.
 *   ● Kconfig option: CONFIG_ESPI_PERIPHERAL_DEBUG_PORT80
 *   ● DT node: host_sub.reg_names: "shm"
 *   ● Port 80 debug support
 *   ● Configured by one logical device: SHM (LDN 0x0F)
 *
 * 4. Serial Port (Legacy UART) Support
 *   It provides UART functionality by supporting serial data communication
 *   with a remote peripheral device or a modem.
 *   ● Kconfig option: CONFIG_ESPI_PERIPHERAL_UART
 *   ● DT node: host_uart for pin-muxing
 *   ● Configured by one logical device: Serial Port (LDN 0x03)
 *
 * 5. Chrome OS Host Command Protocol Support.
 *   This module allows sharing of the on-chip RAM by both Core and the Host.
 *   It also supports the following features:
 *   ● Kconfig option: ESPI_PERIPHERAL_EC_HOST_CMD
 *   ● DT node: host_sub.reg_names: "pm_hcmd" (ie. PM2) and "shm"
 *   ● One Core/Host communication windows for direct RAM access - Command content
 *   ● One Power Management (PM) Channel - Comaand protocol hand-shaking
 *   ● Configured by one logical device: SHM (LDN 0x0F)
 *
 */

#define espi_dev DEVICE_DT_GET(DT_NODELABEL(espi0))

#define VWIRE_PULSE_TRIGGER_TIME 1000 /* Unit: us */
/* LPC command status byte masks */
/* EC has written a byte in the data register and host hasn't read it yet */
#define EC_LPC_STATUS_TO_HOST 0x01
/* Host has written a command/data byte and the EC hasn't read it yet */
#define EC_LPC_STATUS_FROM_HOST 0x02
/* EC is processing a command */
#define EC_LPC_STATUS_PROCESSING 0x04
/* Last write to EC was a command, not data */
#define EC_LPC_STATUS_LAST_CMD 0x08
/* EC is in burst mode */
#define EC_LPC_STATUS_BURST_MODE 0x10
/* SCI event is pending (requesting SCI query) */
#define EC_LPC_STATUS_SCI_PENDING 0x20
/* SMI event is pending (requesting SMI query) */
#define EC_LPC_STATUS_SMI_PENDING 0x40
/* (reserved) */
#define EC_LPC_STATUS_RESERVED 0x80

void kbc_host_write_data(int data, int is_cmd)
{
	/* TODO: inform upper layer we got 8042 data/command */
}

bool kbc_is_8042_ibf(uint32_t data)
{
	struct espi_evt_data_kbc *kbc = (struct espi_evt_data_kbc *)&data;

	return kbc->evt & HOST_KBC_EVT_IBF;
}

bool kbc_is_8042_obe(uint32_t data)
{
	struct espi_evt_data_kbc *kbc = (struct espi_evt_data_kbc *)&data;

	return kbc->evt & HOST_KBC_EVT_OBE;
}

uint32_t kbc_get_8042_type(uint32_t data)
{
	struct espi_evt_data_kbc *kbc = (struct espi_evt_data_kbc *)&data;

	return kbc->type;
}

uint32_t kbc_get_8042_data(uint32_t data)
{
	struct espi_evt_data_kbc *kbc = (struct espi_evt_data_kbc *)&data;

	return kbc->data;
}

int espi_vw_set_wire(enum espi_vwire_signal signal, uint8_t level)
{
	int ret = espi_send_vwire(espi_dev, signal, level);

	if (ret != 0)
		LOG_ERR("Encountered error sending virtual wire signal");

	return ret;
}

static void espi_generate_sci(void)
{
	/* Enforce signal-high for long enough to debounce high */
	espi_vw_set_wire(ESPI_VWIRE_SIGNAL_SCI, 1);
	k_busy_wait(VWIRE_PULSE_TRIGGER_TIME);
	espi_vw_set_wire(ESPI_VWIRE_SIGNAL_SCI, 0);
	k_busy_wait(VWIRE_PULSE_TRIGGER_TIME);
	espi_vw_set_wire(ESPI_VWIRE_SIGNAL_SCI, 1);
}

bool acpi_is_command(uint32_t data)
{
	struct espi_evt_data_acpi *acpi = (struct espi_evt_data_acpi *)&data;

	return acpi->type;
}

uint32_t acpi_get_value(uint32_t data)
{
	struct espi_evt_data_acpi *acpi = (struct espi_evt_data_acpi *)&data;

	return acpi->data;
}

int acpi_ap_to_ec(int is_cmd, uint8_t value, uint8_t *resultptr)
{
	/* TODO: Process ACPI command from EC region */
	return 0;
}

/* eSPI Callback Functions for Channel Ready/Not Ready Events */
void espi_channel_en_handler(const struct device *dev,
			     struct espi_callback *cb,
			     struct espi_event event)
{
	LOG_DBG("ESPI event type 0x%x %d:%d", event.evt_type, event.evt_details,
		event.evt_data);

	/* Virtual wire channel status change */
	if (event.evt_details == ESPI_CHANNEL_VWIRE) {
		if (event.evt_data) {
			/* TODO: Do something if VW Channel is ready */
		} else {
			/* TODO: Do something if VW Channel is not ready */
		}
	}
}

/* eSPI Callback Functions for Receiving Vire-Wire Events */
static void espi_vwire_handler(const struct device *dev,
			       struct espi_callback *cb,
			       struct espi_event event)
{
	LOG_DBG("ESPI event type 0x%x %d:%d", event.evt_type, event.evt_details,
		event.evt_data);

	/* If PLTRST# asserted (low) then send reset hook */
	if (event.evt_details == ESPI_VWIRE_SIGNAL_PLTRST &&
	    event.evt_data == 0) {
		/* TODO: Do something when PLTRST# is asserted */
	} else if (event.evt_details == ESPI_VWIRE_SIGNAL_PLTRST &&
		   event.evt_data == 1) {
		/* TODO: Do something when PLTRST# is de-asserted */
	}
}

/* eSPI Callback Functions for Peripheral Events */
static void espi_periph_port_80_write(int data)
{
	/* TODO: handle port 80 data here */
}

static void espi_periph_acpi_write(uint32_t data)
{
	uint8_t value, result;
	uint8_t is_cmd = acpi_is_command(data);
	uint32_t status;
	int rv;

	value = acpi_get_value(data);

	/* Handle whatever this was. */
	if (acpi_ap_to_ec(is_cmd, value, &result)) {
		data = result;
		rv = espi_write_lpc_request(espi_dev, EACPI_WRITE_CHAR, &data);
		if (rv) {
			LOG_ERR("ESPI write failed: EACPI_WRITE_CHAR = %d", rv);
		}
	}

	/* Clear processing flag */
	rv = espi_read_lpc_request(espi_dev, EACPI_READ_STS, &status);
	if (rv) {
		LOG_ERR("ESPI read failed: EACPI_READ_STS = %d", rv);
	} else {
		status &= ~EC_LPC_STATUS_PROCESSING;
		rv = espi_write_lpc_request(espi_dev, EACPI_WRITE_STS, &status);
		if (rv) {
			LOG_ERR("ESPI write failed: EACPI_WRITE_STS = %d", rv);
		}
	}

	/*
	 * ACPI 5.0-12.6.1: Generate SCI for Input Buffer Empty / Output Buffer
	 * Full condition on the kernel channel.
	 */
	espi_generate_sci();
}

static void espi_periph_kbc_ibf_obe(uint32_t data)
{
	uint8_t is_ibf = kbc_is_8042_ibf(data);
	uint32_t status = I8042_AUX_DATA;
	int rv;

	if (is_ibf) {
		int data = kbc_get_8042_data(data);
		int is_cmd = kbc_get_8042_type(data);

		kbc_host_write_data(data, is_cmd);
	} else {
		/* Host has read kbc data out */
		rv = espi_write_lpc_request(espi_dev, E8042_CLEAR_FLAG,
					    &status);
		if (rv) {
			LOG_ERR("ESPI write failed: E8042_CLEAR_FLAG = %d", rv);
		}
	}

	/* TODO: wake up task which is in charge of 8042 */
}

static void espi_peripheral_handler(const struct device *dev,
				    struct espi_callback *cb,
				    struct espi_event event)
{
	uint16_t event_type = event.evt_details;

	if (event_type == ESPI_PERIPHERAL_DEBUG_PORT80) {
		espi_periph_port_80_write(event.evt_data);
	}

	if (event_type == ESPI_PERIPHERAL_HOST_IO) {
		espi_periph_acpi_write(event.evt_data);
	}

	if (event_type == ESPI_PERIPHERAL_8042_KBC) {
		espi_periph_kbc_ibf_obe(event.evt_data);
	}
}

static int zephyr_shim_espi_setup(void)
{
	uint32_t enable = 1;
	static const struct {
		espi_callback_handler_t handler;
		enum espi_bus_event event_type;
	} callbacks[] = {
		/* Peripheral events such as KBC, ACPI and port80 */
		{
			.handler = espi_peripheral_handler,
			.event_type = ESPI_BUS_PERIPHERAL_NOTIFICATION,
		},
		/* Receiving Vire-Wire events such as ESPI_VWIRE_SIGNAL_XXXX */
		{
			.handler = espi_vwire_handler,
			.event_type = ESPI_BUS_EVENT_VWIRE_RECEIVED,
		},
		/* eSPI channels Ready/Not Ready */
		{
			.handler = espi_channel_en_handler,
			.event_type = ESPI_BUS_EVENT_CHANNEL_READY,
		},
	};
	static struct espi_callback cb[ARRAY_SIZE(callbacks)];

	struct espi_cfg cfg = {
		.io_caps = ESPI_IO_MODE_QUAD_LINES,
		.channel_caps = ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL |
				ESPI_CHANNEL_OOB,
		.max_freq = 50,
	};

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("espi device isn't ready!");
	}

	/* Setup callbacks */
	for (size_t i = 0; i < ARRAY_SIZE(callbacks); i++) {
		espi_init_callback(&cb[i], callbacks[i].handler,
				   callbacks[i].event_type);
		espi_add_callback(espi_dev, &cb[i]);
	}

	/* Configure eSPI after callbacks are registered */
	if (espi_config(espi_dev, &cfg)) {
		LOG_ERR("Failed to configure eSPI device");
		return -1;
	}

	/* Enable host interface interrupts */
	espi_write_lpc_request(espi_dev, ECUSTOM_HOST_SUBS_INTERRUPT_EN,
			       &enable);
	return 0;
}

/* Must be before zephyr_shim_setup_hooks. */
SYS_INIT(zephyr_shim_espi_setup, APPLICATION, 0);
