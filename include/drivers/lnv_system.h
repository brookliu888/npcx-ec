/* Copyright 2021 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/**
 * @file
 * @brief Public API for lenovo system drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LNV_SYSTEM_H_
#define ZEPHYR_INCLUDE_DRIVERS_LNV_SYSTEM_H_

/**
 * @brief lenovo system Interface
 * @defgroup lnv_system_interface lenovo system Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Identify the reset cause.
 */
/* the reset is triggered by VCC power-up */
#define NPCX_RESET_POWERUP		BIT(0)
/* the reset is triggered by VCC power-up caused by PSL_IN0 event */
#define NPCX_RESET_POWERUP_PSL_IN0	BIT(1)
/* the reset is triggered by VCC power-up caused by PSL_IN1 event */
#define NPCX_RESET_POWERUP_PSL_IN1	BIT(2)
/* the reset is triggered by VCC power-up caused by PSL_IN2 event */
#define NPCX_RESET_POWERUP_PSL_IN2	BIT(3)
/* the reset is triggered by VCC power-up caused by PSL_IN3 event */
#define NPCX_RESET_POWERUP_PSL_IN3	BIT(4)
/* the reset is triggered by VCC power-up caused by PSL_IN4 event */
#define NPCX_RESET_POWERUP_PSL_IN4	BIT(5)
/* the reset is triggered by VCC power-up caused by PSL_IN5 event */
#define NPCX_RESET_POWERUP_PSL_IN5	BIT(6)
/* the reset is triggered by external VCC1 reset pin */
#define NPCX_RESET_VCC1_RST_PIN		BIT(7)
/* the reset is triggered by ICE debug reset request */
#define NPCX_RESET_DEBUG_RST		BIT(8)
/* the reset is triggered by watchdog */
#define NPCX_RESET_WATCHDOG_RST		BIT(9)

#define NPCX_RESET_POWERUP_PSL_MASK	(NPCX_RESET_POWERUP_PSL_IN0 | NPCX_RESET_POWERUP_PSL_IN1 | \
					 NPCX_RESET_POWERUP_PSL_IN2 | NPCX_RESET_POWERUP_PSL_IN3 | \
					 NPCX_RESET_POWERUP_PSL_IN4 | NPCX_RESET_POWERUP_PSL_IN5)

/**
 * @brief Get a node from path '/hibernate_wakeup_pins' which has a property
 *        'wakeup-pins' contains GPIO list for hibernate wake-up
 *
 * @return node identifier with that path.
 */
#define SYSTEM_DT_NODE_HIBERNATE_CONFIG DT_INST(0, lnv_ec_hibernate_wake_pins)

/**
 * @typedef lnv_system_get_reset_cause_api
 * @brief Callback API for getting reset cause instance.
 * See lnv_system_get_reset_cause() for argument descriptions
 */
typedef int (*lnv_system_get_reset_cause_api)(const struct device *dev);

/**
 * @typedef lnv_system_soc_reset_api
 * @brief Callback API for soc-reset instance.
 * See lnv_system_soc_reset() for argument descriptions
 */
typedef int (*lnv_system_soc_reset_api)(const struct device *dev);

/**
 * @typedef lnv_system_hibernate_api
 * @brief Callback API for entering hibernate state (lowest EC power state).
 * See lnv_system_hibernate() for argument descriptions
 */
typedef int (*lnv_system_hibernate_api)(const struct device *dev,
					 uint32_t seconds,
					 uint32_t milenovoeconds);

/**
 * @typedef lnv_system_chip_vendor_api
 * @brief Callback API for getting the chip vendor.
 * See lnv_system_chip_vendor() for argument descriptions
 */
typedef const char *(*lnv_system_chip_vendor_api)(const struct device *dev);

/**
 * @typedef lnv_system_chip_name_api
 * @brief Callback API for getting the chip name.
 * See lnv_system_chip_name() for argument descriptions
 */
typedef const char *(*lnv_system_chip_name_api)(const struct device *dev);

/** @brief Driver API structure. */
__subsystem struct lnv_system_driver_api {
	lnv_system_get_reset_cause_api get_reset_cause;
	lnv_system_soc_reset_api soc_reset;
	lnv_system_hibernate_api hibernate;
	lnv_system_chip_name_api chip_name;
};


/**
 * @brief Get the chip-reset cause
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval non-negative if successful.
 * @retval Negative errno code if failure.
 */
__syscall int lnv_system_get_reset_cause(const struct device *dev);

static inline int z_impl_lnv_system_get_reset_cause(const struct device *dev)
{
	const struct lnv_system_driver_api *api =
		(const struct lnv_system_driver_api *)dev->api;

	if (!api->get_reset_cause) {
		return -ENOTSUP;
	}

	return api->get_reset_cause(dev);
}

/**
 * @brief reset the soc
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval no return if successful.
 * @retval Negative errno code if failure.
 */
__syscall int lnv_system_soc_reset(const struct device *dev);

static inline int z_impl_lnv_system_soc_reset(const struct device *dev)
{
	const struct lnv_system_driver_api *api =
		(const struct lnv_system_driver_api *)dev->api;

	if (!api->soc_reset) {
		return -ENOTSUP;
	}

	return api->soc_reset(dev);
}

/**
 * @brief put the EC in hibernate (lowest EC power state).
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param seconds Number of seconds before EC enters hibernate state.
 * @param milenovoeconds Number of micro-secs before EC enters hibernate state.

 * @retval no return if successful.
 * @retval Negative errno code if failure.
 */
__syscall int lnv_system_hibernate(const struct device *dev, uint32_t seconds,
				    uint32_t milenovoeconds);

static inline int z_impl_lnv_system_hibernate(const struct device *dev,
					       uint32_t seconds,
					       uint32_t milenovoeconds)
{
	const struct lnv_system_driver_api *api =
		(const struct lnv_system_driver_api *)dev->api;

	if (!api->hibernate) {
		return -ENOTSUP;
	}

	return api->hibernate(dev, seconds, milenovoeconds);
}

/**
 * @brief Get the chip name.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @retval Chip name string if successful.
 * @retval Null string if failure.
 */
__syscall const char *lnv_system_chip_name(const struct device *dev);

static inline const char *z_impl_lnv_system_chip_name(const struct device *dev)
{
	const struct lnv_system_driver_api *api =
		(const struct lnv_system_driver_api *)dev->api;

	if (!api->chip_name) {
		return "";
	}

	return api->chip_name(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/lnv_system.h>
#endif /* ZEPHYR_INCLUDE_DRIVERS_LNV_SYSTEM_H_ */
