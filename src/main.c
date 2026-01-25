/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <canopennode.h>
#include "valve.h"
#include "relais.h"
#include "watchdog.h"
#include "sensor.h"
#include "led.h"

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2105(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2107(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2109(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2111(CO_ODF_arg_t *odf_arg);

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

/**
 * @brief Configure LED indicators pins and callbacks.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @param nmt CANopenNode NMT object.
 */
static void initialize()
{
	if (led_init() != 0) {
		LOG_ERR("LED initialization failed");
	}
	if (sensor_init() != 0) {
		LOG_ERR("Sensor initialization failed");
	}
	if (relais_init() != 0) {
		LOG_ERR("Relais initialization failed");
	}
}

/**
 * @brief Set sensor Enable
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}

	uint8_t value;

	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	value = *odf_arg->data;
	
	if (!sensor_enable(value)) {
		return CO_SDO_AB_GENERAL;
	}
	return CO_SDO_AB_NONE;
}

/**
 * @brief Set sensor Mapping
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2105(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	if (odf_arg->subIndex == 0U || odf_arg->subIndex > 4U) {
		return CO_SDO_AB_GENERAL;
	}
	uint8_t value;
	value = *odf_arg->data;
	if (value >= 4) {
		LOG_ERR("Mapping value not possible");
		return CO_SDO_AB_GENERAL;
	}
	renew_sensor_mapping();
	return CO_SDO_AB_NONE;
}

/**
 * @brief Set relais command
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2107(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	if (odf_arg->subIndex == 0U) {
		return CO_SDO_AB_GENERAL;
	}

	uint8_t value;
	value = *odf_arg->data;
	// if (relais_set(odf_arg->subIndex - 1, (bool)value) != 0) {
	// 	return CO_SDO_AB_GENERAL;
	// }
	if (valve_set_state((odf_arg->subIndex - 1)>>1, 0xFF) != 0) {
		return CO_SDO_AB_GENERAL;
	}
	return CO_SDO_AB_NONE;
}

/**
 * @brief Set valve order
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2109(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	if (odf_arg->subIndex == 0U) {
		return CO_SDO_AB_GENERAL;
	}
	if (odf_arg->subIndex > 2) {
		LOG_ERR("SubIndex not possible");
		return CO_SDO_AB_GENERAL;
	}
	uint8_t value;
	value = *odf_arg->data;
	if (valve_order(odf_arg->subIndex - 1, (bool)value) != 0) {
		return CO_SDO_AB_GENERAL;
	}
	return CO_SDO_AB_NONE;
}

/**
 * @brief Set valve Mapping
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2111(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	if (odf_arg->subIndex == 0U || odf_arg->subIndex > 4U) {
		return CO_SDO_AB_GENERAL;
	}
	uint8_t value;
	value = *odf_arg->data;
	if (value >= 4) {
		LOG_ERR("Mapping value not possible");
		return CO_SDO_AB_GENERAL;
	}

	return CO_SDO_AB_NONE;
}

/**
 * @brief Main application entry point.
 *
 * The main application thread is responsible for initializing the
 * CANopen stack and doing the non real-time processing.
 */
int main(void)
{
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	CO_ReturnError_t err;
	struct canopen_context can;
	uint16_t timeout;
	uint32_t elapsed;
	int64_t timestamp;
#ifdef CONFIG_CANOPENNODE_STORAGE
	int ret;
#endif /* CONFIG_CANOPENNODE_STORAGE */
	can.dev = CAN_INTERFACE;
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface not ready");
		return 0;
	}

#ifdef CONFIG_CANOPENNODE_STORAGE
	ret = settings_subsys_init();
	if (ret) {
		LOG_ERR("failed to initialize settings subsystem (err = %d)",
			ret);
		return 0;
	}

	ret = settings_load();
	if (ret) {
		LOG_ERR("failed to load settings (err = %d)", ret);
		return 0;
	}
#endif /* CONFIG_CANOPENNODE_STORAGE */

	OD_powerOnCounter++;

	while (reset != CO_RESET_APP) {
		elapsed =  0U; /* milliseconds */

		LOG_INF("CANopen stack initialize");
		err = CO_init(&can, CONFIG_CANOPEN_NODE_ID, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			return 0;
		}
		LOG_INF("CANopen stack initialized");

#ifdef CONFIG_CANOPENNODE_STORAGE
		canopen_storage_attach(CO->SDO[0], CO->em);
#endif /* CONFIG_CANOPENNODE_STORAGE */

		initialize();
		led_configure(CO->NMT);
		CO_OD_configure(CO->SDO[0], OD_2102_sensorEnable,
				odf_2102, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2105_sensorMapping,
				odf_2105, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2107_relaisCmd,
				odf_2107, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2109_valveCmd,
				odf_2109, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2111_valveMapping,
				odf_2111, NULL, 0U, 0U);

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0],
							CO->em);
		}

		sensor_enable(OD_sensorEnable);

		CO_CANsetNormalMode(CO->CANmodule[0]);

		while (true) {
			timeout = 1U; /* default timeout in milliseconds */
			timestamp = k_uptime_get();
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);

			if (reset != CO_RESET_NOT) {
				break;
			}

			if (timeout > 0) {
				sensor_process();
				relais_process();
				valve_process();
#ifdef CONFIG_CANOPENNODE_STORAGE
				ret = canopen_storage_save(
					CANOPEN_STORAGE_EEPROM);
				if (ret) {
					LOG_ERR("failed to save EEPROM");
				}
#endif /* CONFIG_CANOPENNODE_STORAGE */
				/*
				 * Try to sleep for as long as the
				 * stack requested and calculate the
				 * exact time elapsed.
				 */
				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
			} else {
				/*
				 * Do not sleep, more processing to be
				 * done by the stack.
				 */
				elapsed = 0U;
			}
		}

        sensor_enable(false);

		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	LOG_INF("Resetting device");

	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}