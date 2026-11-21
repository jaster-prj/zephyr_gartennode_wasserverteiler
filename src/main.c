#if defined(CONFIG_BOARD_NATIVE_SIM)
#define _POSIX_C_SOURCE 200809L
#endif

/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <zephyr/drivers/gpio/gpio_emul.h>
#endif
#include <zephyr/drivers/can.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/printk.h>
#include <canopennode.h>
#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <time.h>
K_SEM_DEFINE(native_sim_can_rx_wakeup, 0, 1);
#endif
#include "valve.h"
#include "relais.h"
#include "watchdog.h"
#include "sensor.h"
#include "touch.h"
#include "led.h"

#define LOG_LEVEL CONFIG_CANOPENNODE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#if defined(CONFIG_BOARD_NATIVE_SIM)
static int64_t native_sim_monotonic_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void native_sim_can_bus_activity(const struct device *dev,
					struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frame);
	ARG_UNUSED(user_data);
	k_sem_give(&native_sim_can_rx_wakeup);
}

static void native_sim_update_hall_inputs(void)
{
	static const struct gpio_dt_spec hall[] = {
		GPIO_DT_SPEC_GET(DT_NODELABEL(sensor1), gpios),
		GPIO_DT_SPEC_GET(DT_NODELABEL(sensor2), gpios),
	};

	for (int i = 0; i < ARRAY_SIZE(hall); i++) {
		/* Energized motor windings release the Hall switch during travel. */
		bool moving = OD_relaisState[i * 2] || OD_relaisState[i * 2 + 1];
		bool high = moving || OD_valveState[i] != 0U;

		(void)gpio_emul_input_set(hall[i].port, hall[i].pin, high);
	}
}
#endif

static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2103(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2105(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2107(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2109(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2111(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2112(CO_ODF_arg_t *odf_arg);
static CO_SDO_abortCode_t odf_2113(CO_ODF_arg_t *odf_arg);

#define STORAGE_PARTITION	storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)


/**
 * @brief Watchdog callback function
 *
 * @param channel_id Watchdog channel ID
 * @param user_data Pointer to user data (thread ID)
 */
static void main_watchdog_callback(int channel_id, void *user_data)
{
    LOG_ERR("Watchdog channel %d callback, user data: %p",
        channel_id, k_thread_name_get((k_tid_t)user_data));
	sys_reboot(SYS_REBOOT_COLD);
}

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
	if (touch_init() != 0) {
		LOG_ERR("Touch initialization failed");
	}
	// if (watchdog_init() != 0) {
	// 	LOG_ERR("Watchdog initialization failed");
	// }
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
	
	int ret = sensor_enable(value);
	LOG_DBG("sensor_enable returned %d", ret);
	if (ret == 0 || ret == 1) {
		return CO_SDO_AB_NONE;
	}
	return CO_SDO_AB_GENERAL;
}

/**
 * @brief Set sensor period
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2103(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	uint8_t value;
	value = *odf_arg->data;
	LOG_DBG("Set sensor period to %d ms", value);
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
	if (odf_arg->subIndex == 0U || odf_arg->subIndex > 2U) {
		return CO_SDO_AB_GENERAL;
	}
	uint8_t value;
	value = *odf_arg->data;
	if (value == 0xFF) {
#ifdef CONFIG_CANOPENNODE_STORAGE
		canopen_storage_erase(CANOPEN_STORAGE_EEPROM);
		return CO_SDO_AB_NONE;
#else
		return CO_SDO_AB_GENERAL;
#endif
	}
	if (value >= 8) {
		LOG_ERR("Mapping value not possible");
		return CO_SDO_AB_GENERAL;
	}
	// memcpy(odf_arg->ODdataStorage, odf_arg->data, sizeof(uint8_t));

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
 * @brief Set valve routine time
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2110(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}
	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	uint32_t value;
	value = (*(uint32_t *)(odf_arg->data));
	LOG_DBG("Set valve routine time to %"PRIu32" ms", value);
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
	// memcpy(odf_arg->ODdataStorage, odf_arg->data, sizeof(uint8_t));

	return CO_SDO_AB_NONE;
}

/**
 * @brief Set touch Enable
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2112(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_NONE;
	}

	uint8_t value;

	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	value = *odf_arg->data;
	LOG_DBG("Set touch enable to %d", value);
	int ret = touch_enable(value);
	LOG_DBG("touch_enable returned %d", ret);
	if (ret == 0 || ret == 1) {
		return CO_SDO_AB_NONE;
	}
	return CO_SDO_AB_GENERAL;
}

/**
 * @brief Set touch valve index
 *
 * @param odf_arg SDO argument structure.
 * @return SDO abort code.
 */
static CO_SDO_abortCode_t odf_2113(CO_ODF_arg_t *odf_arg)
{
	if (odf_arg->reading) {
		return CO_SDO_AB_GENERAL;
	}

	uint8_t value;

	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	value = *odf_arg->data;
	
	if (value >= 2) {
		return CO_SDO_AB_GENERAL;
	}
	// memcpy(odf_arg->ODdataStorage, odf_arg->data, sizeof(uint8_t));
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
	int task_id = -1;
#ifdef CONFIG_CANOPENNODE_STORAGE
	int ret;
#endif /* CONFIG_CANOPENNODE_STORAGE */
	can.dev = CAN_INTERFACE;
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface not ready");
		return 0;
	}

#ifdef CONFIG_CANOPENNODE_STORAGE
	LOG_INF("Initializing settings subsystem\n");
	ret = settings_subsys_init();
	if (ret) {
		LOG_ERR("failed to initialize settings subsystem (err = %d)",
			ret);
		int error = 0;
		const struct flash_area *storage = NULL;
		// Open the partition
		error = flash_area_open(PARTITION_ID(storage_partition), &storage);
		if (error < 0) {
			return error;
		}

		// Erase the first sector (e.g., 4KB)
		error = flash_area_erase(storage, 0, PARTITION_SIZE(storage_partition));
		if (error < 0) {
			return error;
		}
		sys_reboot(SYS_REBOOT_COLD);
	}

	ret = settings_load();
	if (ret) {
		LOG_ERR("failed to load settings (err = %d)", ret);
		return 0;
	}
#endif /* CONFIG_CANOPENNODE_STORAGE */

	OD_powerOnCounter++;

	//task_id = watchdog_add(2000, main_watchdog_callback, NULL);
	if (task_id < 0) {
		LOG_ERR("Failed to add watchdog channel");
	}

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
		CO_OD_configure(CO->SDO[0], OD_2103_sensorPeriode,
				odf_2103, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2105_sensorMapping,
				odf_2105, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2107_relaisCmd,
				odf_2107, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2109_valveCmd,
				odf_2109, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2110_valveRoutineTime,
				odf_2110, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2111_valveMapping,
				odf_2111, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2112_touchEnable,
				odf_2112, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2113_touchValveIndex,
				odf_2113, NULL, 0U, 0U);

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0],
							CO->em);
		}

#if defined(CONFIG_BOARD_NATIVE_SIM)
		const struct can_filter native_sim_wakeup_filter = {
			.id = 0U,
			.mask = 0U,
		};
		int native_sim_wakeup_filter_id =
			can_add_rx_filter(can.dev, native_sim_can_bus_activity, NULL,
					  &native_sim_wakeup_filter);
		if (native_sim_wakeup_filter_id < 0) {
			LOG_ERR("Failed to add native-sim CAN activity filter: %d",
				native_sim_wakeup_filter_id);
		}
#endif

#if defined(CONFIG_BOARD_NATIVE_SIM)
		sensor_enable(true);
#else
		sensor_enable(OD_sensorEnable);
#endif

		CO_CANsetNormalMode(CO->CANmodule[0]);
		struct sCO_OD_EEPROM eeprom_old = CO_OD_EEPROM;

		while (true) {
			timeout = 1U; /* default timeout in milliseconds */
#if defined(CONFIG_BOARD_NATIVE_SIM)
			timestamp = native_sim_monotonic_ms();
#else
			timestamp = k_uptime_get();
#endif
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);

			if (reset != CO_RESET_NOT) {
				break;
			}

			/*
			 * Application work must run regardless of the CANopen stack's
			 * requested sleep timeout. In particular, native-sim can request
			 * immediate processing (timeout == 0) while an SDO valve command
			 * is queued.
			 */
			sensor_process();
			relais_process();
			valve_process();
#if defined(CONFIG_BOARD_NATIVE_SIM)
			native_sim_update_hall_inputs();
			/*
			 * CANopenNode's CO_process() handles SDO/NMT only. The optional
			 * sync thread is disabled in native-sim, so explicitly process
			 * TPDOs here. Do not process RPDOs: valve commands remain SDO-only.
			 */
			CO_process_TPDO(CO, false, elapsed * 1000U);
#endif

			if (timeout > 0) {
#ifdef CONFIG_CANOPENNODE_STORAGE
				ret = memcmp(&eeprom_old, &CO_OD_EEPROM, sizeof(CO_OD_EEPROM));
				if (ret != 0) {
					LOG_DBG("EEPROM changed, saving to flash");
					memcpy(&eeprom_old, &CO_OD_EEPROM, sizeof(CO_OD_EEPROM));
					ret = canopen_storage_save(
						CANOPEN_STORAGE_EEPROM);
					if (ret) {
						LOG_ERR("failed to save EEPROM");
					}
				}

#endif /* CONFIG_CANOPENNODE_STORAGE */
				if (task_id >= 0) {
					int ret = watchdog_feed(task_id);
					if (ret != 0) {
						LOG_ERR("Failed to feed watchdog channel %d: %d", task_id, ret);
					}
				}
				/*
				 * Try to sleep for as long as the
				 * stack requested and calculate the
				 * exact time elapsed.
				 */
#if defined(CONFIG_BOARD_NATIVE_SIM)
				uint32_t wait_ms = MAX(timeout, 10U);
				(void)k_sem_take(&native_sim_can_rx_wakeup, K_MSEC(wait_ms));
				elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
				if (elapsed == 0U) {
					elapsed = 1U;
				}
#else
				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
#endif
			} else {
				/*
				 * Do not sleep, more processing to be
				 * done by the stack.
				 */
#if defined(CONFIG_BOARD_NATIVE_SIM)
				/*
				 * Keep CANopen timers advancing when the stack requests
				 * immediate processing on native_sim.
				 */
				(void)k_sem_take(&native_sim_can_rx_wakeup, K_MSEC(10));
				elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
				if (elapsed == 0U) {
					elapsed = 1U;
				}
#else
				elapsed = 0U;
#endif
			}
		}

        sensor_enable(false);

		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}
	if (task_id >= 0) {
		int ret = watchdog_delete(task_id);
		if (ret != 0) {
			LOG_ERR("Failed to delete watchdog channel %d: %d", task_id, ret);
		}
	}

	LOG_INF("Resetting device");

	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}
