/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <canopennode.h>

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);


#if !DT_NODE_EXISTS(DT_NODELABEL(hall_sw))
#error "Overlay for hall_sw output node not properly defined."
#endif

static const struct gpio_dt_spec hall_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(hall_sw), gpios, {0});
static const struct gpio_dt_spec hall1 =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(hall1), gpios, {0});
static const struct gpio_dt_spec hall2 =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(hall2), gpios, {0});

void us_hall_func(void *d0, void *d1, void *d2);
int hall_enable(uint8_t value);

#define US_HALL_STACK 1024

static bool us_hall_loop = false;
K_THREAD_STACK_DEFINE(us_hall_stack_area, US_HALL_STACK);
static _thread_t us_hall_thread;
static k_tid_t us_hall_tid;
static bool hall_value[2] = {false};


#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

static struct gpio_dt_spec led_green_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(green_led), gpios, {0});
static struct gpio_dt_spec led_red_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(red_led), gpios, {0});

struct led_indicator {
	const struct device *dev;
	gpio_pin_t pin;
};

/**
 * @brief Callback for setting LED indicator state.
 *
 * @param value true if the LED indicator shall be turned on, false otherwise.
 * @param arg argument that was passed when LEDs were initialized.
 */
static void led_callback(bool value, void *arg)
{
	struct gpio_dt_spec *led_gpio = arg;

	if (!led_gpio || !led_gpio->port) {
		return;
	}

	gpio_pin_set_dt(led_gpio, value);
}

/**
 * @brief Configure LED indicators pins and callbacks.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @param nmt CANopenNode NMT object.
 */
static void config_leds(CO_NMT_t *nmt)
{
	int err;

	if (!led_green_gpio.port) {
		LOG_DBG("Green LED not available");
	} else if (!gpio_is_ready_dt(&led_green_gpio)) {
		LOG_ERR("Green LED device not ready");
		led_green_gpio.port = NULL;
	} else {
		err = gpio_pin_configure_dt(&led_green_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("failed to configure Green LED gpio: %d", err);
			led_green_gpio.port = NULL;
		}
	}

	if (!led_red_gpio.port) {
		LOG_DBG("Red LED not available");
	} else if (!gpio_is_ready_dt(&led_red_gpio)) {
		LOG_ERR("Red LED device not ready");
		led_red_gpio.port = NULL;
	} else {
		err = gpio_pin_configure_dt(&led_red_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("failed to configure Red LED gpio: %d", err);
			led_red_gpio.port = NULL;
		}
	}

	canopen_leds_init(nmt,
			  led_callback, &led_green_gpio,
			  led_callback, &led_red_gpio);
}

/**
 * @brief Configure LED indicators pins and callbacks.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @param nmt CANopenNode NMT object.
 */
static void config_powerstates()
{
	int err;
	if (!gpio_is_ready_dt(&hall_sw)) {
		LOG_ERR("The hall switch pin GPIO port is not ready.\n");
		return; // CO_SDO_AB_GENERAL;
	}
	err = gpio_pin_configure_dt(&hall_sw, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Configuring hall switch GPIO pin failed: %d\n", err);
		return; // CO_SDO_AB_GENERAL;
	}
}

// us_hall_func
void us_hall_func(void *d0, void *d1, void *d2) {
	LOG_DBG("start us_hall_func");
    const struct gpio_dt_spec *dev[2];
    dev[0] = &hall1;
    dev[1] = &hall2;
	// bool set_update = false;
    while(us_hall_loop) {
		//set_update = false;
		for (int i=0;i<2;i++) {
			if (dev[i]==NULL) {
				continue;
			}
			if (gpio_pin_get_dt(dev[i]) != 0) {
				hall_value[i] = true;
			}else {
				hall_value[i] = false;
			}

			if ((BOOLEAN)(OD_halSensorMeasurement[i]) != (BOOLEAN)hall_value[i]) {
				CO_LOCK_OD();
				OD_halSensorMeasurement[i] = (BOOLEAN)(hall_value[i]);
				CO_UNLOCK_OD();
			}
		}
        k_sleep(K_MSEC(OD_halSensorPeriode));
    }
}

// Enable hall thread
static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *odf_arg)
{
	LOG_DBG("odf_2102 function");
	if (odf_arg->reading) {
		LOG_INF("Read SDO 0x2102\n");
		return CO_SDO_AB_NONE;
	}

	LOG_INF("Write SDO 0x2102\n");
	uint8_t value;

	if (odf_arg->data == NULL) {
		return CO_SDO_AB_GENERAL;
	}
	value = *odf_arg->data;
	
	if (!hall_enable(value)) {
		return CO_SDO_AB_GENERAL;
	}
	return CO_SDO_AB_NONE;
}

int hall_enable(uint8_t value)
{
	int err;
	
	LOG_DBG("Write SDO 0x2102\n");
	if (!gpio_is_ready_dt(&hall_sw)) {
		LOG_ERR("The hall switch pin GPIO port is not ready.\n");
		return -1;
	}
    if (value != 0) {
		err = gpio_pin_set_dt(&hall_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring hall switch GPIO pin failed: %d\n", err);
    		return -1;
		}
		us_hall_loop = true;
		us_hall_tid = k_thread_create(&us_hall_thread, us_hall_stack_area,
                                 K_THREAD_STACK_SIZEOF(us_hall_stack_area),
                                 us_hall_func,
                                 NULL, NULL, NULL,
                                 K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
		CO_LOCK_OD();
		OD_halSensorEnable = 1;
		CO_UNLOCK_OD();
	} else {
		us_hall_loop = false;
		err = gpio_pin_set_dt(&hall_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting hall switch GPIO pin level failed: %d\n", err);
    		return -1;
		}
		CO_LOCK_OD();
		OD_halSensorEnable = 0;
		CO_UNLOCK_OD();
    }
    return 0;
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

		config_leds(CO->NMT);
		config_powerstates();
		CO_OD_configure(CO->SDO[0], OD_2102_halSensorEnable,
				odf_2102, NULL, 0U, 0U);

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0],
							CO->em);
		}

		hall_enable(OD_halSensorEnable);

		CO_CANsetNormalMode(CO->CANmodule[0]);

		while (true) {
			timeout = 1U; /* default timeout in milliseconds */
			timestamp = k_uptime_get();
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);

			if (reset != CO_RESET_NOT) {
				break;
			}

			if (timeout > 0) {
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

        us_hall_loop = false;

		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	LOG_INF("Resetting device");

	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}