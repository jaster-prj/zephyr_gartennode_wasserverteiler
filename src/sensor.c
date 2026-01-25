#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include "sensor.h"
#include "watchdog.h"
#include "valve.h"

#define SENSOR_STACK 1024

LOG_MODULE_REGISTER(sensor);
K_THREAD_STACK_DEFINE(sensor_stack_area, SENSOR_STACK);

static void sensor_watchdog_callback(int channel_id, void *user_data);

typedef struct {
	uint8_t idx;
	bool state;
	bool value;
}sensor_valve_state_t;

static struct gpio_dt_spec sensor_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(sensor_sw), gpios, {0});
static struct gpio_dt_spec sensor1 =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(sensor1), gpios, {0});
static struct gpio_dt_spec sensor2 =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(sensor2), gpios, {0});

static struct gpio_dt_spec *sensors[2] = {
	&sensor1,
	&sensor2
};

static sensor_valve_state_t sensor_valve_state[2] = {
	{0, false, false},
	{1, false, false}
};
static volatile bool renew_mapping = true;
static bool sensor_loop = false;
static _thread_t sensor_thread;
static k_tid_t sensor_tid;
static bool sensor_value[2] = {false};


/**
 * @brief Renew sensor to valve mapping
 */
void renew_sensor_mapping(void) {
	renew_mapping = true;
}

/**
 * @brief Update sensor to valve mapping
 */
void sensor_mapping_update(void) {
	for (int i=0;i<2;i++) {
		uint8_t mapping = OD_sensorMapping[i];
		if (mapping >= 8) {
			LOG_ERR("Sensor mapping value not valid");
			continue;
		}
		sensor_valve_state[i].idx = mapping >> 2;
		sensor_valve_state[i].state = ((mapping >> 1) & 0x01) ? true : false;
		sensor_valve_state[i].value = (mapping & 0x01) ? true : false;
	}
}
/**
 * @brief Initialize sensor GPIOs
 */
int sensor_init(void)
{
    int err;
    
    if (!gpio_is_ready_dt(&sensor_sw)) {
        LOG_ERR("The sensor switch pin GPIO port is not ready.\n");
        return -1;
    }
    err = gpio_pin_configure_dt(&sensor_sw, GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        LOG_ERR("Configuring sensor switch GPIO pin failed: %d\n", err);
        return -1;
    }
    for (int i=0;i<2;i++) {
        if (!gpio_is_ready_dt(sensors[i])) {
            LOG_ERR("The sensor%d pin GPIO port is not ready.\n", i+1);
            return -1;
        }
        err = gpio_pin_configure_dt(sensors[i], GPIO_INPUT);
        if (err != 0) {
            LOG_ERR("Configuring sensor%d GPIO pin failed: %d\n", i+1, err);
            return -1;
        }
    }
	sensor_mapping_update();
    return 0;
}

/**
 * @brief Watchdog callback function
 *
 * @param channel_id Watchdog channel ID
 * @param user_data Pointer to user data (thread ID)
 */
static void sensor_watchdog_callback(int channel_id, void *user_data)
{
    LOG_ERR("Watchdog channel %d callback, user data: %p",
        channel_id, k_thread_name_get((k_tid_t)user_data));
	sys_reboot(SYS_REBOOT_COLD);
}

/**
 * @brief Enable or disable sensor thread
 *
 * @param value true to enable, false to disable
 * @return 0 on success, -1 on error, 1 if already started
 */
int sensor_enable(bool value)
{
	int err;
	if (!gpio_is_ready_dt(&sensor_sw)) {
		LOG_ERR("The sensor switch pin GPIO port is not ready.\n");
		return -1;
	}
    if (value) {
		err = gpio_pin_set_dt(&sensor_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring sensor switch GPIO pin failed: %d\n", err);
    		return -1;
		}
		if (OD_sensorEnable == 1) {
			LOG_DBG("Thread already started");
			return 1;
		}
		sensor_loop = true;
		sensor_tid = k_thread_create(&sensor_thread, sensor_stack_area,
                                 K_THREAD_STACK_SIZEOF(sensor_stack_area),
                                 sensor_func,
                                 NULL, NULL, NULL,
                                 K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	} else {
		sensor_loop = false;
		err = gpio_pin_set_dt(&sensor_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting sensor switch GPIO pin level failed: %d\n", err);
    		return -1;
		}
    }
    return 0;
}

/**
 * @brief Sensor thread function
 *
 * @param d0 Unused
 * @param d1 Unused
 * @param d2 Unused
 */
void sensor_func(void *d0, void *d1, void *d2) {
	LOG_DBG("start thread");
	// bool set_update = false;
	int wdg_id = watchdog_add((uint32_t)(2*OD_sensorPeriode), sensor_watchdog_callback,(void *)k_current_get());
	CO_LOCK_OD();
	OD_sensorEnable = 1;
	CO_UNLOCK_OD();
	LOG_DBG("start loop");
    while(sensor_loop) {
		watchdog_feed(wdg_id);
		for (int i=0;i<2;i++) {
			if (sensors[i]==NULL) {
				continue;
			}
			if (gpio_pin_get_dt(sensors[i]) != 0) {
				sensor_value[i] = true;
			}else {
				sensor_value[i] = false;
			}

			if ((BOOLEAN)(OD_sensorMeasurement[i]) != (BOOLEAN)sensor_value[i]) {
				CO_LOCK_OD();
				OD_sensorMeasurement[i] = (BOOLEAN)(sensor_value[i]);
				CO_UNLOCK_OD();
				if (sensor_valve_state[i].state == (BOOLEAN)(OD_sensorMeasurement[i])) {
					valve_set_state(sensor_valve_state[i].idx, sensor_valve_state[i].value);
				}
			}
		}
        k_sleep(K_MSEC(OD_sensorPeriode));
    }
	CO_LOCK_OD();
	OD_sensorEnable = 0;
	CO_UNLOCK_OD();
	watchdog_delete(wdg_id);
}

/**
 *  @brief process sensor mapping update
 */
int sensor_process(void) {
	if (renew_mapping) {
		sensor_mapping_update();
		renew_mapping = false;
	}
	return 0;
}