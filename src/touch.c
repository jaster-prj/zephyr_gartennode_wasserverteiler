#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include "touch.h"
#include "watchdog.h"
#include "valve.h"

#define TOUCH_STACK 1024

LOG_MODULE_REGISTER(touch, CONFIG_CANOPEN_LOG_LEVEL);
K_THREAD_STACK_DEFINE(touch_stack_area, TOUCH_STACK);

static void touch_watchdog_callback(int channel_id, void *user_data);

static struct gpio_dt_spec touch_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(touch_sw), gpios, {0});
static struct gpio_dt_spec touch1 =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(touch1), gpios, {0});

static const uint32_t touch_periode = 500;
static bool touch_loop = false;
static _thread_t touch_thread;
static k_tid_t touch_tid;
static bool touch_value = false;
static bool *touch_value_last = NULL;

/**
 * @brief Initialize touch GPIOs
 */
int touch_init(void)
{
    int err;
    
    if (!gpio_is_ready_dt(&touch_sw)) {
        LOG_ERR("The touch switch pin GPIO port is not ready.\n");
        return -1;
    }
    err = gpio_pin_configure_dt(&touch_sw, GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        LOG_ERR("Configuring touch switch GPIO pin failed: %d\n", err);
        return -1;
    }

	if (!gpio_is_ready_dt(&touch1)) {
		LOG_ERR("The touch1 pin GPIO port is not ready.\n");
		return -1;
	}
	err = gpio_pin_configure_dt(&touch1, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("Configuring touch1 GPIO pin failed: %d\n", err);
		return -1;
	}
	return 0;
}

/**
 * @brief Watchdog callback function
 *
 * @param channel_id Watchdog channel ID
 * @param user_data Pointer to user data (thread ID)
 */
static void touch_watchdog_callback(int channel_id, void *user_data)
{
    LOG_ERR("Watchdog channel %d callback, user data: %p",
        channel_id, k_thread_name_get((k_tid_t)user_data));
	sys_reboot(SYS_REBOOT_COLD);
}

/**
 * @brief Enable or disable touch thread
 *
 * @param value true to enable, false to disable
 * @return 0 on success, -1 on error, 1 if already started
 */
int touch_enable(bool value)
{
	int err;
	if (!gpio_is_ready_dt(&touch_sw)) {
		LOG_ERR("The touch switch pin GPIO port is not ready.\n");
		return 10;
	}
	LOG_DBG("touch_enabled called with value: %d", value);
	if (value) {
		err = gpio_pin_set_dt(&touch_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring touch switch GPIO pin failed: %d\n", err);
			return 11;
		}
		if (OD_touchEnable == 1) {
			LOG_DBG("Thread already started");
			return 1;
		}
		LOG_DBG("Start touch thread");
		touch_loop = true;
		touch_tid = k_thread_create(&touch_thread, touch_stack_area,
								K_THREAD_STACK_SIZEOF(touch_stack_area),
								touch_func,
								NULL, NULL, NULL,
								K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	} else {
		touch_loop = false;
		err = gpio_pin_set_dt(&touch_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting touch switch GPIO pin level failed: %d\n", err);
			return 11;
		}
	}
	return 0;
}

/**
 * @brief Touch thread function
 *
 * @param d0 Unused
 * @param d1 Unused
 * @param d2 Unused
 */
void touch_func(void *d0, void *d1, void *d2) {
	LOG_DBG("start thread");
	// bool set_update = false;
	// int wdg_id = watchdog_add((uint32_t)(2*touch_periode), touch_watchdog_callback,(void *)k_current_get());
	CO_LOCK_OD();
	OD_touchEnable = 1;
	CO_UNLOCK_OD();
	LOG_DBG("start loop");
	bool touch_value_raw = false;
	while(touch_loop) {
		// if (wdg_id >= 0) {
		// 	watchdog_feed(wdg_id);
		// }
		touch_value_raw = gpio_pin_get_dt(&touch1);
		if (touch_value_raw != 0) {
			touch_value = true;
		}else {
			touch_value = false;
		}

		if (touch_value_last == NULL) {
			touch_value_last = k_malloc(sizeof(bool));
			*touch_value_last = touch_value;
		} 
		if (*touch_value_last != touch_value && touch_value == true) {
			valve_toggle(OD_touchValveIndex);
		}
		*touch_value_last = touch_value;
		k_sleep(K_MSEC(touch_periode));
	}
	// if (wdg_id >= 0) {
	// 	watchdog_delete(wdg_id);
	// }
	CO_LOCK_OD();
	OD_touchEnable = 0;
	CO_UNLOCK_OD();
	k_free(touch_value_last);
}