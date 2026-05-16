#include <zephyr/drivers/watchdog.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/logging/log.h>
#include "watchdog.h"

LOG_MODULE_REGISTER(watchdog, CONFIG_CANOPEN_LOG_LEVEL);

bool watchdog_initialized = false;

/**
 * @brief Initialize the watchdog
 *
 * @return int 0 on success, -1 on error
 */
int watchdog_init(void) {
    if (watchdog_initialized) {
        return 0;
    }

	int ret;
	const struct device *const hw_wdt_dev = DEVICE_DT_GET_OR_NULL(WDT_NODE);
	if (!device_is_ready(hw_wdt_dev)) {
		LOG_ERR("Hardware watchdog not ready; ignoring it.\n");
		ret = task_wdt_init(NULL);
	} else {
		ret = task_wdt_init(hw_wdt_dev);
	}

	if (ret != 0) {
		LOG_ERR("task wdt init failure: %d\n", ret);
	}
	return ret;
}
/**
 * @brief Add a watchdog channel
 *
 * @param reload_period Reload period in milliseconds
 * @param callback Callback function to be called on timeout
 * @param user_data Pointer to user data to be passed to the callback
 * @return int Channel ID on success, -1 on error
 */
int watchdog_add(uint32_t reload_period, watchdog_callback_t callback, void *user_data)
{
    int channel_id;
    int ret = watchdog_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize watchdog: %d", ret);
        return -1;
    }
    channel_id = task_wdt_add((uint32_t)reload_period, (task_wdt_callback_t)callback, user_data);
    if (channel_id < 0) {
        LOG_ERR("Failed to add watchdog channel: %d", channel_id);
        return -1;
    }
    return channel_id;
}

/**
 * @brief Delete a watchdog channel
 *
 * @param channel_id Channel ID to be deleted
 * @return int 0 on success, -1 on error
 */
int watchdog_delete(int channel_id)
{
    int ret;
    ret = task_wdt_delete(channel_id);
    if (ret != 0) {
        LOG_ERR("Failed to delete watchdog channel %d: %d", channel_id, ret);
        return -1;
    }
    return 0;
}

/**
 * @brief Feed a watchdog channel
 *
 * @param channel_id Channel ID to be fed
 * @return int 0 on success, -1 on error
 */
int watchdog_feed(int channel_id)
{
    int ret;
    ret = task_wdt_feed(channel_id);
    if (ret != 0) {
        LOG_ERR("Failed to feed watchdog channel %d: %d", channel_id, ret);
        return -1;
    }
    return 0;
}