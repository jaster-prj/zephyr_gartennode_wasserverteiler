#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <zephyr/kernel.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(watchdog0))
#define WDT_NODE DT_ALIAS(watchdog0)
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_window_watchdog)
#define WDT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(st_stm32_window_watchdog)
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_watchdog)
#define WDT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(st_stm32_watchdog)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

typedef void (*watchdog_callback_t)(int channel_id, void *user_data);
int watchdog_add(uint8_t reload_period, watchdog_callback_t callback, void *user_data);
int watchdog_delete(int channel_id);
int watchdog_feed(int channel_id);

#endif /* _WATCHDOG_H */