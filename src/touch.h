#ifndef _TOUCH_H
#define _TOUCH_H

#include <zephyr/kernel.h>

#if !DT_NODE_EXISTS(DT_NODELABEL(touch_sw))
#error "Overlay for touch_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(touch1))
#error "Overlay for touch1 output node not properly defined."
#endif

int touch_init(void);
void touch_func(void *d0, void *d1, void *d2);
int touch_enable(bool value);

#endif /* _TOUCH_H */