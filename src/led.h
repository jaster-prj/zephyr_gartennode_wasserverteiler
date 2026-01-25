#ifndef _LED_H
#define _LED_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>

int led_init(void);
void led_configure(CO_NMT_t *nmt);

#endif /* _LED_H */