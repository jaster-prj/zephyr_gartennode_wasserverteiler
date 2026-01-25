#ifndef _SENSOR_H
#define _SENSOR_H

#include <zephyr/kernel.h>

#if !DT_NODE_EXISTS(DT_NODELABEL(sensor_sw))
#error "Overlay for sensor_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(sensor1))
#error "Overlay for sensor1 output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(sensor2))
#error "Overlay for sensor2 output node not properly defined."
#endif

void renew_sensor_mapping(void);
void sensor_mapping_update(void);
int sensor_init(void);
void sensor_func(void *d0, void *d1, void *d2);
int sensor_enable(bool value);
int sensor_process(void);

#endif /* _SENSOR_H */