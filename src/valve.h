#ifndef _VALVE_H
#define _VALVE_H

int valve_set_state(uint8_t idx, uint8_t state);
int valve_order(uint8_t idx, bool value);
void valve_func(void *d0, void *d1, void *d2);
int valve_start_thread(void);
int valve_process(void);

#endif