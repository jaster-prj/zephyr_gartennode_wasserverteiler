#ifndef _RELAIS_H
#define _RELAIS_H

#include <zephyr/kernel.h>

#if !DT_NODE_EXISTS(DT_NODELABEL(relaypw_sw))
#error "Overlay for relaypw_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relay1_sw))
#error "Overlay for relay1_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relay2_sw))
#error "Overlay for relay2_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relay3_sw))
#error "Overlay for relay3_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relay4_sw))
#error "Overlay for relay4_sw output node not properly defined."
#endif

int relais_init(void);
// int relais_set(uint8_t idx, bool value);
int relais_process(void);

#endif /* _RELAIS_H */