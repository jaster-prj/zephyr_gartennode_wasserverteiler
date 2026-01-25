#ifndef _RELAIS_H
#define _RELAIS_H

#include <zephyr/kernel.h>

#if !DT_NODE_EXISTS(DT_NODELABEL(relais1_sw))
#error "Overlay for relais1_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relais2_sw))
#error "Overlay for relais2_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relais3_sw))
#error "Overlay for relais3_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(relais4_sw))
#error "Overlay for relais4_sw output node not properly defined."
#endif

int relais_init(void);
// int relais_set(uint8_t idx, bool value);
int relais_process(void);

#endif /* _RELAIS_H */