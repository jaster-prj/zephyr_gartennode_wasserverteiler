#ifndef WASSERVERTEILER_UPDATE_DIAGNOSTICS_H
#define WASSERVERTEILER_UPDATE_DIAGNOSTICS_H

#include <canopennode.h>

#if defined(CONFIG_WASSERVERTEILER_UPDATE_DIAGNOSTICS)
void wasserverteiler_update_diagnostics_attach(CO_SDO_t *sdo);
#endif

#endif
