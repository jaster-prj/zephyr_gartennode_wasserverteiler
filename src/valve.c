#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include "valve.h"

#define VALVE_STACK 1024

LOG_MODULE_REGISTER(valve);

K_THREAD_STACK_DEFINE(valve_stack_area1, VALVE_STACK);
K_THREAD_STACK_DEFINE(valve_stack_area2, VALVE_STACK);

typedef struct {
    uint8_t idx;
    bool value;
}valve_thread_params;

static volatile bool renew_mapping = false;
static bool valve_running[2] = {false};
static valve_thread_params *valve_cmd[2] = {NULL};
static k_tid_t valve_tid[2];
/* 
static k_thread_stack_t *valve_stack_area[2] = {
	valve_stack_area1,
	valve_stack_area2
};
static _thread_t valve_thread[2];
*/
static _thread_t valve_thread1;
static _thread_t valve_thread2;

/**
 * @brief set valve state in object dictionary
 *
 * @param idx valve index (0 or 1)
 * @param state valve state (0=closed, 1=open, 0xFF=unknown)
 * @return 0 on success, -1 on error
 */
int valve_set_state(uint8_t idx, uint8_t state) {
	if (idx >= 2) {
		LOG_ERR("Valve index out of range");
		return -1;
	}
	if (state != 0 && state != 1 && state != 0xFF) {
		LOG_ERR("Valve state not valid");
		return -1;
	}
	CO_LOCK_OD();
	OD_valveState[idx] = state;
	CO_UNLOCK_OD();
	return 0;
}
/**
 * @brief set valve command
 *
 * @param idx valve index (0 or 1)
 * @param value valve state (true for open, false for closed)
 * @return 0 on success, -1 on error
 */
int valve_order(uint8_t idx, bool value) {
    if (idx >= 2) {
        LOG_ERR("Valve index out of range");
        return -1;
    }
    if (valve_cmd[idx] == NULL) {
        valve_cmd[idx] = k_malloc(sizeof(valve_thread_params));
        if (valve_cmd[idx] == NULL) {
            LOG_ERR("could not alloc memory");
            return -1;
        }
    }
    valve_cmd[idx]->idx = idx;
    valve_cmd[idx]->value = value;
    return 0;
}

/**
 * @brief valve thread function
 *
 * @param d0 pointer to valve_thread_params structure
 * @param d1 unused
 * @param d2 unused
 */
void valve_func(void *d0, void *d1, void *d2) {
	if (d0 == NULL) {
		return;
	}
	valve_thread_params params = *(valve_thread_params *)d0;
	k_free(d0);
	if (OD_valveState[params.idx] == (uint8_t)params.value) {
		return;
	}
    uint8_t relais = 0;
	valve_running[params.idx] = true;
	LOG_DBG("start valve_func for %d with %d",params.idx,(uint8_t)params.value);
    if (OD_valveState[params.idx] == 0xFF) {
        bool old_value = !params.value;
        relais = OD_valveMapping[(1<<params.idx) + (uint8_t)old_value];
        if (relais >= 4) {
            LOG_ERR("Relais index out of range");
            valve_set_state(params.idx, 0xFF);
            valve_running[params.idx] = false;
            return;
        }
        CO_LOCK_OD();
        OD_relaisCmd[relais] = 1;
        CO_UNLOCK_OD();
        k_sleep(K_MSEC(OD_valveRoutineTime));
        CO_LOCK_OD();
        OD_relaisCmd[relais] = 0;
        CO_UNLOCK_OD();
    }
	relais = OD_valveMapping[(1<<params.idx) + (uint8_t)params.value];
    if (relais >= 4) {
        LOG_ERR("Relais index out of range");
        valve_set_state(params.idx, 0xFF);
        valve_running[params.idx] = false;
        return;
    }
	CO_LOCK_OD();
	OD_relaisCmd[relais] = 1;
	CO_UNLOCK_OD();
	k_sleep(K_MSEC(OD_valveRoutineTime));
	CO_LOCK_OD();
	OD_relaisCmd[relais] = 0;
	CO_UNLOCK_OD();
	valve_set_state(params.idx, (uint8_t)params.value);
	valve_running[params.idx] = false;
}

/**
 *  @brief start valve thread if command is pending
 *
 *  @return 0 on success, -1 on error
 */
int valve_start_thread(void) {
	for (int i=0;i<2;i++) {
		if (valve_running[i]) {
			continue;
		}
		if (valve_cmd[i] == NULL) {
			continue;
		}
        valve_thread_params *params = valve_cmd[i];
		if (i == 0) {
			valve_tid[i] = k_thread_create(&valve_thread1, valve_stack_area1,
									K_THREAD_STACK_SIZEOF(valve_stack_area1),
									valve_func,
									(void *)params, NULL, NULL,
									K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
		} else if (i == 1) {
			valve_tid[i] = k_thread_create(&valve_thread2, valve_stack_area2,
									K_THREAD_STACK_SIZEOF(valve_stack_area2),
									valve_func,
									(void *)params, NULL, NULL,
									K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
		}
		valve_cmd[i] = NULL;
	}
    return 0;
}

/**
 * @brief routine to start valve threads if commands are pending
 */
int valve_process(void) {
	if (valve_cmd[0] != NULL || valve_cmd[1] != NULL) {
		valve_start_thread();
	}
	return 0;
}