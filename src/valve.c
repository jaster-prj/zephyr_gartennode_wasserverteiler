#if defined(CONFIG_BOARD_NATIVE_SIM)
#define _POSIX_C_SOURCE 200809L
#include <time.h>
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include "valve.h"

#define VALVE_STACK 1024

LOG_MODULE_REGISTER(valve, CONFIG_CANOPENNODE_LOG_LEVEL);

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

#if defined(CONFIG_BOARD_NATIVE_SIM)
struct native_valve_motion {
	bool active;
	bool target_open;
	bool calibrating;
	uint8_t relay;
	int64_t deadline_ms;
};

static struct native_valve_motion native_motion[2];

static int64_t native_valve_now_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int native_valve_start_phase(uint8_t idx, bool opening, bool calibrating,
				    int64_t now_ms)
{
	uint8_t relay = OD_valveMapping[(idx * 2U) + (uint8_t)opening];

	if (relay >= 4U) {
		LOG_ERR("Relais index out of range");
		return -1;
	}

	CO_LOCK_OD();
	OD_relaisCmd[relay] = 1;
	CO_UNLOCK_OD();
	native_motion[idx].relay = relay;
	native_motion[idx].calibrating = calibrating;
	native_motion[idx].deadline_ms = now_ms + OD_valveRoutineTime;
	return 0;
}

static void native_valve_process(void)
{
	int64_t now_ms = native_valve_now_ms();

	for (uint8_t idx = 0; idx < 2U; idx++) {
		struct native_valve_motion *motion = &native_motion[idx];

		if (motion->active && now_ms >= motion->deadline_ms) {
			CO_LOCK_OD();
			OD_relaisCmd[motion->relay] = 0;
			CO_UNLOCK_OD();

			if (motion->calibrating) {
				if (native_valve_start_phase(idx, motion->target_open, false,
							     now_ms) != 0) {
					(void)valve_set_state(idx, 0xFF);
					motion->active = false;
				}
				continue;
			}

			(void)valve_set_state(idx, motion->target_open ? 1U : 0U);
			motion->active = false;
		}

		if (motion->active || valve_cmd[idx] == NULL) {
			continue;
		}

		valve_thread_params *params = valve_cmd[idx];

		valve_cmd[idx] = NULL;
		bool target_open = params->value;
		k_free(params);
		if (OD_valveState[idx] == (uint8_t)target_open) {
			continue;
		}

		motion->target_open = target_open;
		motion->active = true;
		bool calibrating = OD_valveState[idx] == 0xFF;
		bool opening = calibrating ? !target_open : target_open;
		if (native_valve_start_phase(idx, opening, calibrating, now_ms) != 0) {
			(void)valve_set_state(idx, 0xFF);
			motion->active = false;
		}
	}
}
#endif

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
 * @brief toggle valve state
 *
 * @param idx valve index (0 or 1)
 * @return 0 on success, -1 on error
 */
int valve_toggle(uint8_t idx) {
	if (idx >= 2) {
		LOG_ERR("Valve index out of range");
		return -1;
	}
	return valve_order(idx, !OD_valveState[idx]);
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
	LOG_DBG("Set valve_cmd for valve %d to %d", idx, value);
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
	LOG_DBG("start valve_func");
	if (d0 == NULL) {
		LOG_DBG("finish valve_func due to NULL parameter");
		return;
	}
	valve_thread_params params = *(valve_thread_params *)d0;
	k_free(d0);
	if (OD_valveState[params.idx] == (uint8_t)params.value) {
		LOG_DBG("finish valve_func due to valve already in desired state");
		return;
	}
    uint8_t relais = 0;
	valve_running[params.idx] = true;
	LOG_DBG("start valve_func for %d with %d",params.idx,(uint8_t)params.value);
    if (OD_valveState[params.idx] == 0xFF) {
        bool old_value = !params.value;
        relais = OD_valveMapping[(params.idx*2) + (uint8_t)old_value];
        if (relais >= 4) {
            LOG_ERR("Relais index out of range");
            valve_set_state(params.idx, 0xFF);
            valve_running[params.idx] = false;
            return;
        }
        CO_LOCK_OD();
        OD_relaisCmd[relais] = 1;
        CO_UNLOCK_OD();
		LOG_DBG("Sleep for %d ms", OD_valveRoutineTime);
        k_sleep(K_MSEC(OD_valveRoutineTime));
        CO_LOCK_OD();
        OD_relaisCmd[relais] = 0;
        CO_UNLOCK_OD();
    }
	relais = OD_valveMapping[(params.idx*2) + (uint8_t)params.value];
    if (relais >= 4) {
        LOG_ERR("Relais index out of range");
        valve_set_state(params.idx, 0xFF);
        valve_running[params.idx] = false;
        return;
    }
	LOG_DBG("Set relais %d for valve %d", relais+1, params.idx);
	CO_LOCK_OD();
	OD_relaisCmd[relais] = 1;
	CO_UNLOCK_OD();
	LOG_DBG("Sleep for %d ms", OD_valveRoutineTime);
	k_sleep(K_MSEC(OD_valveRoutineTime));
	LOG_DBG("Reset relais %d for valve %d", relais+1, params.idx);
	CO_LOCK_OD();
	OD_relaisCmd[relais] = 0;
	CO_UNLOCK_OD();
	valve_set_state(params.idx, (uint8_t)params.value);
	valve_running[params.idx] = false;
	LOG_DBG("finish valve_func");
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
#if defined(CONFIG_BOARD_NATIVE_SIM)
	native_valve_process();
#else
	if (valve_cmd[0] != NULL || valve_cmd[1] != NULL) {
		valve_start_thread();
	}
#endif
	return 0;
}
