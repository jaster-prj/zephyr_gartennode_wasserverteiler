#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <canopennode.h>
#include "relais.h"

LOG_MODULE_REGISTER(relais);

static struct gpio_dt_spec relais1_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relais1_sw), gpios, {0});
static struct gpio_dt_spec relais2_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relais2_sw), gpios, {0});
static struct gpio_dt_spec relais3_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relais3_sw), gpios, {0});
static struct gpio_dt_spec relais4_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relais4_sw), gpios, {0});

static struct gpio_dt_spec *relais[4] = {
	&relais1_sw,
	&relais2_sw,
	&relais3_sw,
	&relais4_sw
};

/**
 * @brief Initialize relais GPIOs
 *
 * @return int 0 on success, -1 on error
 */
int relais_init(void) {
    int err;
    for (int i = 0; i < 4; i++) {
        if (!gpio_is_ready_dt(relais[i])) {
            LOG_ERR("The relais%d_sw switch pin GPIO port is not ready.\n", i+1);
            return -1;
        }
        err = gpio_pin_configure_dt(relais[i], GPIO_OUTPUT_INACTIVE);
        if (err != 0) {
            LOG_ERR("Configuring relais%d_sw switch GPIO pin failed: %d\n", i+1, err);
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Set Relais state
 * 
 * @param idx Relais index (0-3)
 * @param value Relais state (0=off, 1=on)
 * @return int 0 on success, -1 on error
 */
// int relais_set(uint8_t idx, bool value) {
//     int err;
//     if (idx >= 4) {
//         LOG_ERR("Relais index out of range");
//         return -1;
//     }
//     err = gpio_pin_set_dt(relais[idx], value);
//     if (err != 0) {
//         LOG_ERR("Setting relais%d switch GPIO pin level failed: %d", idx+1, err);
//         return -1;
//     }
//     CO_LOCK_OD();
//     OD_relaisState[idx] = (BOOLEAN)value;
//     CO_UNLOCK_OD();
//     return 0;
// }

/**
 *  @brief process relais commands
 *
 *  @return 0 on success, -1 on error
 */
int relais_process(void) {
	int err;
	for (uint8_t i=0;i<4;i++) {
		if (OD_relaisState[i] != (BOOLEAN)(OD_relaisCmd[i])) {
			int set_value = 0; 
			if ((int)(OD_relaisCmd[i]) != 0) {
				set_value = 1;
			}
			err = gpio_pin_set_dt(relais[i], set_value);
			if (err != 0) {
				LOG_ERR("Setting relais switch GPIO pin level failed: %d", err);
				return -1;
			}
			CO_LOCK_OD();
			OD_relaisState[i] = (BOOLEAN)set_value;
			CO_UNLOCK_OD();
		}
	}
	return 0;
}