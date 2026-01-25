#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "led.h"

LOG_MODULE_REGISTER(led);

static void led_callback(bool value, void *arg);

static struct gpio_dt_spec led_green_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(green_led), gpios, {0});
static struct gpio_dt_spec led_red_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(red_led), gpios, {0});

/**
 * @brief Initialize LED indicators pins.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @return 0 on success, negative error code on failure.
 */
int led_init(void)
{
    int err;

    if (!led_green_gpio.port) {
        LOG_DBG("Green LED not available");
    } else if (!gpio_is_ready_dt(&led_green_gpio)) {
        LOG_ERR("Green LED device not ready");
        led_green_gpio.port = NULL;
    } else {
        err = gpio_pin_configure_dt(&led_green_gpio,
                        GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("failed to configure Green LED gpio: %d", err);
            led_green_gpio.port = NULL;
        }
    }

    if (!led_red_gpio.port) {
        LOG_DBG("Red LED not available");
    } else if (!gpio_is_ready_dt(&led_red_gpio)) {
        LOG_ERR("Red LED device not ready");
        led_red_gpio.port = NULL;
    } else {
        err = gpio_pin_configure_dt(&led_red_gpio,
                        GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("failed to configure Red LED gpio: %d", err);
            led_red_gpio.port = NULL;
        }
    }

    return 0;
}

/**
 * @brief Callback for setting LED indicator state.
 *
 * @param value true if the LED indicator shall be turned on, false otherwise.
 * @param arg argument that was passed when LEDs were initialized.
 */
static void led_callback(bool value, void *arg)
{
	struct gpio_dt_spec *led_gpio = arg;

	if (!led_gpio || !led_gpio->port) {
		return;
	}

	gpio_pin_set_dt(led_gpio, value);
}

void led_configure(CO_NMT_t *nmt) {
    canopen_leds_init(nmt,
              led_callback, &led_green_gpio,
              led_callback, &led_red_gpio);
}