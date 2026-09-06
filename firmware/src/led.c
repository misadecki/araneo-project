#include "led.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(debugled), gpios);

int init_led(void) {
  if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("LED GPIO not ready.");
    return -ENODEV;
  }

  int ret;

  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

  if (ret < 0) {
    LOG_ERR("Error configuring LED GPIO: %d.", ret);
    return ret;
  }

  return 0;
}

void set_led_on(void) {
  gpio_pin_set_dt(&led, 1);
}

void set_led_off(void) {
  gpio_pin_set_dt(&led, 0);
}

void toggle_led(void) {
  gpio_pin_toggle_dt(&led);
}
