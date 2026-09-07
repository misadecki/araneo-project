#include "buck.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(buck, LOG_LEVEL_INF);
static const struct gpio_dt_spec buck_en = GPIO_DT_SPEC_GET(DT_ALIAS(buck), gpios);

int buck_init(void) {
  if (!gpio_is_ready_dt(&buck_en)) {
    LOG_ERR("Error: Buck isn't ready.");
    return -ENODEV;
  }

  int ret;

  ret = gpio_pin_configure_dt(&buck_en, GPIO_OUTPUT_INACTIVE);

  if (ret < 0) {
    LOG_ERR("Error configuring buck GPIO: %d.", ret);
    return ret;
  }

  return 0;
}

void buck_on(void) {
  gpio_pin_set_dt(&buck_en, 1);
}

void buck_off(void) {
  gpio_pin_set_dt(&buck_en, 0);
}
