#include "button.h"
#include "display_manager.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0),
                                                           gpios);
static struct gpio_callback button_cb_data;

static void button_pressed_isr(const struct device *dev, struct gpio_callback
                               *cb, uint32_t pins) {
  static int64_t last_press_time = 0;
  int64_t now = k_uptime_get();

  if (now - last_press_time > 200) {
    last_press_time = now;
    display_trigger_change();
  }
}

int button_init(void) {
  int ret;

  if (!gpio_is_ready_dt(&button)) {
    LOG_ERR("GPIO for button isn't ready.");
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&button, GPIO_INPUT);

  if (ret < 0) {
    LOG_ERR("Error configuring button GPIO: %d.", ret);
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);

  if (ret < 0) {
    LOG_ERR("Error configuring interrupt for button: %d", ret);
    return ret;
  }

  gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
  ret = gpio_add_callback(button.port, &button_cb_data);

  if (ret < 0) {
    LOG_ERR("Error adding GPIO callback for button %d.", ret);
    return ret;
  }

  return 0;
}
