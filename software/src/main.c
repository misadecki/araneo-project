#include "oled.h"
#include "icons.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

static const struct gpio_dt_spec debug_led = GPIO_DT_SPEC_GET(DT_ALIAS(debugled), gpios);
static const struct pwm_dt_spec servo_lf = PWM_DT_SPEC_GET(DT_ALIAS(servolf));

void set_servo_angle(uint32_t angle_deg) {
  uint32_t pulse_us = 1000 + (angle_deg * (2000 - 1000) / 180);
  pwm_set_pulse_dt(&servo_lf, PWM_USEC(pulse_us));
}

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&debug_led)) {
    return 1;
  }

  if (!pwm_is_ready_dt(&servo_lf)) {
    return 1;
  }

  ret = gpio_pin_configure_dt(&debug_led, GPIO_OUTPUT_ACTIVE);

  if (ret < 0) {
    printk("Debug LED init failed.\n");
    return 1;
  }

  setup_display();
  int i = 0;

  while(1) {
    draw_img(get_icon((ICON_ELIZA + i) % ICON_COUNT));
    gpio_pin_toggle_dt(&debug_led);
    ++i;
  set_servo_angle(170);
    k_msleep(2000);
  }
  return 0;
}
