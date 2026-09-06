#include "button.h"
#include "servo.h"
#include "led.h"
#include <zephyr/kernel.h>

int main(void) {
  if (servo_init(SERVO_COXA_LEFT_BACK) < 0)
    return 1;

  if (button_init() < 0) {
    return 2;
  }

  if (init_led() < 0) {
    return 3;
  }

  while (1) {
    toggle_led();
    k_msleep(2000);
  }
  return 0;
}
