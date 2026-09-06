#include "button.h"
#include "servo.h"
#include <zephyr/kernel.h>

int main(void) {
  if (servo_init(SERVO_COXA_LEFT_BACK) < 0)
    return 1;

  if (button_init() < 0) {
    return 2;
  }

  k_sleep(K_FOREVER);
  return 0;
}
