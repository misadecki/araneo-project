#include "button.h"
#include "servo.h"
#include "led.h"
#include "buck.h"
#include "wifi.h"
#include "communication.h"
#include <zephyr/kernel.h>

int main(void) {
  // if (servo_init(SERVO_COXA_LEFT_BACK) < 0)
  //   return 1;
  //
  // if (button_init() < 0) {
  //   return 2;
  // }
  //
  // if (led_init() < 0) {
  //   return 3;
  // }
  //
  // if (buck_init() < 0) {
  //   return 4;
  // }

  wifi_init();
  communication_start_thread();
  // buck_on();

  struct robot_command curr_cmd;

  while (1) {
    get_latest_command(&curr_cmd);
    // toggle_led();
    k_msleep(2000);
  }
  return 0;
}
