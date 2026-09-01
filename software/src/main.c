#include "oled.h"
#include "icons.h"
#include "servo.h"
#include <zephyr/kernel.h>

int main(void) {
  if (servo_init(SERVO_COXA_LEFT_BACK) < 0)
    return 1;

  setup_display();
  int i = 0;

  while(1) {
    draw_img(get_icon((ICON_ELIZA + i) % ICON_COUNT));
    ++i;
    k_msleep(2000);
  }
  return 0;
}
