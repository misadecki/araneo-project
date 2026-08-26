#include "oled.h"
#include <zephyr/kernel.h>

int main(void) {

  setup_display();
  draw_img(smile_face);

  while(1) {
    k_sleep(K_MSEC(1000));
  }
  return 0;
}
