#include "display_manager.h"
#include "oled.h"
#include "icons.h"
#include <zephyr/kernel.h>

static K_SEM_DEFINE(display_update_sem, 0, 1);

void display_trigger_change(void) {
  k_sem_give(&display_update_sem);
}

static void display_thread_entry(void *p1, void *p2, void *p3) {
  setup_display();
  draw_img(get_icon(ICON_SMILE));
  uint8_t i = 0;

  while (1) {
    k_sem_take(&display_update_sem, K_FOREVER);
    draw_img(get_icon(i % (ANIM_FACE_COUNT)));
    ++i;
  }
}

K_THREAD_DEFINE(display_thread, 1024, display_thread_entry, NULL, NULL, NULL, 7,
                0, 0);
