#include "oled.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

LOG_MODULE_REGISTER(oled, 3);

void setup_display() {
  if (!device_is_ready(display)) {
    LOG_ERR("Display not ready");
    return;
  }

  if (cfb_framebuffer_init(display)) {
     LOG_ERR("CFB init failed");
    return;
  }

  cfb_framebuffer_clear(display, true);
}

void draw_img(const uint8_t *img) {
  cfb_framebuffer_clear(display, true);

  int bytes_per_row = OLED_WIDTH / 8;

  for (int r = 0; r < OLED_HEIGHT; ++r) {
    for (int c = 0; c < OLED_WIDTH; ++c) {
      uint8_t byte = img[r * bytes_per_row + (c / 8)];
      bool pixel_on = (byte & (1 << (c % 8))) != 0;

      if (!pixel_on) {
        struct cfb_position pos = {.x = c, .y = r};
        cfb_draw_point(display, &pos);
      }
    }
  }

  cfb_framebuffer_finalize(display);
}
