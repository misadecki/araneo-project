#include "oled.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

LOG_MODULE_REGISTER(oled, 3);

void setup_display() {
  if (!device_is_ready(display)) {
    LOG_ERR("Display not ready\n");
    return;
  }

  if (cfb_framebuffer_init(display)) {
     LOG_ERR("CFB init failed\n");
    return;
  }

  cfb_framebuffer_clear(display, true);
  cfb_framebuffer_finalize(display);
}

void draw_smile() {
  return;
}
