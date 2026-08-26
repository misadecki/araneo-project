#ifndef OLED_H__
#define OLED_H__

#include <zephyr/kernel.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

void setup_display(void);
void draw_img(const uint8_t *img);

#endif // OLED_H__
