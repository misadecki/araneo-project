#ifndef OLED_H__
#define OLED_H__

#include <zephyr/kernel.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

extern const uint8_t smile_face[];
extern const uint8_t sad_face[];
extern const uint8_t shocked_face[];
extern const uint8_t kidding_face[];
extern const uint8_t angry_face[];
extern const uint8_t standard_face[];
extern const uint8_t eliza_face[];
extern const uint8_t excited_face[];
extern const uint8_t sleep_face[];

extern const uint8_t battery_full[];
extern const uint8_t battery_empty[];
extern const uint8_t battery_1[];
extern const uint8_t battery_2[];
extern const uint8_t battery_3[];

void setup_display(void);
void draw_img(const uint8_t *img);

#endif // OLED_H__
