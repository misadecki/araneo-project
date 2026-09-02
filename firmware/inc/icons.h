#ifndef ICONS_H__
#define ICONS_H__

#include <stdint.h>

typedef enum {
  ICON_STANDARD = 0,
  ICON_SMILE,
  ICON_ANGRY,
  ICON_SAD,
  ICON_EXCITED,
  ICON_SLEEP,
  ICON_KIDDING,
  ICON_ELIZA,
  ICON_SHOCKED,
  ICON_BATTERY_FULL,
  ICON_BATTERY_EMPTY,
  ICON_BATTERY_1,
  ICON_BATTERY_2,
  ICON_BATTERY_3,
  ICON_BATTERY_4,

  ICON_COUNT
} icon_id_t;

const uint8_t *get_icon(icon_id_t id);

#endif // ICONS_H__
