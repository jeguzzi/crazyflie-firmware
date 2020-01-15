#ifndef __STATUS_LED_H__
#define __STATUS_LED_H__

#include "FreeRTOS.h"

typedef struct
{
  void (* setColor)(uint8_t *rgb, uint8_t index);
  bool (* update)();
  bool active;
  uint8_t brightness;
} statusLed_t;

void initStatusLed(statusLed_t *led);

#endif //__STATUS_LED_H__
