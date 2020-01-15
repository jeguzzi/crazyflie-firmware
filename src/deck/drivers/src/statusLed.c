/**
  Generic status led
 */

#include "statusLed.h"

#include "debug.h"
#include "timers.h"
#include "log.h"
#include "param.h"
#include "worker.h"
#include "pm.h"
#include "estimator_kalman.h"

#define SEQUENCE_LENGTH 3

static uint8_t batteryLevel = 95;
static uint8_t shared_period = 100;
static uint8_t shared_counter = 0;

static statusLed_t *leds[2];
static uint8_t statusLedNumber = 0;

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

typedef enum
{
  state_s,
  extern_s,
  shared_s
} mode_type;

typedef enum
{
  warn,
  info
} state_level_type;

static state_level_type state_level = info;
static uint8_t shared_level_mask = (1 << warn) | (1 << info);
static mode_type mode = extern_s;

typedef enum state_e
{
  flying_s,
  charging_s,
  landed_s,
} state_type;

typedef struct
{
  uint8_t colors[SEQUENCE_LENGTH][4];
  uint8_t periods[4];
  uint8_t counter;
  uint8_t index;
} sequence_t;

static sequence_t e_sequence = {
  .colors = {{255, 0, 255, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
  .periods = {20, 20, 0, 0},
  .counter = 0,
  .index = 0
};
static sequence_t s_sequence = {
  .colors = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
  .periods = {0, 0, 0, 0},
  .counter = 0,
  .index = 0
};

static state_type state = landed_s;
static uint8_t state_steps = 5;

static void play_sequence(sequence_t * sequence)
{
  if(sequence->periods[sequence->index] > 0 || sequence->index > 0)
  {
    if(sequence->periods[sequence->index] <= sequence->counter)
    {
      sequence->index++;
      sequence->counter = 0;
      if(sequence->index >= SEQUENCE_LENGTH || sequence->periods[sequence->index] == 0)
      {
        sequence->index = 0;
        if(sequence->periods[3] > 0){
          sequence->periods[3]--;
          if(!sequence->periods[3])
          {
            sequence->periods[0] = 0;
            sequence->colors[0][0] = 0;
            sequence->colors[0][1] = 0;
            sequence->colors[0][2] = 0;
          }
        }
      }
    }
    sequence->counter++;
  }

  uint8_t *rgb = sequence->colors[sequence->index];
  for (size_t i = 0; i < statusLedNumber; i++) {
    if(leds[i]->active)
    {
      uint8_t color[3];
      uint16_t brightness = (uint16_t) (leds[i]->brightness);
      for (size_t j = 0; j < 3; j++) {
        color[j] = (brightness * rgb[j]) / 100;
      }
      leds[i]->setColor(color, 0);
    }
  }
}

const static float batCharging[10] =
{
  3.00, // 00%
  3.78, // 10%
  3.83, // 20%
  3.87, // 30%
  3.89, // 40%
  3.92, // 50%
  3.96, // 60%
  4.00, // 70%
  4.04, // 80%
  4.10  // 90%
};

const static float batFlying[20] =
{
  3.197, // 5%
  3.273, // 10%
  3.318,
  3.353,
  3.382,
  3.407,
  3.430,
  3.451,
  3.472,
  3.492,
  3.512,
  3.532,
  3.554,
  3.576,
  3.602,
  3.630,
  3.665,
  3.712,
  3.786, // 95%
  3.96
};

const static float batLanded[20] =
{
  3.518, // 5%
  3.592,
  3.639,
  3.673,
  3.702,
  3.727,
  3.750,
  3.771,
  3.792,
  3.812,
  3.832,
  3.852,
  3.874,
  3.897,
  3.922,
  3.951,
  3.986,
  4.031,
  4.106, // 95%
  4.280
};

static uint8_t pmBatteryChargeFromVoltage(float voltage, const float *bats, uint8_t length, uint8_t delta)
{
  uint8_t charge = 0;
  if (voltage < bats[0])
  {
    return 0;
  }
  if (voltage > bats[length-1])
  {
    // DEBUG_PRINT("FFFF %d | %.2f > %.2f", delta * (length-1), (double) voltage,(double)bats[length-1]);
    return delta * (length-1);
  }
  int i = 0;
  while (voltage >  bats[i])
  {
    charge += delta;
    // DEBUG_PRINT("%d | %.2f > %.2f", charge, (double) voltage,(double)bats[i]);
    i++;
  }
  return charge;
}


static void stateWorker(void * data)
{
  uint8_t *rgb = s_sequence.colors[0];
  float batteryVoltage = pmGetBatteryVoltage();
  bool is_flying = estimatorKalmanIsFlying();
  bool is_charging = !pmIsDischarging();
  state_level = info;

  if(is_flying)
  {
    if(state!=flying_s)
    {
      state_steps = 0;
      state = flying_s;
    }
    else if(state_steps < 5)
    {
      state_steps++;
    }
  }
  else if(is_charging)
  {
    state = charging_s;
  }
  else
  {
    if(state != landed_s)
    {
      state_steps = 0;
      state = landed_s;
    }
    else if(state_steps < 5)
    {
      state_steps++;
    }
  }

  if(state != charging_s && state_steps == 5)
  {
    switch (state) {
      case flying_s:
          batteryLevel = MIN(batteryLevel, pmBatteryChargeFromVoltage(batteryVoltage, batFlying, 20, 5));
          break;
      case landed_s:
          batteryLevel = MIN(batteryLevel, pmBatteryChargeFromVoltage(batteryVoltage, batLanded, 20, 5));
          break;
      default:
        break;
    }
  }

  if(state == charging_s)
  {
    batteryLevel = pmBatteryChargeFromVoltage(batteryVoltage, batCharging, 10, 10);
  }
  if(state == flying_s)
  {
    rgb[0] = 0;
    rgb[1] = 0;
    rgb[2] = 255;
  }
  else if(state == landed_s)
  {
    rgb[0] = 0;
    rgb[1] = 255;
    rgb[2] = 0;
  }
  else
  {
    rgb[0] = 0;
    rgb[1] = 255;
    rgb[2] = 255;
  }
  if(state != charging_s)
  {
    if(batteryLevel < 10)
    {
      rgb[0] = 255;
      rgb[1] = 0;
      rgb[2] = 0;
      state_level = warn;
    }
    else if(batteryLevel < 15)
    {
      rgb[0] = 255;
      rgb[1] = 30;
      rgb[2] = 0;
      state_level = warn;
    }
    else if(batteryLevel < 20)
    {
      rgb[0] = 255;
      rgb[1] = 100;
      rgb[2] = 0;
      state_level = warn;
    }
}

  if(state != charging_s && batteryLevel < 10)
  {
    s_sequence.periods[0] = 2;
    s_sequence.periods[1] = 2;
    s_sequence.periods[3] = 0;
  }
  else
  {
    s_sequence.periods[0] = batteryLevel / 5;
    s_sequence.periods[1] = 20 - s_sequence.periods[0];
    s_sequence.periods[3] = 1;
  }
}


static void ledWorker(void * data)
{

  for (size_t i = 0; i < statusLedNumber; i++) {
    leds[i]->active = !leds[i]->update() && leds[i]->active;
  }

  if(mode == shared_s && (shared_level_mask & (1 << state_level)))
  {
    if(shared_counter >= shared_period)
    {
      shared_counter = 0;
      s_sequence.counter = 0;
      s_sequence.index = 0;
    }
    if(shared_counter < 20) play_sequence(&s_sequence);
    if(shared_counter == 20)
    {
      // e_sequence.counter = 0;
      // e_sequence.index = 0;
    }
    if(shared_counter >= 20) play_sequence(&e_sequence);
    shared_counter++;
    return;
  }
  if(mode == state_s)
  {
    play_sequence(&s_sequence);
    return;
  }
  play_sequence(&e_sequence);
}

static void ledTimer(xTimerHandle timer)
{
  workerSchedule(ledWorker, NULL);
}

static void stateTimer(xTimerHandle timer)
{
  workerSchedule(stateWorker, NULL);
}

void initStatusLed(statusLed_t *myLed)
{
  leds[statusLedNumber++] = myLed;
  if(statusLedNumber == 1)
  {
    xTimerHandle _timer = xTimerCreate( "ledTimer", M2T(50), pdTRUE, NULL, ledTimer );
    xTimerHandle _stateTimer = xTimerCreate( "stateTimer", M2T(1000), pdTRUE, NULL, stateTimer );
    xTimerStart(_timer, 100);
    xTimerStart(_stateTimer, 100);
    DEBUG_PRINT("led initialized! \n");
  }
}

PARAM_GROUP_START(led)
PARAM_ADD(PARAM_UINT8, mode, &mode)
PARAM_ADD(PARAM_UINT8, shared_mask, &shared_level_mask)
PARAM_ADD(PARAM_UINT32, color1, &(e_sequence.colors[0]))
PARAM_ADD(PARAM_UINT32, color2, &(e_sequence.colors[1]))
PARAM_ADD(PARAM_UINT32, color3, &(e_sequence.colors[2]))
PARAM_ADD(PARAM_UINT32, sequence, &(e_sequence.periods))
PARAM_GROUP_STOP(led)
