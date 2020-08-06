/**
 Minimal blinkM (i2c led) driver
 */

#define DEBUG_MODULE "BLINKM"

#include "debug.h"
#include "deck.h"
#include "i2cdev.h"
#include "stdio.h"

#include "system.h"
#include "timers.h"
#include "log.h"
#include "param.h"
#include "worker.h"
#include "pm.h"
#include "estimator_kalman.h"

#define BLINKM_DEFAULT_ADDRESS 0x09
#define NUMBER_OF_LEDS 1
#define SEQUENCE_LENGTH 3

// static uint8_t b_color[2][4]= {{'n', 0, 0, 0}, {'n', 0, 0, 0}};
// static uint8_t state_color[4]= {'n', 0, 0, 0};
// static uint8_t oldColor[2][4]= {{'n', 0, 0, 0}, {'n', 0, 0, 0}};
static uint8_t b_addresses[2] = {BLINKM_DEFAULT_ADDRESS, BLINKM_DEFAULT_ADDRESS};
static uint8_t b_stopScript = 'o';
// static uint8_t address = 0x00;
// static uint8_t oldAddress = 0x00;
static uint32_t script = 0x00;
static uint32_t cmd = 0;
static xTimerHandle timer;
static xTimerHandle state_timer;
static bool isInit = false;

static uint8_t fade_speed = 0xFF;
static uint8_t old_fade_speed = 0x00;

static uint8_t batteryLevel = 95;
uint8_t black[4] = {'n', 0, 0, 0};
uint8_t red[4] = {'n', 255, 0, 0};
static uint8_t current_color[4];
static uint8_t shared_period = 100;
static uint8_t shared_counter = 0;


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
  .colors = {{'n', 255, 0, 255}, {'n', 0, 0, 0}, {'n', 0, 0, 0}},
  .periods = {20, 20, 0, 0},
  .counter = 0,
  .index = 0
};
static sequence_t s_sequence = {
  .colors = {{'n', 0, 0, 0}, {'n', 0, 0, 0}, {'n', 0, 0, 0}},
  .periods = {0, 0, 0, 0},
  .counter = 0,
  .index = 0
};
static state_type state = landed_s;
static uint8_t state_steps = 5;


bool read_bytes(uint8_t key, uint8_t length, uint8_t * value)
{
  i2cdevWriteByte(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, key);
  bool result = false;
  while(!result) result = i2cdevRead(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, length, value);
  return true;
}


bool getInfo()
{
  uint8_t address;
  uint8_t version[2] = {0, 0};
  read_bytes('a', 1, &address);
  read_bytes('Z', 2, version);
  char version_str[4];
  sprintf(version_str, "%c,%c", version[0], version[1]);
  DEBUG_PRINT("BlinkM with firmware %s at I2C 0x%02x \n", version_str, address);
  return true;
}

void getColor()
{
  uint8_t color[3];
  read_bytes('g', 3, color);
  DEBUG_PRINT("BlinkM read color => (%d, %d, %d) \n", color[0], color[1], color[2]);
}

bool setFadeSpeed(uint8_t speed, uint8_t index)
{
  if(speed == old_fade_speed)
  {
    return true;
  }
  old_fade_speed = speed;
  uint8_t data[2] = {'f', speed};
  bool result = i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 2, data);
  DEBUG_PRINT("BlinkM set fade speed %d => %d \n", data[1], result);
  return result;
}

void setColor(uint8_t *color, uint8_t index)
{
  if(memcmp(color, current_color, 4) !=0)
  {
    // i2cdevWriteByte(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 'n');
    i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 4, color);
    // DEBUG_PRINT("BlinkM set color to (%d, %d, %d) \n", color[0], color[1], color[2]);
    memcpy(current_color, color, 4);
  }
}


void play_sequence(sequence_t * sequence)
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
            sequence->colors[0][1] = 0;
            sequence->colors[0][2] = 0;
            sequence->colors[0][3] = 0;
          }
        }
      }
    }
    sequence->counter++;
  }
  setColor(sequence->colors[sequence->index], 0);
  // DEBUG_PRINT("sequence:\ncolor1 0X%02x%02x%02x\ncolor2 0X%02x%02x%02x\ncolor3 0X%02x%02x%02x\nperiod 1 %d, period 2 %d, period 3 %d, rep %d\n counter %d, index %d\n",
  // sequence->colors[0][1],
  // sequence->colors[0][2],
  // sequence->colors[0][3],
  // sequence->colors[1][1],
  // sequence->colors[1][2],
  // sequence->colors[1][3],
  // sequence->colors[2][1],
  // sequence->colors[2][2],
  // sequence->colors[2][3],
  // sequence->periods[0], sequence->periods[1], sequence->periods[2], sequence->periods[3],
  // sequence->counter, sequence->index);
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


void blinkMStateWorker(void * data)
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
    rgb[1] = 0;
    rgb[2] = 0;
    rgb[3] = 255;
  }
  else if(state == landed_s)
  {
    rgb[1] = 0;
    rgb[2] = 255;
    rgb[3] = 0;
  }
  else
  {
    rgb[1] = 0;
    rgb[2] = 255;
    rgb[3] = 255;
  }
  if(state != charging_s)
  {
    if(batteryLevel < 10)
    {
      rgb[1] = 255;
      rgb[2] = 0;
      rgb[3] = 0;
      state_level = warn;
    }
    else if(batteryLevel < 15)
    {
      rgb[1] = 255;
      rgb[2] = 30;
      rgb[3] = 0;
      state_level = warn;
    }
    else if(batteryLevel < 20)
    {
      rgb[1] = 255;
      rgb[2] = 100;
      rgb[3] = 0;
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

  // DEBUG_PRINT("Will update State (%d) to (%d, %d, %d) %.3f %d %d %d \n", state_steps, rgb[0], rgb[1], rgb[2], (double) batteryVoltage, is_flying, is_charging, batteryLevel);

}

void setScript(uint32_t script, uint8_t index)
{
  uint8_t *data = (uint8_t *) &script;
  uint8_t data_p[4] = {'p', data[0], data[1], 0};
  uint8_t data_t[2] = {'t', data[2]};
  bool result = i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 2, data_t);
  result = i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 4, data_p);
  DEBUG_PRINT("BlinkM play script %d %d %d => %d \n", data[0], data[1], (int8_t) data[2], result);
}

void setCmd(uint32_t cmd)
{
  uint8_t *data = (uint8_t *) &cmd;
  uint8_t length = 0xff;
  switch (data[0]) {
    case 'n':
    case 'c':
    case 'h':
    case 'C':
    case 'H':
    case 'p':
      length = 4;
      break;
    case 'f':
    case 't':
      length = 2;
      break;
    case 'o':
      length = 1;
      break;
  }
  if (length == 0xff) return;
  i2cdevWrite(I2C1_DEV, b_addresses[0], I2CDEV_NO_MEM_ADDR, length, data);
}



// void setAddress(uint8_t address)
// {
//   if(address != oldAddress && address > 0 && address <= 0xF0)
//   {
//     uint8_t data[5] = {'A', address, 0xd0, 0x0d, address};
//     bool result = i2cdevWrite(I2C1_DEV, 0x00, I2CDEV_NO_MEM_ADDR, 5, data);
//     DEBUG_PRINT("BlinkM changed address to %d => %d \n", address, result);
//     oldAddress = address;
//   }
// }

void stopScript()
{
  i2cdevWriteByte(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, b_stopScript);
}


void blinkMWorker(void * data)
{
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

  if(script)
  {
    setScript(script, 0);
    script = 0;
  }
  else if (cmd)
  {
    setCmd(cmd);
    cmd = 0;
  }
  else
  {
    play_sequence(&e_sequence);
    // DEBUG_PRINT("counter %d, l_index %d, sequences %d, period %d \n", counter,l_index, sequences, periods[l_index]);
  }
  setFadeSpeed(fade_speed, 0);

  // setColor(b_color[1], 1);
  // setAddress(address);

}

static void blinkMTimer(xTimerHandle timer)
{
  workerSchedule(blinkMWorker, NULL);
}

static void blinkMStateTimer(xTimerHandle timer)
{
  workerSchedule(blinkMStateWorker, NULL);
}

void blinkMInit(DeckInfo* info)
{

  if (isInit) {
    return;
  }
  bool res = i2cdevInit(I2C1_DEV);
  DEBUG_PRINT("i2cdevInit => %d \n", res);

   isInit = setFadeSpeed(255, 0);
   // stopScript();
   // getColor(oldColor);
   if(isInit)
   {
    getInfo();
    //getColor();
    timer = xTimerCreate( "blinkMTimer", M2T(50), pdTRUE, NULL, blinkMTimer );
    state_timer = xTimerCreate( "blinkMStateTimer", M2T(1000), pdTRUE, NULL, blinkMStateTimer );
    xTimerStart(timer, 100);
    xTimerStart(state_timer, 100);
    DEBUG_PRINT("BlinkM deck initialized! \n");
  }
  else
  {
    DEBUG_PRINT("BlinkM deck NOT initialized! \n");
  }
}

// bool blinkMTest(void)
// {
//   DEBUG_PRINT("BlinkM test passed!\n");
//   return true;
// }


static const DeckDriver blinkM_deck = {
  .vid = 0,
  .pid = 0,
  .usedGpio = DECK_USING_SDA | DECK_USING_SCL,
  .name = "blinkM",
  .init = blinkMInit,
  // .test = blinkMTest,
};

DECK_DRIVER(blinkM_deck);

PARAM_GROUP_START(blinkM)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, blinkM, &isInit)
PARAM_ADD(PARAM_UINT8, mode, &mode)
PARAM_ADD(PARAM_UINT8, shared_mask, &shared_level_mask)
PARAM_ADD(PARAM_UINT8, fadeSpeed, &fade_speed)
PARAM_ADD(PARAM_UINT32, color1, &(e_sequence.colors[0]))
PARAM_ADD(PARAM_UINT32, color2, &(e_sequence.colors[1]))
PARAM_ADD(PARAM_UINT32, color3, &(e_sequence.colors[2]))
PARAM_ADD(PARAM_UINT32, sequence, &(e_sequence.periods))
// PARAM_ADD(PARAM_UINT8, solidRed2, &(b_color[1][1]))
// PARAM_ADD(PARAM_UINT8, solidGreen2, &(b_color[1][2]))
// PARAM_ADD(PARAM_UINT8, solidBlue2, &(b_color[1][3]))

PARAM_ADD(PARAM_UINT32, script, &script)
PARAM_ADD(PARAM_UINT32, cmd, &cmd)
// PARAM_ADD(PARAM_UINT8, address, &address)
PARAM_GROUP_STOP(blinkM)
