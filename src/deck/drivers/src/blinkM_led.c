/**
 Minimal blinkM (i2c led) driver
 */

#define DEBUG_MODULE "BLINKM"

#include "i2cdev.h"
#include "stdio.h"
#include "debug.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "statusLed.h"
#include "task.h"

#define BLINKM_DEFAULT_ADDRESS 0x09
#define NUMBER_OF_LEDS 1

static uint8_t b_addresses[2] = {BLINKM_DEFAULT_ADDRESS, BLINKM_DEFAULT_ADDRESS};
static uint32_t script = 0x00;
static uint32_t cmd = 0;
static bool isInit = false;
static uint8_t fade_speed = 0xFF;
static uint8_t old_fade_speed = 0x00;
static uint8_t current_color[3];

bool blinkMReadBytes(uint8_t key, uint8_t length, uint8_t * value)
{
  i2cdevWriteByte(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, key);
  bool result = false;
  while(!result) result = i2cdevRead(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, length, value);
  return true;
}

bool blinkMGetInfo()
{
  uint8_t address;
  uint8_t version[2] = {0, 0};
  blinkMReadBytes('a', 1, &address);
  blinkMReadBytes('Z', 2, version);
  char version_str[4];
  sprintf(version_str, "%c,%c", version[0], version[1]);
  DEBUG_PRINT("BlinkM with firmware %s at I2C 0x%02x \n", version_str, address);
  return true;
}

void blinkMGetColor()
{
  uint8_t color[3];
  blinkMReadBytes('g', 3, color);
  DEBUG_PRINT("BlinkM read color => (%d, %d, %d) \n", color[0], color[1], color[2]);
}

bool blinkMSetFadeSpeed(uint8_t speed, uint8_t index)
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

void blinkMSetColor(uint8_t *color, uint8_t index)
{
  if(memcmp(color, current_color, 3) !=0)
  {
    // i2cdevWriteByte(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 'n');
    uint8_t data[4] = {'n', color[0], color[1], color[2]};
    i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 4, data);
    // DEBUG_PRINT("BlinkM set color to (%d, %d, %d) \n", color[0], color[1], color[2]);
    memcpy(current_color, color, 3);
  }
}

void blinkMSetScript(uint32_t script, uint8_t index)
{
  uint8_t *data = (uint8_t *) &script;
  uint8_t data_p[4] = {'p', data[0], data[1], 0};
  uint8_t data_t[2] = {'t', data[2]};
  bool result = i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 2, data_t);
  result = i2cdevWrite(I2C1_DEV, b_addresses[index], I2CDEV_NO_MEM_ADDR, 4, data_p);
  DEBUG_PRINT("BlinkM play script %d %d %d => %d \n", data[0], data[1], (int8_t) data[2], result);
}

void blinkMSetCmd(uint32_t cmd)
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

// void blinkMSetAddress(uint8_t address)
// {
//   if(address != oldAddress && address > 0 && address <= 0xF0)
//   {
//     uint8_t data[5] = {'A', address, 0xd0, 0x0d, address};
//     bool result = i2cdevWrite(I2C1_DEV, 0x00, I2CDEV_NO_MEM_ADDR, 5, data);
//     DEBUG_PRINT("BlinkM changed address to %d => %d \n", address, result);
//     oldAddress = address;
//   }
// }

void blinkMStopScript()
{
  i2cdevWriteByte(I2C1_DEV, BLINKM_DEFAULT_ADDRESS, I2CDEV_NO_MEM_ADDR, 'o');
}


bool blinkMUpdate()
{
  blinkMSetFadeSpeed(fade_speed, 0);
  // setAddress(address);
  if(script)
  {
    blinkMSetScript(script, 0);
    script = 0;
    return true;
  }
  else if (cmd)
  {
    blinkMSetCmd(cmd);
    cmd = 0;
    return true;
  }
  return false;
}

static statusLed_t blinkMLed ={&blinkMSetColor, &blinkMUpdate, true, 100};

void blinkMInit(DeckInfo* info)
{
  if (isInit) {
    return;
  }
  bool res = i2cdevInit(I2C1_DEV);
  vTaskDelay(10);
  DEBUG_PRINT("i2cdevInit => %d \n", res);
  isInit =  blinkMSetFadeSpeed(255, 0);

   if(isInit)
   {
    blinkMGetInfo();
    // stopScript();
    // getColor();
    DEBUG_PRINT("BlinkM deck initialized! \n");
    initStatusLed(&blinkMLed);
  }
  else
  {
    DEBUG_PRINT("BlinkM deck NOT initialized! \n");
  }
}

static const DeckDriver blinkM_deck = {
  .vid = 0,
  .pid = 0,
  .usedGpio = DECK_USING_SDA | DECK_USING_SCL,
  .name = "blinkM",
  .init = blinkMInit,
};

DECK_DRIVER(blinkM_deck);

PARAM_GROUP_START(blinkM)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, isInit, &isInit)
PARAM_ADD(PARAM_UINT8, fadeSpeed, &fade_speed)
PARAM_ADD(PARAM_UINT32, script, &script)
PARAM_ADD(PARAM_UINT32, cmd, &cmd)
PARAM_ADD(PARAM_UINT8, useForStatus, &(blinkMLed.active))
PARAM_ADD(PARAM_UINT8, brightness, &(blinkMLed.brightness))
// PARAM_ADD(PARAM_UINT8, address, &address)
PARAM_GROUP_STOP(blinkM)
