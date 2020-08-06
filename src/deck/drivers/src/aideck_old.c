/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * aideck.c - Deck driver for the AIdeck
 */
#define DEBUG_MODULE "AIDECK"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "system.h"
#include "uart1.h"
#include "uart2.h"

#include "timers.h"
#include "worker.h"

#define HEADER_LENGTH 4

typedef struct StreamConfig
{
  uint8_t on;
  uint8_t format; // 0 RAW, 1 JPEG
  uint8_t transport; // 0 WIFI, 1 PIPE
} __attribute__((packed)) stream_config_t;

static stream_config_t desired_stream_config;
static stream_config_t _desired_stream_config;
// static uint8_t stream;

typedef struct CameraConfig
{
  uint16_t top;
  uint16_t right;
  uint16_t bottom;
  uint16_t left;
  uint8_t format;
  uint8_t step;
  uint8_t target_value;
} __attribute__((packed)) camera_config_t;


static camera_config_t desired_camera_config;
static camera_config_t _desired_camera_config;

typedef uint8_t inference_config_t;
static inference_config_t desired_inference_config;
static inference_config_t _desired_inference_config;


// static camera_config_t camera_config;

//  = {
//   .top = 33,
//   .right = 2,
//   .bottom = 33,
//   .left = 0,
//   .format = 3,
//   .step = 1,
//   .target_value = 0x80
// };

static xTimerHandle timer;

static uint8_t camera_config_header[HEADER_LENGTH] = "!CAM";
static uint8_t stream_config_header[HEADER_LENGTH] = "!STR";
static uint8_t inference_config_header[HEADER_LENGTH] = "!INF";

void update_config(void *data)
{
  // CHANGED: update 1 config at time
  if(memcmp(&desired_camera_config, &_desired_camera_config, sizeof(camera_config_t)))
  {
    DEBUG_PRINT("Will update the camera config\n");
    // memcpy(&_camera_config, &camera_config, sizeof(CameraConfig));

    uart1SendData(HEADER_LENGTH, camera_config_header);
    // vTaskDelay(M2T(100));
    // uart1SendData(HEADER_LENGTH, camera_config_header);
    uart1SendData(sizeof(camera_config_t), (uint8_t *)&desired_camera_config);
    _desired_camera_config = desired_camera_config;
    return;
    // DEBUG_PRINT("Updated the camera config %d\n", sizeof(CameraConfig));
  }
  if(memcmp(&desired_stream_config, &_desired_stream_config, sizeof(stream_config_t)))
  {
    DEBUG_PRINT("Will update the stream config\n");
    uart1SendData(HEADER_LENGTH, stream_config_header);
    uart1SendData(sizeof(stream_config_t), (uint8_t *)&desired_stream_config);
    _desired_stream_config = desired_stream_config;
    return;
  }
  if(memcmp(&desired_inference_config, &_desired_inference_config, sizeof(inference_config_t)))
  {
    DEBUG_PRINT("Will update the inference config\n");
    uart1SendData(HEADER_LENGTH, inference_config_header);
    uart1SendData(sizeof(inference_config_t), (uint8_t *)&desired_inference_config);
    _desired_inference_config = desired_inference_config;
    return;
  }
}

static void configTimer(xTimerHandle timer)
{
  workerSchedule(update_config, NULL);
}


static bool isInit = false;
static uint8_t in_byte = 0;
static uint8_t old_byte = 0;

//Uncomment when NINA printout read is desired from console
// #define DEBUG_NINA_PRINT

#ifdef DEBUG_NINA_PRINT
static void NinaTask(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Starting reading out NINA debugging messages:\n");
    vTaskDelay(M2T(2000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the NINA sends and immediately send it to the console.
    uint8_t byte;
    while (1)
    {
        if (uart2GetDataWithTimout(&byte) == true)
        {
            consolePutchar(byte);
        }
        if(in_byte != old_byte)
        {
          old_byte = in_byte;
          uart2SendData(1, &old_byte);
        }
    }
}
#endif

static float x_;
static float y_;
static float z_;
static float phi_;
static uint32_t lastUpdate = 0;


#define BUFFER_LENGTH 100
#define HEADER_NUMBER 4
static uint8_t header_buffer[HEADER_LENGTH];
static uint8_t rx_buffer[BUFFER_LENGTH];

static int read_uart_bytes(int size)
{
  // DEBUG_PRINT("Should read %d bytes and store them in %d\n", size, bytes);
  uint8_t *byte = rx_buffer;
  for (int i = 0; i < size; i++) {
    if(uart1GetDataWithTimout(byte))
    {
      // DEBUG_PRINT("Got %d\n", *byte);
      byte++;
    }
    else
    {
      return i;
    }
  }
  // DEBUG_PRINT("Read %d bytes: %d ...\n", size, bytes[0]);
  return size;
}

static void camera_cb()
{
  int size = read_uart_bytes(sizeof(camera_config_t));
  if( size == sizeof(camera_config_t))
  {
    camera_config_t *c = (camera_config_t *)rx_buffer;
    desired_camera_config = _desired_camera_config = *c;
    DEBUG_PRINT("Has updated camera config\n");
  }
  else{
    DEBUG_PRINT("Failed to update camera config: (%d vs %d byes received)\n",
                size, sizeof(camera_config_t));
  }
}

static void stream_cb()
{
  int size = read_uart_bytes(sizeof(stream_config_t));
  if( size == sizeof(stream_config_t))
  {
    stream_config_t *c = (stream_config_t *)rx_buffer;
    desired_stream_config = _desired_stream_config = *c;
    DEBUG_PRINT("Has updated stream config\n");
  }
  else{
    DEBUG_PRINT("Failed to update stream config: (%d vs %d byes received)\n",
                size, sizeof(stream_config_t));
  }
}


static void inference_cb()
{
  int size = read_uart_bytes(sizeof(inference_config_t));
  if( size == sizeof(inference_config_t))
  {
    inference_config_t *c = (inference_config_t *)rx_buffer;
    desired_inference_config = _desired_inference_config = *c;
    DEBUG_PRINT("Has updated inference config\n");
  }
  else{
    DEBUG_PRINT("Failed to update inference config: (%d vs %d byes received)\n",
                size, sizeof(inference_config_t));
  }
}

static void inference_output_cb()
{
  if(read_uart_bytes(4 * sizeof(float)) == 4 * sizeof(float))
  {
    float * buffer = (float *) rx_buffer;
    x_ = buffer[0];
    y_ = buffer[1];
    z_ = buffer[2];
    phi_ = buffer[3];
    lastUpdate = xTaskGetTickCount();
    // DEBUG_PRINT("Got inference %.3f %.3f %.3f %.3f\n", x_, y_, z_, phi_);
  }
}

typedef void (*Callback)(void);
static const char headers[HEADER_NUMBER][HEADER_LENGTH]  = {"\x90\x19\x08\x31", "!CAM", "!STR", "!INF"};
static const Callback message_callback[HEADER_NUMBER] = {inference_output_cb, camera_cb, stream_cb, inference_cb};

// Read UART 1 while looking for structured messages.
// When none are found, print everything to console.

static void read_uart_message()
{
  uint8_t *byte = header_buffer;
  int n = 0;
  int index;
  int valid_index[HEADER_NUMBER];
  for (index = 0; index < HEADER_NUMBER; index++) valid_index[index] = 1;
  while(n < HEADER_LENGTH)
  {
    if(uart1GetDataWithTimout(byte))
    {
      // DEBUG_PRINT("Got 0x%x\n", *byte);
      int valid = 0;
      for (index = 0; index < HEADER_NUMBER; index++) {
        if(!valid_index[index]) continue;
        if(*byte != headers[index][n]){
          // DEBUG_PRINT("Expected 0x%x\n", headers[index][n]);
          valid_index[index] = 0;
        }
        else{
          valid = 1;
        }
      }
      n++;
      if(valid)
      {
        // Keep reading
        byte++;
        continue;
      }
    }
    // DEBUG_PRINT("No header after %d bytes\n", n);
    // forward to console and return;
    for (size_t i = 0; i < n; i++) {
      consolePutchar(header_buffer[i]);
    }
    return;
  }
  // Found message with id = index
  // Call the corresponding callback
  for (index = 0; index < HEADER_NUMBER; index++)
  {
    if(valid_index[index]) break;
  }
  // DEBUG_PRINT("Got callback %d\n", index);
  message_callback[index]();
}

static void Gap8Task(void *param)
{
    systemWaitStart();
    vTaskDelay(M2T(1000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
    DEBUG_PRINT("Starting UART listener\n");
    while (1)
    {
        read_uart_message();
    }
}

// static uint8_t _header[4] = {0x90, 0x19, 0x08, 0x31};
// static float rx[5] = {0, 0, 0, 0, 0};

// static void Gap8Task(void *param)
// {
//     systemWaitStart();
//     vTaskDelay(M2T(1000));
//
//     // Pull the reset button to get a clean read out of the data
//     pinMode(DECK_GPIO_IO4, OUTPUT);
//     digitalWrite(DECK_GPIO_IO4, LOW);
//     vTaskDelay(10);
//     digitalWrite(DECK_GPIO_IO4, HIGH);
//     pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
//     uint8_t *byte = (uint8_t *)rx;
//     uint8_t *end = byte + 5 * 4;
//     uint8_t *end_header = byte + 4;
//     uint8_t valid = 0;
//     uint8_t *h = _header;
//     // Read out the byte the Gap8 sends and immediately send it to the console.
//     DEBUG_PRINT("Starting UART listener\n");
//     while (1)
//     {
//         if(uart1GetDataWithTimout(byte))
//         {
//           if(!valid)
//           {
//             // DEBUG_PRINT("Got: 0x%x vs 0x%x\n", *byte, *h);
//             if(*h == *byte)
//             {
//               byte++;
//               h++;
//             }
//             else{
//               byte = (uint8_t *) &rx;
//               h = _header;
//               continue;
//             }
//             // DEBUG_PRINT("End? %d\n", (int) (end_header - byte));
//             if(byte == end_header)
//             {
//               // DEBUG_PRINT("VALID\n");
//               valid = 1;
//               h = _header;
//             }
//             continue;
//           }
//           byte++;
//           if(byte == end)
//           {
//             x_ = rx[1];
//             y_ = rx[2];
//             z_ = rx[3];
//             phi_ = rx[4];
//             // DEBUG_PRINT("GOT %d %d %d %d\n", rx[1], rx[2], rx[3], rx[4]);
//             // DEBUG_PRINT("-> floats %.4f %.4f %.4f %.4f\n", x_, y_, z_, phi_);
//             lastUpdate = xTaskGetTickCount();
//             byte = (uint8_t *) &rx;
//             valid = 0;
//           }
//         }
//     }
// }

static void aideckInit(DeckInfo *info)
{

    if (isInit)
        return;

    // Intialize the UART for the GAP8
    uart1Init(115200);
    // Initialize task for the GAP8
    xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

#ifdef DEBUG_NINA_PRINT
    // Initialize the UART for the NINA
    uart2Init(115200);
    // Initialize task for the NINA
    xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

#endif
  // memcpy(&_camera_config, &camera_config, sizeof(camera_config_t));
  _desired_stream_config = desired_stream_config;
  _desired_camera_config = desired_camera_config;
  timer = xTimerCreate( "configTimer", M2T(1000), pdTRUE, NULL, configTimer );
  xTimerStart(timer, 1000);

  isInit = true;
}

static bool aideckTest()
{

    return true;
}

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedPeriph = 0,
    .usedGpio = 0, // FIXME: Edit the used GPIOs

    .init = aideckInit,
    .test = aideckTest,
};

LOG_GROUP_START(frontnet)
LOG_ADD(LOG_FLOAT, x, &x_)
LOG_ADD(LOG_FLOAT, y, &y_)
LOG_ADD(LOG_FLOAT, z, &z_)
LOG_ADD(LOG_FLOAT, phi, &phi_)
LOG_ADD(LOG_UINT32, lastUpdate, &lastUpdate)
LOG_GROUP_STOP(frontnet)


PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)
PARAM_GROUP_STOP(deck)

#ifdef DEBUG_NINA_PRINT
PARAM_GROUP_START(NINA)
PARAM_ADD(PARAM_UINT8, byte, &in_byte)
PARAM_GROUP_STOP(NINA)
#endif

PARAM_GROUP_START(frontnet)
PARAM_ADD(PARAM_UINT8, active, &desired_inference_config)
PARAM_GROUP_STOP(frontnet)

PARAM_GROUP_START(HIMAX)
PARAM_ADD(PARAM_UINT16, marginTop, &desired_camera_config.top)
PARAM_ADD(PARAM_UINT16, marginRight, &desired_camera_config.right)
PARAM_ADD(PARAM_UINT16, marginBottom, &desired_camera_config.bottom)
PARAM_ADD(PARAM_UINT16, marginLeft, &desired_camera_config.left)
PARAM_ADD(PARAM_UINT8, format, &desired_camera_config.format)
PARAM_ADD(PARAM_UINT8, step, &desired_camera_config.step)
PARAM_ADD(PARAM_UINT8, target_value, &desired_camera_config.target_value)
PARAM_ADD(PARAM_UINT8, streamOn, &desired_stream_config.on)
PARAM_ADD(PARAM_UINT8, streamFormat, &desired_stream_config.format)
PARAM_ADD(PARAM_UINT8, streamTransport, &desired_stream_config.transport)
PARAM_GROUP_STOP(HIMAX)

DECK_DRIVER(aideck_deck);
