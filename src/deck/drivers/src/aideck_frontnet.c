/*
This file is an extension of the original example provided by Bitcraze
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

// Include the autogenerate code implementing the protocol
#define HEADER_LENGTH 4
#define INPUT_NUMBER 1

typedef struct {
  const char *header;
  uint8_t size;
  void (*callback)(void *);
  bool valid;
} input_t;
// --- input inference_output

typedef struct {
  float x;
  float y;
  float z;
  float phi;
} __attribute__((packed)) inference_output_t;


void log_inference_output(inference_output_t *value)
{
  DEBUG_PRINT("x=%.3g, y=%.3g, z=%.3g, phi=%.3g\n", value->x, value->y, value->z, value->phi);
}

// To be implemented
static void inference_output_callback(inference_output_t *);
static void __inference_output_cb(void *buffer)
{
  inference_output_t *value = (inference_output_t *)buffer;
  inference_output_callback(value);
}
static input_t inputs[INPUT_NUMBER]  = {
  { .header = "\x90\x19\x08\x31", .callback = __inference_output_cb, .size = sizeof(inference_output_t) }
};

static float x_;
static float y_;
static float z_;
static float phi_;
static uint32_t lastUpdate = 0;

static void inference_output_callback(inference_output_t *value)
{
  x_ = value->x;
  y_ = value->y;
  z_ = value->z;
  phi_ = value->phi;
  lastUpdate = xTaskGetTickCount();
}

LOG_GROUP_START(frontnet)
LOG_ADD(LOG_FLOAT, x, &x_)
LOG_ADD(LOG_FLOAT, y, &y_)
LOG_ADD(LOG_FLOAT, z, &z_)
LOG_ADD(LOG_FLOAT, phi, &phi_)
LOG_ADD(LOG_UINT32, lastUpdate, &lastUpdate)
LOG_GROUP_STOP(frontnet)

#ifdef AIDECK_HAS_CONFIGS
static xTimerHandle timer;
static void configTimer(xTimerHandle timer)
{
  workerSchedule(update_config, NULL);
}
#endif

static bool isInit = false;

// Uncomment when NINA printout read is desired from console
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

// Read n bytes from UART, returning the read size before ev. timing out.
static int read_uart_bytes(int size, uint8_t *buffer)
{
  uint8_t *byte = buffer;
  for (int i = 0; i < size; i++) {
    if(uart1GetDataWithTimout(byte))
    {
      byte++;
    }
    else
    {
      return i;
    }
  }
  return size;
}

// Read UART 1 while looking for structured messages.
// When none are found, print everything to console.
static uint8_t header_buffer[HEADER_LENGTH];
static void read_uart_message()
{
  uint8_t *byte = header_buffer;
  int n = 0;
  input_t *input;
  input_t *begin = (input_t *) inputs;
  input_t *end = begin + INPUT_NUMBER;
  for (input = begin; input < end; input++) input->valid = 1;
  while(n < HEADER_LENGTH)
  {
    if(uart1GetDataWithTimout(byte))
    {
      int valid = 0;
      for (input = begin; input < end; input++) {
        if(!(input->valid)) continue;
        if(*byte != (input->header)[n]){
          input->valid = 0;
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
    // forward to console and return;
    for (size_t i = 0; i < n; i++) {
      consolePutchar(header_buffer[i]);
    }
    return;
  }
  // Found message
  for (input = begin; input < end; input++)
  {
    if(input->valid) break;
  }
  uint8_t buffer[input->size];
  int size = read_uart_bytes(input->size, buffer);
  if( size == input->size )
  {
    // Call the corresponding callback
    input->callback(buffer);
  }
  else{
    DEBUG_PRINT("Failed to receive message %4s: (%d vs %d bytes received)\n",
                 input->header, size, input->size);
  }
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
#ifdef AIDECK_HAS_CONFIGS
  timer = xTimerCreate( "configTimer", M2T(1000), pdTRUE, NULL, configTimer );
  xTimerStart(timer, 1000);
#endif
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

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)
PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
