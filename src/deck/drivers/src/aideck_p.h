/*
  This file was autogenerate on 07/09/20 from the following Python config:

  from base import Config, Input, c_float, c_uint8, c_uint16  # type: ignore


  class Camera(Config):
      marginTop: c_uint16
      marginRight: c_uint16
      marginBottom: c_uint16
      marginLeft: c_uint16
      format: c_uint8  # noqa
      step: c_uint8
      target_value: c_uint8


  class Stream(Config):
      on: c_uint8
      format: c_uint8  # noqa
      transport: c_uint8


  class Inference(Config):
      active: c_uint8


  class InferenceInput(Input):
      x: c_float
      y: c_float
      z: c_float
      phi: c_float


  configs = [
      Camera(name='camera', group='aideck_HIMAX', header="!CAM"),
      Stream(name='stream', group='aideck_stream', header="!STR"),
      Inference(name='inference', group='aideck_inference', header="!INF")
  ]

  inputs = [
      InferenceInput(name='inference_output', group='frontnet', header="\\x90\\x19\\x08\\x31")
  ]

*/

#define HEADER_LENGTH 4
#define INPUT_NUMBER 4

typedef struct {
  const char *header;
  uint8_t size;
  void (*callback)(void *);
  uint8_t valid;
} input_t;
// --- config camera

typedef struct {
  uint16_t marginTop;
  uint16_t marginRight;
  uint16_t marginBottom;
  uint16_t marginLeft;
  uint8_t format;
  uint8_t step;
  uint8_t target_value;
} __attribute__((packed)) camera_t;


void log_camera(camera_t *value)
{
  DEBUG_PRINT("marginTop=%u, marginRight=%u, marginBottom=%u, marginLeft=%u, format=%u, step=%u, target_value=%u\n", value->marginTop, value->marginRight, value->marginBottom, value->marginLeft, value->format, value->step, value->target_value);
}

static struct {
  camera_t value, dvalue;
  const char *header;
} __camera__config = { .header= "!CAM" };

static void __camera_cb(void *buffer)
{
  camera_t *value = (camera_t *)buffer;
  __camera__config.value = __camera__config.dvalue = *value;
  DEBUG_PRINT("GAP has updated camera config\n");
  log_camera(&(__camera__config.value));
}

// --- config stream

typedef struct {
  uint8_t on;
  uint8_t format;
  uint8_t transport;
} __attribute__((packed)) stream_t;


void log_stream(stream_t *value)
{
  DEBUG_PRINT("on=%u, format=%u, transport=%u\n", value->on, value->format, value->transport);
}

static struct {
  stream_t value, dvalue;
  const char *header;
} __stream__config = { .header= "!STR" };

static void __stream_cb(void *buffer)
{
  stream_t *value = (stream_t *)buffer;
  __stream__config.value = __stream__config.dvalue = *value;
  DEBUG_PRINT("GAP has updated stream config\n");
  log_stream(&(__stream__config.value));
}

// --- config inference

typedef struct {
  uint8_t active;
} __attribute__((packed)) inference_t;


void log_inference(inference_t *value)
{
  DEBUG_PRINT("active=%u\n", value->active);
}

static struct {
  inference_t value, dvalue;
  const char *header;
} __inference__config = { .header= "!INF" };

static void __inference_cb(void *buffer)
{
  inference_t *value = (inference_t *)buffer;
  __inference__config.value = __inference__config.dvalue = *value;
  DEBUG_PRINT("GAP has updated inference config\n");
  log_inference(&(__inference__config.value));
}

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
  { .header = "!CAM", .callback = __camera_cb, .size = sizeof(camera_t) },
  { .header = "!STR", .callback = __stream_cb, .size = sizeof(stream_t) },
  { .header = "!INF", .callback = __inference_cb, .size = sizeof(inference_t) },
  { .header = "\x90\x19\x08\x31", .callback = __inference_output_cb, .size = sizeof(inference_output_t) }
};
void update_config(void *data)
{
  if(memcmp(&(__camera__config.value), &(__camera__config.dvalue), sizeof(camera_t)))
  {
    DEBUG_PRINT("Will request GAP to update camera config\n");
    log_camera(&(__camera__config.value));
    uart1SendData(HEADER_LENGTH, (uint8_t *) __camera__config.header);
    uart1SendData(sizeof(camera_t), (uint8_t *)&(__camera__config.value));
    __camera__config.dvalue = __camera__config.value;
    return;
  }

  if(memcmp(&(__stream__config.value), &(__stream__config.dvalue), sizeof(stream_t)))
  {
    DEBUG_PRINT("Will request GAP to update stream config\n");
    log_stream(&(__stream__config.value));
    uart1SendData(HEADER_LENGTH, (uint8_t *) __stream__config.header);
    uart1SendData(sizeof(stream_t), (uint8_t *)&(__stream__config.value));
    __stream__config.dvalue = __stream__config.value;
    return;
  }

  if(memcmp(&(__inference__config.value), &(__inference__config.dvalue), sizeof(inference_t)))
  {
    DEBUG_PRINT("Will request GAP to update inference config\n");
    log_inference(&(__inference__config.value));
    uart1SendData(HEADER_LENGTH, (uint8_t *) __inference__config.header);
    uart1SendData(sizeof(inference_t), (uint8_t *)&(__inference__config.value));
    __inference__config.dvalue = __inference__config.value;
    return;
  }

}

PARAM_GROUP_START(aideck_HIMAX)
PARAM_ADD(PARAM_UINT16, marginTop, &(__camera__config.value.marginTop))
PARAM_ADD(PARAM_UINT16, marginRight, &(__camera__config.value.marginRight))
PARAM_ADD(PARAM_UINT16, marginBottom, &(__camera__config.value.marginBottom))
PARAM_ADD(PARAM_UINT16, marginLeft, &(__camera__config.value.marginLeft))
PARAM_ADD(PARAM_UINT8, format, &(__camera__config.value.format))
PARAM_ADD(PARAM_UINT8, step, &(__camera__config.value.step))
PARAM_ADD(PARAM_UINT8, target_value, &(__camera__config.value.target_value))
PARAM_GROUP_STOP(aideck_HIMAX)

PARAM_GROUP_START(aideck_inference)
PARAM_ADD(PARAM_UINT8, active, &(__inference__config.value.active))
PARAM_GROUP_STOP(aideck_inference)

PARAM_GROUP_START(aideck_stream)
PARAM_ADD(PARAM_UINT8, on, &(__stream__config.value.on))
PARAM_ADD(PARAM_UINT8, format, &(__stream__config.value.format))
PARAM_ADD(PARAM_UINT8, transport, &(__stream__config.value.transport))
PARAM_GROUP_STOP(aideck_stream)
