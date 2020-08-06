/*
  This file was autogenerate on 07/28/20 from the following Python config:

  from base import config, input, c_float, c_uint8, c_uint16  # type: ignore


  @config(name='camera', group='aideck_HIMAX', header="!CAM")
  class Camera:
      marginTop: c_uint16
      marginRight: c_uint16
      marginBottom: c_uint16
      marginLeft: c_uint16
      format: c_uint8  # noqa
      step: c_uint8
      target_value: c_uint8
      ae: c_uint8
      fps: c_uint8


  @config(name='stream', group='aideck_stream', header="!STR")
  class Stream:
      on: c_uint8
      format: c_uint8  # noqa
      transport: c_uint8


  @config(name='inference', group='aideck_inference', header="!INF")
  class Inference:
      active: c_uint8
      verbose: c_uint8


  @input(name='inference_output', header=b"\x90\x19\x08\x31")
  class InferenceInput:
      x: c_float
      y: c_float
      z: c_float
      phi: c_float

*/

#define VERBOSE_RX
#define VERBOSE_TX
// --- input inference_output

typedef struct {
  float x;
  float y;
  float z;
  float phi;
} __attribute__((packed)) inference_output_t;

// To be implemented
void inference_output_callback(inference_output_t *);
