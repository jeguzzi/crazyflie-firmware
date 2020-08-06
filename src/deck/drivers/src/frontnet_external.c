#include <stdint.h>
#include "aideck_protocol.h"
#include "task.h"
#include "log.h"


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
