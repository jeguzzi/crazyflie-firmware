#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "stabilizer.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "commander.h"
#include "timers.h"
#include "debug.h"
#include "worker.h"
#include "math3d.h"
#include "aideck_protocol.h"

// Uncomment to log the kalman state
// #define LOG_FILTERED_ODOM
// The setpoint priority.
// If larger than COMMANDER_PRIORITY_CRTP = 1, our control setpoint will have priority
// over external [crtp] setpoints)
#define SETPOINT_PRIORITY (COMMANDER_PRIORITY_CRTP + 1)

// helper function

static float normalize_angle(float value) {
  if(value > M_PI_F) value -= 2 * M_PI_F;
  else if(value < -M_PI_F) value += 2 * M_PI_F;
  return value;
}

static float x_;
static float y_;
static float z_;
static float phi_;
static uint32_t lastUpdate = 0;
static bool should_control = false;
static bool control_active = false;
static bool _control_active = false;
static bool relative_altitude = false;
static float target_range = 1.3f;
static float target_altitude = 1.4f;
static float eta = 1.0f;
static float maximal_speed = 1.0f;
static float maximal_angular_speed = 1.0f;
static float maximal_vertical_speed = 0.5f;
#define FREQ_STEPS 10
static uint32_t number_of_updates = 0;
static uint8_t update_hz = 0;
static uint32_t prevUpdate = 0;
static setpoint_t setpoint = {.mode.x = modeVelocity, .mode.y = modeVelocity,
                              .mode.z = modeVelocity, .mode.yaw = modeVelocity,
                              .velocity_body=false};

typedef struct {
  float x;
  float v;
  float p_xx;
  float p_vv;
  float p_xv;
} d1_kalman_state_t;

typedef struct {
  d1_kalman_state_t state;
  float r_xx;
  float q_vv;
  bool periodic;
} d1_kalman_t;

static d1_kalman_t x_odom = {.periodic=false, .r_xx=0.012f, .q_vv=2.7f,
                             .state={.p_xx=100, .p_xv=0, .p_vv=10}};
static d1_kalman_t y_odom = {.periodic=false, .r_xx=0.012f, .q_vv=2.7f,
                             .state={.p_xx=100, .p_xv=0, .p_vv=10}};
static d1_kalman_t z_odom = {.periodic=false, .r_xx=0.012f, .q_vv=1.0,
                             .state={.p_xx=100, .p_xv=0, .p_vv=10}};
static d1_kalman_t phi_odom = {.periodic=true, .r_xx=0.08f, .q_vv=5.3f,
                                 .state={.p_xx=10, .p_xv=0, .p_vv=10}};

// get_state returns a state_t in degrees for angles [and angular speeds]
// We use instead radians. We avoid filling roll and pitch as we won't need them.
state_t drone_state() {
  state_t * state = get_state();
  state_t d_state;
  d_state.position = state->position;
  d_state.attitude.yaw = radians(state->attitude.yaw);
  return d_state;
}

typedef struct {
  point_t position;  // m
  attitude_t attitude; // rad
} pose_t;

static pose_t pose_in_odom_frame(state_t * state, float x, float y, float z, float phi) {
  float yaw = state->attitude.yaw;
  float sn = sinf(yaw);
  float cs = cosf(yaw);
  pose_t pose;
  pose.position.x = state->position.x + cs * x - sn * y;
  pose.position.y = state->position.y + sn * x + cs * y;
  pose.position.z = state->position.z + z;
  pose.attitude.yaw = yaw + phi;
  return pose;
}

static pose_t target_pose(state_t * state) {
  pose_t pose;
  pose.position.x = x_odom.state.x + cosf(phi_odom.state.x) * target_range;
  pose.position.y = y_odom.state.x + sinf(phi_odom.state.x) * target_range;
  pose.position.z = relative_altitude ? z_odom.state.x + target_altitude : target_altitude;
  pose.attitude.yaw = atan2f(y_odom.state.x - state->position.y, x_odom.state.x - state->position.x);
  return pose;
}

static velocity_t desired_velocity(point_t position, velocity_t velocity,
                                   point_t target_position, velocity_t target_velocity) {
  velocity_t v;
  v.z = clamp((target_position.z - position.z) / eta + target_velocity.z, -maximal_vertical_speed, maximal_vertical_speed);
  v.x = (target_position.x - position.x) / eta + target_velocity.x;
  v.y = (target_position.y - position.y) / eta + target_velocity.y;
  float speed = sqrtf(v.x * v.x + v.y * v.y);
  if(speed > maximal_speed) {
    v.x *= maximal_speed / speed;
    v.y *= maximal_speed / speed;
  }
  return v;
}

static attitude_t desired_angular_velocity(attitude_t attitude, attitude_t target_attitude) {
  float diff = normalize_angle(target_attitude.yaw - attitude.yaw);
  attitude_t omega;
  omega.yaw = clamp(diff / eta, -maximal_angular_speed, maximal_angular_speed);
  return omega;
}

// TODO(Jerome): Add fence
static void update_control(setpoint_t * setpoint, state_t * state, pose_t target_pose,
                           velocity_t target_velocity) {
  // setpoint angles and angular speeds are in degrees and degrees/s
  setpoint->attitudeRate.yaw = degrees(desired_angular_velocity(state->attitude, target_pose.attitude).yaw);
  setpoint->velocity = desired_velocity(state->position, state->velocity, target_pose.position, target_velocity);
  setpoint->timestamp = lastUpdate;
}

void update_kalman(d1_kalman_t *filter, float dt, float x_new) {
  float q_vv = filter->q_vv * dt * dt;
  float x_ = filter->state.x + dt * filter->state.v;
  float p_xx_ = dt * dt * filter->state.p_vv + 2 * dt * filter->state.p_xv + filter->state.p_xx;
  float p_xv_ = dt * filter->state.p_vv + filter->state.p_xv;
  float p_vv_ = filter->state.p_vv + q_vv;
  float y = x_new - x_;
  if(filter->periodic) {
    y = normalize_angle(y);
  }
  float S = p_xx_ + filter->r_xx;
  filter->state.x = x_ + p_xx_ * y / S;
  filter->state.v = filter->state.v + p_xv_ * y / S;
  filter->state.p_xx = p_xx_ - p_xx_ * p_xx_ / S;
  filter->state.p_xv = p_xv_ - p_xv_ * p_xx_ / S;
  filter->state.p_vv = p_vv_ - p_xv_ * p_xv_ / S;
}

static xTimerHandle timer;

static void updateTimer(xTimerHandle timer) {
  if(!_control_active) {
    xTimerStop(timer, 1000);
    control_active = false;
  }
  else {
    _control_active = false;
  }
}

void inference_output_callback(inference_output_t *value) {
  x_ = value->x;
  y_ = value->y;
  z_ = value->z;
  phi_ = value->phi;
  float dt;
  if(lastUpdate) {
    dt = (float) (T2M(xTaskGetTickCount() - lastUpdate) / 1000.0);
  }
  else {
    dt = 0.0;
  }
  lastUpdate = xTaskGetTickCount();
  state_t state = drone_state();
  pose_t pose = pose_in_odom_frame(&state, x_, y_, z_, phi_ + M_PI_F);
  update_kalman(&x_odom, dt, pose.position.x);
  update_kalman(&y_odom, dt, pose.position.y);
  update_kalman(&z_odom, dt, pose.position.z);
  update_kalman(&phi_odom, dt, pose.attitude.yaw);
  pose = target_pose(&state);
  number_of_updates++;
#if defined(FREQ_STEPS) && FREQ_STEPS > 0
  if(number_of_updates % FREQ_STEPS == 0) {
    if(prevUpdate){
      update_hz = 1000 * FREQ_STEPS / T2M(lastUpdate - prevUpdate);
    }
    prevUpdate = lastUpdate;
    number_of_updates = 0;
  }
#endif

  if(!should_control) {
    control_active = false;
    return;
  }
  if(!timer) {
    timer = xTimerCreate( "updateTimer", M2T(500), pdTRUE, NULL, updateTimer);
  }
  if(!xTimerIsTimerActive(timer)) {
    xTimerStart(timer, 1000);
  }
  velocity_t velocity = {.x=x_odom.state.v, .y=y_odom.state.v, .z=z_odom.state.v};
  update_control(&setpoint, &state, pose, velocity);
  commanderSetSetpoint(&setpoint, SETPOINT_PRIORITY);
  control_active = _control_active = true;
}

PARAM_GROUP_START(frontnet)
PARAM_ADD(PARAM_UINT8, enable_control, &should_control)
PARAM_ADD(PARAM_UINT8, rel_altitude, &relative_altitude)
PARAM_ADD(PARAM_FLOAT, distance, &target_range)
PARAM_ADD(PARAM_FLOAT, altitude, &target_altitude)
PARAM_ADD(PARAM_FLOAT, eta, &eta)
PARAM_ADD(PARAM_FLOAT, max_speed, &maximal_speed)
PARAM_ADD(PARAM_FLOAT, max_ang_speed, &maximal_angular_speed)
PARAM_ADD(PARAM_FLOAT, max_vert_speed, &maximal_vertical_speed)
PARAM_ADD(PARAM_FLOAT, kalman_x_q, &x_odom.q_vv)
PARAM_ADD(PARAM_FLOAT, kalman_x_r, &x_odom.r_xx)
PARAM_ADD(PARAM_FLOAT, kalman_y_q, &y_odom.q_vv)
PARAM_ADD(PARAM_FLOAT, kalman_y_r, &y_odom.r_xx)
PARAM_ADD(PARAM_FLOAT, kalman_z_q, &z_odom.q_vv)
PARAM_ADD(PARAM_FLOAT, kalman_z_r, &z_odom.r_xx)
PARAM_ADD(PARAM_FLOAT, kalman_phi_q, &phi_odom.q_vv)
PARAM_ADD(PARAM_FLOAT, kalman_phi_r, &phi_odom.r_xx)
PARAM_GROUP_STOP(frontnet)

LOG_GROUP_START(frontnet)
LOG_ADD(LOG_FLOAT, x, &x_)
LOG_ADD(LOG_FLOAT, y, &y_)
LOG_ADD(LOG_FLOAT, z, &z_)
LOG_ADD(LOG_FLOAT, phi, &phi_)
LOG_ADD(LOG_UINT32, lastUpdate, &lastUpdate)
LOG_ADD(LOG_UINT8, control_active, &control_active)
LOG_ADD(LOG_UINT8, update_frequency, &update_hz)
#ifdef LOG_FILTERED_ODOM
LOG_ADD(LOG_FLOAT, f_x, &x_odom.state.x)
LOG_ADD(LOG_FLOAT, f_y, &y_odom.state.x)
LOG_ADD(LOG_FLOAT, f_z, &z_odom.state.x)
LOG_ADD(LOG_FLOAT, f_phi, &phi_odom.state.x)
LOG_ADD(LOG_FLOAT, f_vx, &x_odom.state.v)
LOG_ADD(LOG_FLOAT, f_vy, &y_odom.state.v)
LOG_ADD(LOG_FLOAT, f_vz, &z_odom.state.x)
LOG_ADD(LOG_FLOAT, f_vphi, &phi_odom.state.v)
#endif // LOG_FILTERED_ODOM
LOG_GROUP_STOP(frontnet)
