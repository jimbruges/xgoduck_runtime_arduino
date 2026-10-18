#pragma once

#include <Arduino.h>

static const uint8_t NUM_MOTORS = 15;
static const uint32_t SERVO_BAUD = 1000000;
static const uint32_t SERVO_REPLY_US = 6000;

static const float M_N = 4095.0f;
static const float M_A = 360.0f;
// Present speed: bit 15 is direction, bits 0-14 are magnitude, 50 steps/s per unit.
static const float SERVO_SPEED_UNIT = 50.0f;
static const uint16_t POS_MIN = 0;
static const uint16_t POS_MAX = 4095;
static const uint16_t CAL_CENTER_POS = 2047;
// Finish re-centres a servo's position offset so its zero reads this, mid-range.
static const uint16_t CAL_ZERO_POS = 2048;
// Calibration moves at most this many encoder counts per 100 Hz tick (~35 deg/s).
static const uint16_t CAL_STEP_PER_TICK = 4;
static const uint16_t ALL_MOTORS_MASK = 0x7FFF;

static const uint8_t ADDR_GOAL_POS = 0x2A;
static const uint8_t ADDR_PRESENT = 0x38;
// Present position, speed, load (2 bytes each), then voltage (0.1 V) and temperature (°C).
static const uint8_t PRESENT_LEN = 8;
static const uint8_t ADDR_TORQUE = 40;
static const uint8_t ADDR_TEMP_KP = 50;
static const uint8_t ADDR_TEMP_KD = 51;
static const uint8_t ADDR_PERM_KP_KD = 0x15;
static const uint8_t ADDR_UNLOCK = 0x37;
static const uint8_t ADDR_ID = 0x05;
static const uint8_t ADDR_MIN_ANGLE = 9;   // EEPROM; min = max = 0 selects multi-turn mode
static const uint8_t ADDR_MAX_ANGLE = 11;
static const uint8_t ADDR_POS_OFFSET = 31;  // EEPROM; servo reports (encoder - offset) mod 4096
// Writing this to ADDR_TORQUE makes the servo store an offset so its present position reads 2048.
static const uint8_t TORQUE_SET_MIDDLE = 128;

static const uint8_t TEMP_KP_RUN = 6;
static const uint8_t TEMP_KD_RUN = 20;
static const uint8_t TEMP_KP_MOUTH = 10;
static const uint8_t MOUTH_MOTOR_ID = 34;
static const uint8_t TEMP_KP_CAL = 3;
static const uint8_t TEMP_KD_CAL = 0;

static const uint8_t MOTOR_IDS[NUM_MOTORS] = {
    10, 11, 12, 13, 14,
    20, 21, 22, 23, 24,
    30, 31, 32, 33, 34};

static const uint16_t ZERO_POS[NUM_MOTORS] = {
    2236, 2005, 1912, 1866, 1945,
    2127, 2135, 1914, 2228, 2206,
    2550, 2060, 2128, 2034, 2190};

// Default joint direction relative to the Microduck model the policies were trained on:
// every joint negative on a correctly assembled UNO Q robot. A robot with a servo mounted
// the other way round overrides this at runtime (CAL_SET_SIGNS from data/joint_signs.json,
// set on the Check page).
static const float MOTOR_SIGN[NUM_MOTORS] = {
    -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1};

static const float DEFAULT_DEG[NUM_MOTORS] = {
    0.0f, -5.0f, -23.0f, 0.0f, 23.0f,
    0.0f, 5.0f, 23.0f, 0.0f, -23.0f,
    20.0f, 20.0f, 0.0f, 0.0f, 0.0f};

struct Motor {
  uint8_t id;
  uint16_t zeroPos;
  uint16_t fbPos;
  float sign;
  float desAngleDeg;
  uint16_t desPos;
  float fbAngleDeg;
  float fbVelDeg;
  uint8_t fbVolt;  // 0.1 V
  uint8_t fbTemp;  // °C
};

extern Motor motors[NUM_MOTORS];
