#include <ACAN_T4.h>
#include "Moteus.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float normalizeAngleDifference(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

#define PITCH_KP  0.10f
#define PITCH_KI  0.005f
#define PITCH_KD  0.002f
#define ROLL_KP   0.10f
#define ROLL_KI   0.002f
#define ROLL_KD   0.0012f
#define MAX_TORQUE 7.0f

Moteus moteus_pitch;
Moteus moteus_roll;
Moteus moteus_hight;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const float TARGET_PITCH = -0.4f;
const float TARGET_ROLL  = -175.04f;
const float TARGET_DISTANCE = 50.0f;
const float MM_TO_ROT = 0.5f;
const float MAX_POS = 0.5f;
const float MIN_POS = -40.0f;

bool system_enabled = true;
float distance_from_uno = 0.0f;
float pitch_integral = 0.0f;
float roll_integral  = 0.0f;
unsigned long prev_time = 0;
float pitch_prev_error = 0.0f;
float roll_prev_error  = 0.0f;

#define FILTER_SIZE 5
float distance_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_filled = false;

void addDistanceSample(float new_sample) {
  distance_buffer[buffer_index] = new_sample;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;
  if (buffer_index == 0) buffer_filled = true;
}

float getAverageDistance() {
  int count = buffer_filled ? FILTER_SIZE : buffer_index;
  if (count == 0) return distance_from_uno;
  float sum = 0.0f;
  for (int i = 0; i < count; i++) sum += distance_buffer[i];
  return sum / count;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial1.begin(115200);

  if (!bno.begin()) while(1);
  bno.setExtCrystalUse(true);

  ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);
  ACAN_T4::can3.beginFD(settings);

  moteus_pitch.options_.id = 1; 
  moteus_pitch.Initialize(); delay(200);

  moteus_roll.options_.id = 2;  
  moteus_roll.Initialize();  delay(200);

  moteus_hight.options_.id = 3; 
  moteus_hight.Initialize(); delay(200);

  Serial.println("[SYSTEM] Ready");
}

void loop() {
  imu::Quaternion quat = bno.getQuat();
  float w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  float roll_rad = atan2(sinr_cosp, cosr_cosp);
  float sinp = 2.0f * (w * y - z * x);
  float pitch_rad = (abs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);
  float roll_deg  = roll_rad  * 180.0f / M_PI;
  float pitch_deg = pitch_rad * 180.0f / M_PI;

  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("DIST:")) {
      float val = msg.substring(5).toFloat();
      if (val > 0.0f && val < 10000.0f) {
        distance_from_uno = val;
        addDistanceSample(val);
      }
    }
  }

  float avg_distance = getAverageDistance();
  unsigned long now = micros();
  float dt = (prev_time > 0) ? (now - prev_time) / 1e6f : 0.002f;
  prev_time = now;

  // PITCH
  float pitch_error = TARGET_PITCH - pitch_deg;
  pitch_integral += pitch_error * dt;
  pitch_integral = constrain(pitch_integral, -50.0f, 50.0f);
  float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  pitch_prev_error = pitch_error;
  float pitch_correction_deg = (PITCH_KP * pitch_error) + (PITCH_KI * pitch_integral) + (PITCH_KD * pitch_derivative);
  pitch_correction_deg = constrain(pitch_correction_deg, -8.0f, 8.0f);

  if (moteus_pitch.SetQuery()) {
    float pitch_correction_rad = pitch_correction_deg * M_PI / 180.0f;
    float current_pitch_pos = moteus_pitch.last_result().values.position;
    float target_pitch_pos = current_pitch_pos - pitch_correction_rad;
    target_pitch_pos = current_pitch_pos + constrain(target_pitch_pos - current_pitch_pos, -0.05f, 0.05f);
    Moteus::PositionMode::Command cmd;
    cmd.position = target_pitch_pos;
    cmd.maximum_torque = MAX_TORQUE;
    moteus_pitch.BeginPosition(cmd);
  }

  // ROLL
  float roll_error = normalizeAngleDifference(TARGET_ROLL, roll_deg);
  roll_integral += roll_error * dt;
  roll_integral = constrain(roll_integral, -50.0f, 50.0f);
  float roll_derivative = (roll_error - roll_prev_error) / dt;
  roll_prev_error = roll_error;
  float roll_correction_deg = (ROLL_KP * roll_error) + (ROLL_KI * roll_integral) + (ROLL_KD * roll_derivative);
  roll_correction_deg = constrain(roll_correction_deg, -8.0f, 8.0f);

  if (moteus_roll.SetQuery()) {
    float roll_correction_rad = roll_correction_deg * M_PI / 180.0f;
    float current_roll_pos = moteus_roll.last_result().values.position;
    float target_roll_pos = current_roll_pos - roll_correction_rad;
    target_roll_pos = current_roll_pos + constrain(target_roll_pos - current_roll_pos, -0.05f, 0.05f);
    Moteus::PositionMode::Command cmd;
    cmd.position = target_roll_pos;
    cmd.maximum_torque = MAX_TORQUE;
    moteus_roll.BeginPosition(cmd);
  }
  Serial.print("Pitch:");
  Serial.print(pitch_deg, 1);
  Serial.print(" Roll:");
  Serial.println(roll_deg, 2);

  HEIGHT
  if (moteus_hight.SetQuery()) {
    float avg_mm = avg_distance;
    float error_mm = TARGET_DISTANCE - avg_mm;
    float target_pos;
    if (fabs(error_mm) > 5.0f) {
      target_pos = constrain(error_mm * MM_TO_ROT, MIN_POS, MAX_POS);
    } else {
      target_pos = moteus_hight.last_result().values.position;
    }
      Moteus::PositionMode::Command h_cmd;
      h_cmd.position = target_pos;
      h_cmd.maximum_torque = MAX_TORQUE;
      moteus_hight.BeginPosition(h_cmd);

      // Serial.print("[H3] dist=");
      // Serial.print(avg_mm, 2);
      // Serial.print(" | err_mm=");
      // Serial.print(error_mm, 2);
      // Serial.print(" | tgt=");
      // Serial.println(target_pos, 3);
    
  }

  moteus_pitch.Poll();
  moteus_roll.Poll();
  moteus_hight.Poll();

  delay(50);
}
