#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

Servo servo1;
Servo servo2;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float pitch = 0;
float roll  = 0;
float rawpitch =0 , rawroll = 0;
int distance_perp = -1;
float distance_real = -1;

float getRealDistance(float d_perp, float pitch_deg, float roll_deg) {
  float pitch_rad = pitch_deg * DEG_TO_RAD;
  float roll_rad  = roll_deg  * DEG_TO_RAD;
  return d_perp / (cos(pitch_rad) * cos(roll_rad));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servo1.attach(9);
  servo2.attach(10);

  Serial.println("UNO READY - BNO055 + VL53L0X (fast update)");

  // --- Init BNO055 ---
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // --- Init VL53L0X ---
  delay(50);
  if (!lox.begin()) {
    Serial.println("VL53L0X not detected! Check wiring or XSHUT HIGH");
    while (1);
  }

  Serial.println("Sensors initialized successfully!");
}

void loop() {
  // --- BNO055 ---
  sensors_event_t event;
  bno.getEvent(&event);
  pitch = event.orientation.;
  roll  = event.orientation.y;
  rawpitch=pitch;
  rawroll=roll;
    Serial.print("P:"); Serial.print(pitch,2);
  Serial.print(" R:"); Serial.print(roll,2);

  if (roll < 0) {
    roll*=(-1);
    roll=90-roll;
  }
  else if (roll > 0) roll = 90+roll;

  // --- Servo control (optional) ---
  if (pitch > 0 && roll > 0) {
    servo2.write(pitch);
    servo1.write(roll);
  }

  // --- VL53L0X ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  distance_perp = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // --- Calcul distanta reala
  if (distance_perp >= 0) {
    distance_real = getRealDistance(distance_perp, rawpitch, rawroll);

    Serial.print("DIST:");
    Serial.println(distance_real, 2);
  }
  

  Serial.print(" Dp:"); 
  if(distance_perp>=0) Serial.print(distance_perp);
  else Serial.print("N/A");
  Serial.print(" Dr:"); 
  if(distance_perp>=0) Serial.print(distance_real,2);
  else Serial.print("N/A");
  Serial.println(" mm");
}
