#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define SDA_PIN 21
#define SCL_PIN 22

BNO080 imu;

// Quaternion + orientation variables
float qw, qx, qy, qz;
float instant_roll, instant_pitch, instant_yaw;

void computeOrientation() {
  if (!imu.dataAvailable()) return;

  qw = imu.getQuatReal();
  qx = imu.getQuatI();
  qy = imu.getQuatJ();
  qz = imu.getQuatK();

  // === MQSPEED mounting correction ===
  float qw_i = qw;
  float qx_i = qx;
  float qy_i = qy;
  float qz_i = qz;

  const float s = -0.70710678; // -sqrt(2)/2
  float qw_r = 0.0f;
  float qx_r = s;
  float qy_r = s;
  float qz_r = 0.0f;

  float qw_m = qw_r*qw_i - qx_r*qx_i - qy_r*qy_i - qz_r*qz_i;
  float qx_m = qw_r*qx_i + qx_r*qw_i + qy_r*qz_i - qz_r*qy_i;
  float qy_m = qw_r*qy_i - qx_r*qz_i + qy_r*qw_i + qz_r*qx_i;
  float qz_m = qw_r*qz_i + qx_r*qy_i - qy_r*qx_i + qz_r*qw_i;

  qw = qw_m;
  qx = qx_m;
  qy = qy_m;
  qz = qz_m;

  // === MQSPEED Euler conversion ===
  instant_roll  = atan2(2.0 * (qw*qx + qy*qz),
                        1.0 - 2.0 * (qx*qx + qy*qy));
  instant_pitch = asin(2.0 * (qw*qy - qz*qx));
  instant_yaw   = atan2(2.0 * (qw*qz + qx*qy),
                        1.0 - 2.0 * (qy*qy + qz*qz));

  instant_roll  *= 180.0 / PI;
  instant_pitch *= 180.0 / PI;
  instant_yaw   *= 180.0 / PI;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // REQUIRED for SparkFun SH-2

  delay(300); // FSM300 boot time

  if (!imu.begin()) {
    Serial.println("IMU not detected!");
    while (1) delay(10);
  }

  imu.enableRotationVector(10); // 10ms = 100Hz

  Serial.println("FSM300 SparkFun Calibration Interface Ready");
  Serial.println("Commands: tare | persist | clear | orientation");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "tare") {
      imu.tareNow();
      Serial.println("OK: tare applied");
    }
    else if (cmd == "persist") {
      imu.saveTare();
      Serial.println("OK: tare persisted");
    }
    else if (cmd == "clear") {
      imu.clearTare();
      Serial.println("OK: tare cleared");
    }
    else if (cmd == "orientation") {
      computeOrientation();
      Serial.print("roll=");
      Serial.print(instant_roll, 2);
      Serial.print(", pitch=");
      Serial.print(instant_pitch, 2);
      Serial.print(", yaw=");
      Serial.println(instant_yaw, 2);
    }
    else {
      Serial.println("ERR: unknown command");
    }
  }

  imu.dataAvailable(); // keep SH-2 packets flowing
}
