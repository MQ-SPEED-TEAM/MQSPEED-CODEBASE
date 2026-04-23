#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define BNO08X_INT  -1
#define BNO08X_RST  -1

BNO08x imu;

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(300);

  if (!imu.begin(BNO08x_DEFAULT_ADDRESS, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("IMU not detected!");
    while (1) delay(10);
  }

  if (!imu.enableRotationVector()) {
    Serial.println("Failed to enable rotation vector");
  }

  Serial.println("FSM300 CEVA Calibration Interface Ready");
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

      if (imu.getSensorEvent() == true) {
        if (imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

          float roll  = imu.getRoll()  * 180.0 / PI;
          float pitch = imu.getPitch() * 180.0 / PI;
          float yaw   = imu.getYaw()   * 180.0 / PI;

          Serial.print("roll=");
          Serial.print(roll, 2);
          Serial.print(", pitch=");
          Serial.print(pitch, 2);
          Serial.print(", yaw=");
          Serial.println(yaw, 2);
        }
      }
    }
    else {
      Serial.println("ERR: unknown command");
    }
  }

  // No dataAvailable() in this library
}
