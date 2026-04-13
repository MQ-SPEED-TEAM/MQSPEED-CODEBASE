#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

BNO08x imu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(500);

  Serial.println("Starting IMU calibration interface...");
  Serial.println("Commands: tare | persist | clear");

  if (!imu.begin()) {
    Serial.println("IMU not detected!");
    while (1);
  }
  delay(300);

  imu.enableRotationVector(10); // 10ms

  Serial.println("IMU detected.");
  Serial.println("Ready for commands.");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Apply tare immediately
    if (cmd == "tare") {
      if (imu.tareNow()) {
        Serial.println("OK: tare applied");
      } else {
        Serial.println("ERR: tare failed");
      }
    }

    // Persist tare to flash
    else if (cmd == "persist") {
      if (imu.saveTare()) {
        Serial.println("OK: tare persisted to flash");
      } else {
        Serial.println("ERR: persist failed");
      }
    }

    // Clear stored tare
    else if (cmd == "clear") {
      if (imu.clearTare()) {
        Serial.println("OK: tare cleared");
      } else {
        Serial.println("ERR: clear failed");
      }
    }

    else {
      Serial.println("ERR: unknown command");
    }
  }
}
