#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <esp_attr.h>

// ====== IMU Setup ======
#define SDA_PIN 21
#define SCL_PIN 22
#define BNO08X_RST -1

Adafruit_BNO08x bno08x(BNO08X_RST);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Starting IMU calibration interface...");
  Serial.println("Commands: tare | persist | status");

  //////////IMU SETUP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // BNO08X supports 400kHz
  delay(500);

  if (!bno08x.begin_I2C()) {
    Serial.println("❌ BNO08X not detected!");
    while (1) delay(10);
  }
  delay(300);
  
  // Enable rotation vector
 if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { // 10 ms
    Serial.println("Failed to enable rotation vector");
}

  Serial.println("IMU detected.");
  Serial.println("Ready for commands.");
}

void loop() {
  // Commands from Raspberry Pi to configure IMU
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equals("tare")) {
      if (bno08x.tareNow()) {
        Serial.println("OK: tare applied");
      } else {
        Serial.println("ERR: tare failed");
      }
    }

    else if (cmd.equals("persist")) {
      if (bno08x.persistTare()) {
        Serial.println("OK: tare persisted");
      } else {
        Serial.println("ERR: persist failed");
      }
    }

    else if (cmd.equals("status")) {
      Serial.println("IMU alive, ready for tare");
    }

    else {
      Serial.println("ERR: unknown command");
    }
  }
}
