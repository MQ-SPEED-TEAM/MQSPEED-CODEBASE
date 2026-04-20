#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ===== I2C pins (match MQSpeed) =====
#define SDA_PIN 21
#define SCL_PIN 22

// Optional pins (same as MQSpeed)
#define BNO08X_INT -1
#define BNO08X_RST -1

Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

// Orientation variables
float qw, qx, qy, qz;
float instant_roll, instant_pitch, instant_yaw;

// ===== Send CEVA SH-2 Tare Command using Adafruit API =====
void sendTare(bool persist) {
  sh2_Command_t cmd;
  cmd.command = SH2_CMD_TARE;
  cmd.persistent = persist ? 1 : 0;
  cmd.tareAxes = SH2_TARE_AXIS_ALL;

  bno08x.sendCommand(&cmd);
}

// ===== Compute orientation exactly like MQSpeed =====
void computeOrientation() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) return;

  qw = sensorValue.un.rotationVector.real;
  qx = sensorValue.un.rotationVector.i;
  qy = sensorValue.un.rotationVector.j;
  qz = sensorValue.un.rotationVector.k;

  // Mounting correction (your exact math)
  float qw_i = qw;
  float qx_i = qx;
  float qy_i = qy;
  float qz_i = qz;

  const float s = -0.70710678;
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
  delay(700);

  Serial.println("FSM300 Calibration + Orientation Interface");
  Serial.println("Commands: tare | persist | clear | orientation");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(500);

  if (!bno08x.begin_I2C()) {
    Serial.println("IMU not detected!");
    while (1) delay(10);
  }
  delay(300);

  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);

  Serial.println("IMU detected.");
  Serial.println("Ready.");
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "tare") {
    sendTare(false);
    Serial.println("OK: tare applied");
  }
  else if (cmd == "persist") {
    sendTare(true);
    Serial.println("OK: tare persisted");
  }
  else if (cmd == "clear") {
    sendTare(true);  // CEVA uses persistent tare with zero axes to clear
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
