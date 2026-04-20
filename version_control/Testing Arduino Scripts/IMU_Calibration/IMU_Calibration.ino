#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ===== I2C pins (match MQSpeed code) =====
#define SDA_PIN 21
#define SCL_PIN 22

// ===== SH-2 Tare Command Constants =====
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define COMMAND_TARE                0x03
#define TARE_AXIS_ALL               0x07
#define TARE_PERSIST                0x01

Adafruit_BNO08x imu(BNO08X_RST);
sh2_SensorValue_t sensorValue;

// Orientation variables
float qw, qx, qy, qz;
float instant_roll, instant_pitch, instant_yaw;

// ===== Send CEVA SH-2 Tare Command =====
void sendTareCommand(uint8_t axes, bool persist) {
  uint8_t packet[6];
  packet[0] = SHTP_REPORT_COMMAND_REQUEST;
  packet[1] = COMMAND_TARE;
  packet[2] = axes;
  packet[3] = persist ? TARE_PERSIST : 0x00;
  packet[4] = 0x00;
  packet[5] = 0x00;

  imu.sendPacket(packet, sizeof(packet));
}

// ===== Compute roll/pitch/yaw EXACTLY like Hatch_Sensors3.ino =====
void computeOrientation() {
  if (!imu.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) return;

  // Raw quaternion
  qw = sensorValue.un.rotationVector.real;
  qx = sensorValue.un.rotationVector.i;
  qy = sensorValue.un.rotationVector.j;
  qz = sensorValue.un.rotationVector.k;

  // ===== Apply your mounting correction quaternion =====
  // From Hatch_Sensors3.ino (s = -sqrt(2)/2)
  const float s = -0.70710678;
  float qw_r = 0.0f;
  float qx_r = s;
  float qy_r = s;
  float qz_r = 0.0f;

  float qw_m = qw_r*qw - qx_r*qx - qy_r*qy - qz_r*qz;
  float qx_m = qw_r*qx + qx_r*qw + qy_r*qz - qz_r*qy;
  float qy_m = qw_r*qy - qx_r*qz + qy_r*qw + qz_r*qx;
  float qz_m = qw_r*qz + qx_r*qy - qy_r*qx + qz_r*qw;

  qw = qw_m;
  qx = qx_m;
  qy = qy_m;
  qz = qz_m;

  // ===== Convert to Euler (same formulas as MQSpeed) =====
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

  if (!imu.begin_I2C()) {
    Serial.println("IMU not detected!");
    while (1) delay(10);
  }
  delay(300);

  imu.enableReport(SH2_ROTATION_VECTOR, 10000); // 10ms
  Serial.println("IMU detected.");
  Serial.println("Ready.");
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "tare") {
    sendTareCommand(TARE_AXIS_ALL, false);
    Serial.println("OK: tare applied");
  }

  else if (cmd == "persist") {
    sendTareCommand(TARE_AXIS_ALL, true);
    Serial.println("OK: tare persisted");
  }

  else if (cmd == "clear") {
    sendTareCommand(0x00, true);
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
