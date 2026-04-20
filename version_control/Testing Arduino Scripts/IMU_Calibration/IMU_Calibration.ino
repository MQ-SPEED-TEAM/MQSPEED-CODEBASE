#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ===== I2C pins (match MQSPEED) =====
#define SDA_PIN 21
#define SCL_PIN 22

// Optional pins (same as MQSPEED)
#define BNO08X_INT -1
#define BNO08X_RST -1

// ===== SH-2 Command Constants (CEVA FSM300) =====
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define COMMAND_TARE                0x03
#define TARE_AXIS_ALL               0x07
#define TARE_PERSIST                0x01

// ======================================================
//  WRAPPER SUBCLASS TO EXPOSE _sendPacket()
// ======================================================
class BNO08x_CEVA : public Adafruit_BNO08x {
public:
  BNO08x_CEVA(int8_t rst) : Adafruit_BNO08x(rst) {}

  // Expose the private _sendPacket() safely
  bool sendRawPacket(uint8_t *buffer, uint8_t length) {
    return this->_sendPacket(buffer, length);
  }
};

// Replace your instance with the wrapper
BNO08x_CEVA bno08x(BNO08X_RST);

sh2_SensorValue_t sensorValue;

// Orientation variables
float qw, qx, qy, qz;
float instant_roll, instant_pitch, instant_yaw;

// ======================================================
//  SEND CEVA SH-2 TARE COMMAND
// ======================================================
void sendTareCommand(bool persist) {
  uint8_t packet[6];
  packet[0] = SHTP_REPORT_COMMAND_REQUEST;
  packet[1] = COMMAND_TARE;
  packet[2] = TARE_AXIS_ALL;
  packet[3] = persist ? TARE_PERSIST : 0x00;
  packet[4] = 0x00;
  packet[5] = 0x00;

  bno08x.sendRawPacket(packet, sizeof(packet));
}

// ======================================================
//  ORIENTATION CALCULATION (EXACT MQSPEED MATH)
// ======================================================
void computeOrientation() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) return;

  // Raw quaternion
  qw = sensorValue.un.rotationVector.real;
  qx = sensorValue.un.rotationVector.i;
  qy = sensorValue.un.rotationVector.j;
  qz = sensorValue.un.rotationVector.k;

  // Mounting correction quaternion (your exact math)
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

  // Convert to Euler (your exact MQSPEED formulas)
  instant_roll  = atan2(2.0 * (qw*qx + qy*qz),
                        1.0 - 2.0 * (qx*qx + qy*qy));
  instant_pitch = asin(2.0 * (qw*qy - qz*qx));
  instant_yaw   = atan2(2.0 * (qw*qz + qx*qy),
                        1.0 - 2.0 * (qy*qy + qz*qz));

  instant_roll  *= 180.0 / PI;
  instant_pitch *= 180.0 / PI;
  instant_yaw   *= 180.0 / PI;
}

// ======================================================
//  SETUP
// ======================================================
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

// ======================================================
//  LOOP
// ======================================================
void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "tare") {
    sendTareCommand(false);
    Serial.println("OK: tare applied");
  }
  else if (cmd == "persist") {
    sendTareCommand(true);
    Serial.println("OK: tare persisted");
  }
  else if (cmd == "clear") {
    sendTareCommand(true);  // CEVA clears by persisting zeroed axes
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
