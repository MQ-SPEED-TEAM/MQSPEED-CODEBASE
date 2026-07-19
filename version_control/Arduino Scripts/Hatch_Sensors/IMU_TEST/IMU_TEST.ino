
// ESP32 firmware (C++) — reads all the sensors and sends data to the Pi via USB serial,
// receives commands from the Pi, and sends them to the transceiver via UART serial.

#include <Adafruit_GPS.h>
#include <RunningAverage.h>
#include <DFRobot_BME680_I2C.h>
#include <Wire.h>
#include <esp_attr.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#define SERIAL_BUFFER_SIZE 2048

#define SDA_PIN 21
#define SCL_PIN 22

#define BNO08X_INT -1
#define BNO08X_RST -1

String pi_data;
String command;

#define START_BYTE 0x02
#define STOP_BYTE 0x03
#define RX_BUFFER_SIZE 2048
#define RADIO_DELAY_ms 500

Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

Adafruit_GPS GPS(&Serial1);
#define GPSECHO false

RunningAverage battery_pi_read(1000);
RunningAverage battery_analog_read(1000);

// ======= CHANGE START: smaller averaging window for faster IMU response =======
RunningAverage averaged_roll_read(10);
RunningAverage averaged_pitch_read(10);
RunningAverage averaged_yaw_read(10);
// ======= CHANGE END =======

DFRobot_BME680_I2C bme(0x77);

unsigned long time_output;
unsigned long prev_output;
unsigned long led_time;
unsigned long led_prev_time = 0;
int led_state = LOW;
int x = 0;
float heading = 0;
float voltage_pi = 0;
float voltage_analog = 0;
float instant_roll;
float instant_pitch;
float instant_yaw;
float average_roll = 0;
float average_pitch = 0;
float average_yaw = 0;
float qw = 0, qx = 0, qy = 0, qz = 0;
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;
float gx = 0, gy = 0, gz = 0;

float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
unsigned long lastRead = 0;

const int set_A = 33;
const int set_B = 32;
const int aux = 25;
#define LED 2
#define pi_bat 27
#define backup_bat 26

const int print_frequency = 50;

// ======= CHANGE START: IMU watchdog and timed battery sampling variables =======
unsigned long lastIMUUpdate = 0;
unsigned long previousBatterySample = 0;
unsigned long lastIMURestartAttempt = 0;

const unsigned long IMU_TIMEOUT_MS = 1000;
const unsigned long IMU_RETRY_INTERVAL_MS = 2000;
const unsigned long BATTERY_SAMPLE_INTERVAL_MS = 10;
// ======= CHANGE END =======

// ======= CHANGE START: one function enables every required IMU report =======
bool enableIMUReports() {
  bool success = true;

  success &= bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);             // 50 Hz
  success &= bno08x.enableReport(SH2_ACCELEROMETER, 20000);               // 50 Hz
  success &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);        // 50 Hz
  success &= bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000);   // 20 Hz

  return success;
}
// ======= CHANGE END =======

// ======= CHANGE START: restart the IMU if reports stop arriving =======
bool restartIMU() {
  lastIMURestartAttempt = millis();

  // Reinitialise the I2C controller before reconnecting to the IMU.
  // This can recover the bus if it was left in a stuck state.
  Wire.end();
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  delay(10);

  if (!bno08x.begin_I2C()) {
    return false;
  }

  delay(100);

  if (!enableIMUReports()) {
    return false;
  }

  lastIMUUpdate = millis();
  return true;
}
// ======= CHANGE END =======

void setup() {
  // Keep the Pi connection at 115200 baud.
  Serial.begin(115200);
  delay(700);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);

  // ======= CHANGE START: prevent readStringUntil() blocking for one second =======
  Serial.setTimeout(10);
  // ======= CHANGE END =======

  Serial2.begin(115200, SERIAL_8N1, 17, 16);
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);

  // ======= CHANGE START: shorter UART read timeout =======
  Serial2.setTimeout(10);
  // ======= CHANGE END =======

  Serial1.begin(9600, SERIAL_8N1, 18, 19);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  pinMode(set_A, OUTPUT);
  pinMode(set_B, OUTPUT);
  pinMode(aux, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(backup_bat, INPUT);
  pinMode(pi_bat, INPUT);

  digitalWrite(set_A, LOW);
  digitalWrite(set_B, LOW);

  battery_pi_read.clear();
  battery_analog_read.clear();
  averaged_roll_read.clear();
  averaged_pitch_read.clear();
  averaged_yaw_read.clear();

  while (!Serial) {
    delay(10);
  }

  // ======= CHANGE START: start I2C before initializing either I2C sensor =======
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  delay(100);
  // ======= CHANGE END =======

  uint8_t rslt = bme.begin();
  if (rslt != 0) {
    Serial.println("BME680 initialization failed");
  }
  bme.startConvert();
  bme.update();
  delay(400);

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08X not detected!");
    while (1) {
      delay(100);
    }
  }
  delay(300);

  // ======= CHANGE START: replace four separate 100 Hz report calls =======
  if (!enableIMUReports()) {
    Serial.println("Failed to enable IMU reports");
  }
  lastIMUUpdate = millis();
  // ======= CHANGE END =======
}

float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) /
         (in_max - in_min) + out_min;
}

void loop() {
  const unsigned long now = millis();

  // ======= CHANGE START: sample batteries every 10 ms, not every loop pass =======
  if (now - previousBatterySample >= BATTERY_SAMPLE_INTERVAL_MS) {
    previousBatterySample = now;
    battery_pi_read.addValue(analogRead(pi_bat));
    battery_analog_read.addValue(analogRead(backup_bat));
  }
  // ======= CHANGE END =======

  // ======= CHANGE START: restore reports after an internal BNO08x reset =======
  if (bno08x.wasReset()) {
    enableIMUReports();
    lastIMUUpdate = millis();
  }
  // ======= CHANGE END =======

  // ======= CHANGE START: read up to eight queued IMU events each loop =======
  for (int eventNumber = 0; eventNumber < 8; eventNumber++) {
    if (!bno08x.getSensorEvent(&sensorValue)) {
      break;
    }

    lastIMUUpdate = millis();

    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR: {
        qw = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;

        const float qw_i = qw;
        const float qx_i = qx;
        const float qy_i = qy;
        const float qz_i = qz;

        const float s = -0.70710678f;
        const float qw_r = 0.0f;
        const float qx_r = s;
        const float qy_r = s;
        const float qz_r = 0.0f;

        qw = qw_r * qw_i - qx_r * qx_i - qy_r * qy_i - qz_r * qz_i;
        qx = qw_r * qx_i + qx_r * qw_i + qy_r * qz_i - qz_r * qy_i;
        qy = qw_r * qy_i - qx_r * qz_i + qy_r * qw_i + qz_r * qx_i;
        qz = qw_r * qz_i + qx_r * qy_i - qy_r * qx_i + qz_r * qw_i;

        instant_roll = atan2f(
          2.0f * (qw * qx + qy * qz),
          1.0f - 2.0f * (qx * qx + qy * qy)
        );

        float pitchInput = 2.0f * (qw * qy - qz * qx);
        pitchInput = constrain(pitchInput, -1.0f, 1.0f);
        instant_pitch = asinf(pitchInput);

        instant_yaw = atan2f(
          2.0f * (qw * qz + qx * qy),
          1.0f - 2.0f * (qy * qy + qz * qz)
        );

        instant_roll *= RAD_TO_DEG;
        instant_pitch *= RAD_TO_DEG;
        instant_yaw *= RAD_TO_DEG;

        averaged_roll_read.addValue(instant_roll);
        averaged_pitch_read.addValue(instant_pitch);
        averaged_yaw_read.addValue(instant_yaw);

        average_roll = averaged_roll_read.getAverage();
        average_pitch = averaged_pitch_read.getAverage();
        average_yaw = averaged_yaw_read.getAverage();
        break;
      }

      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mx = sensorValue.un.magneticField.x;
        my = sensorValue.un.magneticField.y;
        mz = sensorValue.un.magneticField.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        break;
    }
  }
  // ======= CHANGE END =======

  // ======= CHANGE START: restart IMU after one second without any new event =======
  if ((now - lastIMUUpdate >= IMU_TIMEOUT_MS) &&
      (now - lastIMURestartAttempt >= IMU_RETRY_INTERVAL_MS)) {
    restartIMU();
  }
  // ======= CHANGE END =======

  // ======= CHANGE START: process received serial data every loop, not every 50 ms =======
  if (Serial2.available() > 0) {
    command = Serial2.readStringUntil('\n');
    command.trim();

    if (command.equals("stop")) {
      Serial.println("stop");
    }
  }

  if (Serial.available() > 0 && Serial2.available() == 0) {
    pi_data = Serial.readStringUntil('\n');
    pi_data.trim();

    if (digitalRead(aux)) {
      Serial2.write(START_BYTE);
      Serial2.print(pi_data);
      Serial2.write(STOP_BYTE);
    }
  }
  // ======= CHANGE END =======

  if (now - lastRead > 1000) {
    bme.startConvert();
    bme.update();

    // ======= CHANGE START: remove incorrect comma operator from calculations =======
    Temperature = bme.readTemperature() / 100.0f;
    Humidity = bme.readHumidity() / 1000.0f;
    // ======= CHANGE END =======

    Pressure = bme.readPressure();
    lastRead = now;
  }

  voltage_pi = battery_pi_read.getAverage();
  voltage_pi = map_f(voltage_pi, 0.0f, 4095.0f, 0.0f, 3.3f);
  voltage_pi = (voltage_pi * (12.2f / 2.2f)) + 0.535f;

  voltage_analog = battery_analog_read.getAverage();
  voltage_analog = map_f(voltage_analog, 0.0f, 4095.0f, 0.0f, 3.3f);
  voltage_analog = (voltage_analog * (12.2f / 2.2f)) + 0.535f;

  if (voltage_pi <= 14.1f) {
    led_time = now;
    if (led_time - led_prev_time >= 1000) {
      led_state = (led_state == LOW) ? HIGH : LOW;
      digitalWrite(LED, led_state);
      led_prev_time = led_time;
    }
  } else {
    digitalWrite(LED, LOW);
  }

  GPS.read();
  if (GPS.newNMEAreceived()) {
    // ======= CHANGE START: bad GPS sentence no longer exits the entire loop =======
    char *sentence = GPS.lastNMEA();
    GPS.parse(sentence);
    // ======= CHANGE END =======
  }

  time_output = now - prev_output;
  if (time_output >= print_frequency) {
    prev_output = now;

    Serial.print("h");
    Serial.print(",");
    Serial.print(average_pitch, 2);
    Serial.print(",");
    Serial.print(average_roll, 2);
    Serial.print(",");
    Serial.print(average_yaw, 2);
    Serial.print(",");
    Serial.print(qw, 2);
    Serial.print(",");
    Serial.print(qx, 2);
    Serial.print(",");
    Serial.print(qy, 2);
    Serial.print(",");
    Serial.print(qz, 2);
    Serial.print(",");

    Serial.print(ax, 2); Serial.print(",");
    Serial.print(ay, 2); Serial.print(",");
    Serial.print(az, 2); Serial.print(",");
    Serial.print(mx, 2); Serial.print(",");
    Serial.print(my, 2); Serial.print(",");
    Serial.print(mz, 2); Serial.print(",");
    Serial.print(gx, 2); Serial.print(",");
    Serial.print(gy, 2); Serial.print(",");
    Serial.print(gz, 2); Serial.print(",");

    Serial.print(Temperature, 2);
    Serial.print(",");
    Serial.print(Humidity, 2);
    Serial.print(",");
    Serial.print(Pressure, 2);
    Serial.print(",");
    Serial.print(voltage_pi);
    Serial.print(",");
    Serial.print(voltage_analog);
    Serial.print(",");

    if (GPS.fix) {
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(",");
      Serial.print(GPS.longitude, 4);
      Serial.print(GPS.lon);
      Serial.print(",");
      Serial.print(GPS.speed * 1.852f);
      Serial.print(",");
      Serial.print(GPS.altitude);
      Serial.print(",");
      Serial.print((int)GPS.satellites);
    } else {
      Serial.print("0n,0e,0,0,0");
    }

    // ======= CHANGE START: no Serial.flush(), so loop does not wait for transmission =======
    Serial.print("0\n");
    // ======= CHANGE END =======
  }
}
