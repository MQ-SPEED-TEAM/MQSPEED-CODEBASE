// ESP32 firmware (C++) — reads all the sensors and sends data to the Pi via USB serial, also receives commands from the Pi and sends them to the transceiver via UART serial. The ESP32 is also responsible for blinking an LED when the Pi battery is low, and reading the backup screen battery voltage.
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 3 sensors: GPS, BME680(environment), FSM30x(IMU)
#include <Adafruit_GPS.h> // S1: GPS library, actual gps is based on a mtk8889 chipset, designed on adafruit ultimate gps breakout board. Serial communication.
#include <RunningAverage.h> // Rolling average calculations
#include <DFRobot_BME680_I2C.h> // S2: An enviromental sensor that reads temperature, humidity and pressure. I2C
#include <Wire.h> // I2C communication library
#include <esp_attr.h> // ESP32 memory attributes
#include <Adafruit_Sensor.h> // Base sensor library
#include <Adafruit_BNO08x.h> // S3: IMU library name is BNO08x but actual sensor is FSM30x


#define SERIAL_BUFFER_SIZE  2048 // Size of serial read/write buffers

// ===== I2C PINS =====
#define SDA_PIN 21 // I2C data pin (serial bidirectional data line)
#define SCL_PIN 22 // I2C clock pin

// Optional pins (can be -1 if unused)
#define BNO08X_INT -1 // Interrupt pin for BNO08X (not used in this code, set to -1)
#define BNO08X_RST -1 // Reset pin for BNO08X (not used in this code, set to -1)


String pi_data; // Stores data received from Pi {white because declared but never used}
String command; // Stores commands received from transceiver
#define GPSECHO true // overidden below?


#define START_BYTE 0x02
#define STOP_BYTE 0x03
#define RX_BUFFER_SIZE 2048
#define RADIO_DELAY_ms 500

////libraries stuff // Create IMU object with reset pin
Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue; // Stores latest IMU sensor reading

Adafruit_GPS GPS(&Serial1); // Create GPS object on Serial1 port
#define GPSECHO false // Disable echoing raw GPS data to serial


////////////////////////////////////////Libraries initialization////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RunningAverage battery_pi_read(1000); // 1000 sample average of battery voltage readings, last 10 seconds (1000 readings at 10ms intervals)
RunningAverage battery_analog_read(1000); // backup screen battery voltage
RunningAverage averaged_roll_read(100);
RunningAverage averaged_pitch_read(100);
RunningAverage averaged_yaw_read(100);

//////////////////////////////////////////////I2C sensors///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DFRobot_BME680_I2C bme(0x77);  //I2C BME680 ID/

/////////////////////////////////////////////////VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
float qw, qx, qy, qz; // IMU quaternion variables
float ax, ay, az; // IMU accelerometer variables (m/s²)
float mx, my, mz; // IMU magnetometer variables (uTesla)
float gx, gy, gz; // IMU gyroscope/angular velocity variables (rad/s)
bool imuConnected = false;



///////////////////////////////////////////////////AIR QUALITY//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// BME680 environmental variables
float Temperature;          // Temperature value 
float Humidity;            // Humidity value 
float Pressure;           // Pressure value
unsigned long lastRead=0; // Timestamp of last BME680 reading

////////////////////////////////////////////////////////PINS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int set_A = 33;           //transciever m1
const int set_B = 32;           //transciever m0
const int aux = 25;             //transciever aux
#define LED 2                   //ESP led pin
#define pi_bat 27               // Analog pin monitors Pi battery 
#define backup_bat 26           // Analog pin monitors backup screen battery

//////////////////////////////////////////////////// TWEAK variables///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int print_frequency = 50; //milliseconds between Prints

////////////////////////////////////////////////////SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); //Setup USB communication port
  delay(700);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE); 
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  
  Serial2.begin(115200, SERIAL_8N1, 17, 16); // Setup UART serial communication port for radio transceiver on pins 16 (RX) and 17 (TX)
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);

  Serial1.begin(9600, SERIAL_8N1, 18, 19); //Setup GPS serial communication port on pins 18, 19

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request RMC+GGA sentences from GPS
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // Set GPS update rate to 10Hz
  
  pinMode(set_A, OUTPUT); // Configure transceiver and sensor pins
  pinMode(set_B, OUTPUT);
  pinMode(aux, INPUT);
  pinMode(LED,OUTPUT);
  
  pinMode(backup_bat, INPUT);
  pinMode(pi_bat, INPUT);
  
  digitalWrite(set_A, LOW); // Set transceiver to default mode
  digitalWrite(set_B, LOW);
  
  battery_pi_read.clear(); // Clear averaging buffers
  battery_analog_read.clear();
  averaged_roll_read.clear();
  averaged_pitch_read.clear();
  averaged_yaw_read.clear();

  
///////////////////////////////////////////TEMPERATURE SETUP/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


   uint8_t rslt = 1;
   //while(!Serial); 
   delay(1000);
   rslt = bme.begin(); // Initialize BME680 sensor
   //Serial.println("BME WORKING");
   bme.startConvert(); // Begin sensor conversion
   bme.update();
   delay(400);
  
//////////IMU SETUP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(SDA_PIN, SCL_PIN); // Start I2C on specified SDA and SCL pins
  Wire.setClock(400000); // Set I2C speed to 100kHz - BNO08X supports 400kHz
  delay(500);

  imuConnected = bno08x.begin_I2C();

  if (!imuConnected) {
   
      Serial.println("WARNING: BNO08X not detected. Continuing without IMU.");
      
}   else {
      delay(300);

      if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20000)) {
          Serial.println("Failed to enable rotation vector");
      }

      if (!bno08x.enableReport(SH2_ACCELEROMETER, 20000)) {
          Serial.println("Failed to enable accelerometer");
      }

      if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000)) {
          Serial.println("Failed to enable magnetometer");
      }

      if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) {
          Serial.println("Failed to enable gyroscope");
      }
  }

  //delay(300);
  
  // BNO08X reports many types of sensor data (quaternions, Euler angles, accelerometer, gyroscope, magnetometer, etc.) this enables specific reports we want to receive from the IMU. Read in loop().
 //if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20000)) {
   // Serial.println("Failed to enable rotation vector");
//}




///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
float map_f(float x, float in_min, float in_max, float out_min, float out_max) {/////////WTF is this?
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // A function to map a float from one range to another, similar to Arduino's built-in map() but for floats. Used for converting ADC readings to voltages.
}





void loop() {
   battery_pi_read.addValue(analogRead(pi_bat));
   battery_analog_read.addValue(analogRead(backup_bat));

    // Calculate roll pitch yaw



    

if(imuConnected) {
   while (bno08x.getSensorEvent(&sensorValue))  {
     
  
    
    
    
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      
      
      qw = sensorValue.un.rotationVector.real;
      qx = sensorValue.un.rotationVector.i;
      qy = sensorValue.un.rotationVector.j;
      qz = sensorValue.un.rotationVector.k;

      // Quaternion from IMU
      float qw_i = qw;
      float qx_i = qx;
      float qy_i = qy;
      float qz_i = qz;

      // Rotation quaternion for mounting orientation
      const float s = -0.70710678; // -sqrt(2)/2
      float qw_r = 0.0f;
      float qx_r = s;
      float qy_r = s;
      float qz_r = 0.0f;

      // Apply mounting correction
      float qw_m = qw_r*qw_i - qx_r*qx_i - qy_r*qy_i - qz_r*qz_i;
      float qx_m = qw_r*qx_i + qx_r*qw_i + qy_r*qz_i - qz_r*qy_i;
      float qy_m = qw_r*qy_i - qx_r*qz_i + qy_r*qw_i + qz_r*qx_i;
      float qz_m = qw_r*qz_i + qx_r*qy_i - qy_r*qx_i + qz_r*qw_i;

      // Replace IMU quaternion with corrected one
      qw = qw_m;
      qx = qx_m;
      qy = qy_m;
      qz = qz_m;

      // Convert to Euler angles (degrees)
      instant_roll  = atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy));
      instant_pitch = asin(2.0 * (qw * qy - qz * qx));
      instant_yaw   = atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
      
      instant_roll  *= 180.0 / PI; //Rotation about X (forward+ and backwards- axis)
      instant_pitch *= 180.0 / PI; //Rotation about Y (left+ and right- axis)
      instant_yaw   *= 180.0 / PI; //Rotation about Z (up+ and down- axis)

      averaged_roll_read.addValue(instant_roll);
      averaged_pitch_read.addValue(instant_pitch);
      averaged_yaw_read.addValue(instant_yaw);

      average_roll = averaged_roll_read.getAverage();
      average_pitch = averaged_pitch_read.getAverage();
      average_yaw = averaged_yaw_read.getAverage();

      }
      if (sensorValue.sensorId == SH2_ACCELEROMETER) { //*?
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
    }
    if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
        mx = sensorValue.un.magneticField.x;
        my = sensorValue.un.magneticField.y;
        mz = sensorValue.un.magneticField.z;
    }
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
    }
    }

}




        

    

 

    


    
   
 
    // Calculate BME data
     if(millis()-lastRead>1000){ // Read BME680 data every 1 second/1000ms
       bme.startConvert();
       bme.update();
       Temperature = bme.readTemperature() / 100, 2; // Degrees Celsius
       Humidity = bme.readHumidity() / 1000, 2; // % Relative humidity
       Pressure = bme.readPressure(); // Pressure Pa
       lastRead = millis(); // Timestamp for last BME680 reading
     }

    
   /// CALCULATE BATTERY VOLTAGES ///
    voltage_pi  = battery_pi_read.getAverage(); // Get average Pi battery reading from rolling average buffer
    voltage_pi = map_f(voltage_pi, 0.0, 4095.0, 0.0, 3.3); // Convert ADC reading (0-4095) to voltage (0-3.3V) based on ESP32 ADC characteristics
    voltage_pi = (voltage_pi*(12.2/2.2)) + 0.535; // Scale voltage reading based on voltage divider resistors (12.2k and 2.2k resistors) and add offset estimate for the PI battery voltage. 

    voltage_analog  = battery_analog_read.getAverage();
    voltage_analog = map_f(voltage_analog, 0.0, 4095.0, 0.0, 3.3); // Convert ADC reading to voltage for backup battery
    voltage_analog = (voltage_analog*(12.2/2.2)) + 0.535; 

  if(voltage_pi <= 14.1){ // If Pi battery voltage is below 14.1V, blink LED to indicate low battery
    led_time = millis();
    if (led_time - led_prev_time >= 1000) { // Blink LED every 1 second (1000ms)
      // if the LED is off turn it on and vice-versa:
      led_state = (led_state == LOW) ? HIGH : LOW; // Toggle LED state?

      digitalWrite(LED, led_state); // Update LED state to reflect low battery warning
  
      led_prev_time = led_time; // Update timestamp for last LED toggle
    }
  } 
  else {
    digitalWrite(LED, LOW); // If battery is not low, LED is off
  }

   // Parse GPS data
  char c = GPS.read(); // Read a character from GPS
  if (GPS.newNMEAreceived()) { // Check if a new NMEA sentence has been received from the GPS
    GPS.lastNMEA(); // clears the newNMEAreceived() new sentence flag to false
    if (!GPS.parse(GPS.lastNMEA())) // Try to parse the latest NMEA sentence, if parsing fails, skip to next loop iteration and wait for another sentence. This can happen if the GPS data is corrupted or incomplete.
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  //////////////////////////////////////////////Print to screen///////////////////////////////////////////////////////////////////////////
  time_output = millis()-prev_output;
   if (time_output >= print_frequency){
   prev_output = millis();
   Serial.print("h");
   Serial.print(",");
   Serial.print(average_pitch, 2); //(roll and pitch switched values)
   Serial.print(",");
   Serial.print(average_roll, 2); //investigae and switch back after
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

   Serial.print(ax, 2); Serial.print(","); // print accelerometer, magnetometer, and gyroscope readings
   Serial.print(ay, 2); Serial.print(",");
   Serial.print(az, 2); Serial.print(",");
   Serial.print(mx, 2); Serial.print(",");
   Serial.print(my, 2); Serial.print(",");
   Serial.print(mz, 2); Serial.print(",");
   Serial.print(gx, 2); Serial.print(",");
   Serial.print(gy, 2); Serial.print(",");
   Serial.print(gz, 2); Serial.print(",");
   
   //print environment
   Serial.print(Temperature, 2);
   Serial.print(",");
   Serial.print(Humidity, 2);
   Serial.print(",");
   Serial.print(Pressure, 2);
   Serial.print(",");
   Serial.print(voltage_pi);// this gives the reading in voltages 
   Serial.print(",");
   Serial.print(voltage_analog);// this gives the reading in voltages 
   Serial.print(",");

   //GPS write
   if(GPS.fix){
      Serial.print(GPS.latitude, 4); 
      Serial.print(GPS.lat);
      Serial.print(",");
      Serial.print(GPS.longitude, 4); 
      Serial.print(GPS.lon);
      Serial.print(",");
      Serial.print(GPS.speed*1.852);
      Serial.print(",");
      Serial.print(GPS.altitude);
      Serial.print(",");
      Serial.print((int)GPS.satellites);
   }
   else{Serial.print("0n,0e,0,0,0");}
   
   Serial.print("0\n");
   


   /// Receieve from pi and send to transceiver
   if(Serial2.available()>0){
      command = Serial2.readStringUntil('\n');
      command.trim();
      if (command.equals("stop")) {
        Serial.print("stop");
        Serial.println();
        Serial.flush();
        }
      }
    if (Serial.available()>0 && !Serial2.available()>0) {
      pi_data = Serial.readStringUntil('\n');
      pi_data.trim();
      if (digitalRead(aux)) {
        Serial2.write(0x02);          // STX
        Serial2.print(pi_data);       // Payload
        Serial2.write(0x03);          // ETX
        Serial2.flush();
      }
    }
  }
}
