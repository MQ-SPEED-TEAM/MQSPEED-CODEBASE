//
//    FILE: AS5600_demo_status.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include <Wire.h>
#include "esp_attr.h"
#include <AS5600.h>
#include <string.h>
#define SERIAL_BUFFER_SIZE  1024
#define RXD2 16
#define TXD2 17

float steering_angle;
float steering_angle_max;
float steering_angle_min;
float steering_angle_center;

//  Uncomment the line according to your sensor type
//AS5600L as5600;   //  use default Wire
// AS5600 as5600;   //  use default Wire
AS5600 as5600; 

void setup()
{
  Serial.begin(115200);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.begin(115200);
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Wire.begin();


  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
}


void loop()
{
  if(1){
    steering_angle=as5600.rawAngle()/11.37777; //////conversion to degrees
  }
 
   if(steering_angle>180){
     steering_angle=-360+steering_angle;
   }         //////setting negative angles
   if(steering_angle>steering_angle_max){
     steering_angle_max=steering_angle;
   } ///capturing max value
   if(steering_angle<steering_angle_min){
     steering_angle_min=steering_angle;
   } ///capturing min value
   steering_angle_center=(steering_angle_max+steering_angle_min)/2; ////calculating center
  Serial.println(steering_angle-142.4);
  Serial.print("STATUS:\t ");
  Serial.println(as5600.readStatus(), HEX);
  Serial.print("CONFIG:\t ");
  Serial.println(as5600.getConfigure(), HEX);
  Serial.print("  GAIN:\t ");
  Serial.println(as5600.readAGC(), HEX);
  Serial.print("MAGNET:\t ");
  Serial.println(as5600.readMagnitude(), HEX);
  Serial.print("DETECT:\t ");
  Serial.println(as5600.detectMagnet(), HEX);
  Serial.print("M HIGH:\t ");
  Serial.println(as5600.magnetTooStrong(), HEX);
  Serial.print("M  LOW:\t ");
  Serial.println(as5600.magnetTooWeak(), HEX);
  Serial.println();

  delay(1000);
}


//  -- END OF FILE --
