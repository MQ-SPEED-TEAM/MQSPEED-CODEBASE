#include "AS5600.h"
#include "Wire.h"

AS5600 as5600;   //  use default Wire


void setup()
{
  Serial.begin(115200);

  //  ESP32
  //  as5600.begin(14,15);
  //  AVR
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  delay(1000);
}


void loop()
{
  //  Serial.print(millis());
  //  Serial.print("\t");
  Serial.print(as5600.readAngle());
  Serial.print("\t");
  Serial.println(as5600.rawAngle());
  //  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

  delay(1000);
}
