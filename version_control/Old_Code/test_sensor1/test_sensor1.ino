#include "AS5600.h"
#include "Wire.h"

AS5600 asAS5600;
float angle;

void setup() {
  asAS5600.begin(21);
  asAS5600.setDirection(AS5600_CLOCK_WISE);
  Serial.begin(115200);

}

void loop() {
  angle = asAS5600.getZPosition();
  angle = angle * 360/16384.0;

  if (angle > 180){
    angle = angle -360;
  }

  if (angle >= -7 && angle <= 7){
    Serial.print("Angle: ");
    Serial.println(angle);
  }
  delay(10);

}
