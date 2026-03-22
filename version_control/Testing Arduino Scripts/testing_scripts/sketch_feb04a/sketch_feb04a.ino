#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.println("ESP32 will reboot every 5 seconds...");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  Serial.println("Rebooting...");
  ESP.restart();

}
