bool blink = false;
void setup() {
  // put your setup code here, to run once:
 pinMode(2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  digitalWrite(2, blink);
  blink = !blink;
}
