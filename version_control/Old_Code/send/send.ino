#define RXD2 16
#define TXD2 17
unsigned long timeout=0;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(23, INPUT_PULLUP);
}

void loop(){
  while (Serial2.available()) {
    Serial.print(char(Serial2.read()));
  }
  if(Serial.available() && !Serial2.available()){
    while(Serial.available()){
      Serial2.print(char(Serial.read()));
    } 
  Serial2.println();
  }
}
