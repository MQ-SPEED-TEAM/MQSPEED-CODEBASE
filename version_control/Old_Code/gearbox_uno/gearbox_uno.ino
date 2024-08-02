/*////////////////////////////////////////////////////////////////////////////////////////////////
LIBRARIES
////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <TMCStepper_UTILITY.h>
#include <TMCStepper.h>

/*////////////////////////////////////////////////////////////////////////////////////////////////
PINS
////////////////////////////////////////////////////////////////////////////////////////////////*/

#define EN               2 //Enable pin
#define DIR_PIN          3 // Direction pin
#define STEP_PIN         4 // Step pin
#define ENDSTOP          5 //Stallguard pin
#define SW_RX            6 // RX pin
#define SW_TX            7 // TX pin
#define SWITCH1          8 //Gear up pin
#define SWITCH2          9 //Gear down pin
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2 (NC --> 0,0)

/*////////////////////////////////////////////////////////////////////////////////////////////////
TWEAK VALUES
////////////////////////////////////////////////////////////////////////////////////////////////*/

#define R_SENSE 0.11f     //Stepper Driver Rsense (Don't touch if driver is not changed)
int STALL_VALUE=20;       //Stallguard sensitivity (Endstop sensitivity)
int RMS=900;              //Stepper motor RMS current (mA)
int MS=8;                 //Microstepping mode (VALUES = 1,2,4,8,16,32,64,128) (Higher value is more precise but has less torque)
int SPEED=1500;           //Motor speed, decrease value to increase speed (if motor strugles to spin increase value)
bool DEBUGGING=false;     //Enable debugging mode
int DISTANCE=200;         //Number of steps per gear change
unsigned long print_time=0;

/*////////////////////////////////////////////////////////////////////////////////////////////////
VARIABLES
////////////////////////////////////////////////////////////////////////////////////////////////*/

int gear = 1;         //Current Gear
int switch1=0;        //Current switch1 state
int switch2=0;        //Current switch2 state

/*////////////////////////////////////////////////////////////////////////////////////////////////
OTHER
////////////////////////////////////////////////////////////////////////////////////////////////*/

  TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS); //Driver declaration for library

/*////////////////////////////////////////////////////////////////////////////////////////////////
FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////*/

void STEP(){      //Takes two steps

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(SPEED/MS);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(SPEED/MS);
  
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
------------------------------------------------------------------------------------------SETUP-------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

/*////////////////////////////////////////////////////////////////////////////////////////////////
PIN FUNCTION DECLARATION
////////////////////////////////////////////////////////////////////////////////////////////////*/

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  pinMode(ENDSTOP, INPUT);

/*////////////////////////////////////////////////////////////////////////////////////////////////
PIN DEFAULT STATE
////////////////////////////////////////////////////////////////////////////////////////////////*/

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN, LOW);

/*////////////////////////////////////////////////////////////////////////////////////////////////
SERIAL COMMUNICATION
////////////////////////////////////////////////////////////////////////////////////////////////*/
  
  driver.beginSerial(115200);    //STEPPER driver communication, don't touch
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nStart...");
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(RMS);        
  driver.microsteps(MS);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);


  if(DEBUGGING==true){

      while (!Serial);
  Serial.println("\nStart...");
  
  Serial.print(F("\nTesting connection..."));      //Test Connection
    uint8_t result = driver.test_connection();
    if (result) {
        Serial.println(F("failed!"));
        Serial.print(F("Likely cause: "));
        switch(result) {
            case 1: Serial.println(F("loose connection")); break;
            case 2: Serial.println(F("Likely cause: no power")); break;
        }
    }

    
  }
} //Close setup




/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
------------------------------------------------------------------------------------------LOOP--------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/


void loop() {

/*////////////////////////////////////////////////////////////////////////////////////////////////
DOUBLE BUTTON PRESS LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////*/

  if (digitalRead(SWITCH1)== HIGH || digitalRead(SWITCH2)== HIGH){

    delay (50); //Delay to check simultaneous press of buttons
/*////////////////////////////////////////////////////////////////////////////////////////////////
GEARS LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////*/
    if (digitalRead(SWITCH1)== HIGH && digitalRead(SWITCH2)== HIGH && switch1==0 && switch2==0){  ///////////////////GEAR RESET
    
      digitalWrite(DIR_PIN, LOW); //Reverse direction to probe at the first gear
      switch1=1; //Set single press state
      switch2=1; //Set single press state
  
      while (!digitalRead(ENDSTOP)){  /////////////////////////////////////////////////////////Takes a step until the ENDSTOP is triggered
        
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(SPEED/MS*4);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(SPEED/MS*4);
  
  }
    
     gear=1; //Reset gear
     digitalWrite(EN, HIGH); //Reset ENDSTOP (A few steps need to be done for ENDSTOP to untrigger, change gear
     delay(50);
     digitalWrite(EN, LOW);   
     
    } 

    else if(digitalRead(SWITCH1)==HIGH && gear<6 && switch1==0){ //////////////////////////////////GEAR UP
    
      digitalWrite(DIR_PIN, HIGH); //changes direction for gear up
  
      for (int i=0; i<(DISTANCE*MS); i++){ STEP(); } //Takes steps for gear up
    
      gear++; //Increase gear
      switch1=1; //Set single press state
    
    }

    else if (digitalRead(SWITCH2)==HIGH && gear>1 && switch2==0) { ////////////////////////////////GEAR DOWN
    
      digitalWrite(DIR_PIN, LOW); //changes direction for gear down
  
      for (int i=0; i<(DISTANCE*MS); i++){ STEP(); } //Takes steps for gear down
  
      gear--; //Decreases Gear
      switch2=1; //Set single press state

    }
  }

/*////////////////////////////////////////////////////////////////////////////////////////////////
SINGLE PRESS RESET
////////////////////////////////////////////////////////////////////////////////////////////////*/
  
if(digitalRead(SWITCH1)==LOW) {switch1=0;} //Reset single press state
if(digitalRead(SWITCH2)==LOW) {switch2=0;} //Reset single press state


if(millis()-print_time>10){
  Serial.println(gear);
  print_time=millis();
}

} //Close loop
