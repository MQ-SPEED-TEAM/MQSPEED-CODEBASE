/*////////////////////////////////////////////////////////////////////////////////////////////////
LIBRARIES
////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <TMCStepper_UTILITY.h>
#include <TMCStepper.h>
#include <avr/sleep.h>
#include <RunningAverage.h>

/*////////////////////////////////////////////////////////////////////////////////////////////////
PINS
////////////////////////////////////////////////////////////////////////////////////////////////*/

#define EN                9  //Enable pin
#define DIR_PIN           2  //Direction pin
#define STEP_PIN          3  //Step pin
#define ENDSTOP           10 //Stallguard pin
#define SW_RX             5  //RX pin
#define SW_TX             6  //TX pin
#define SWITCH1           12 //Gear up pin
#define SWITCH2           11 //Gear down pin
#define SPEED_PIN         4  //Speed input
#define BATTERY_PIN       A1 //battery input
#define DRIVER_ADDRESS    0b00 // TMC2209 Driver address according to MS1 and MS2 (NC --> 0,0)

/*////////////////////////////////////////////////////////////////////////////////////////////////
TWEAK VALUES
////////////////////////////////////////////////////////////////////////////////////////////////*/

#define R_SENSE 0.11f       //Stepper Driver Rsense (Don't touch if driver is not changed)
int STALL_VALUE=30;         //Stallguard sensitivity (Endstop sensitivity)
int RMS=1600;               //Stepper motor RMS current (mA)
int MS=16;                  //Microstepping mode (VALUES = 1,2,4,8,16,32,64,128) (Higher value is smoother and more precise but has less torque)
int SPEED=550;              //Motor speed, decrease value to increase speed (if motor strugles to spin increase value)
int START_SPEED=1000;       //acceleration starting speed
int ACCELERATION=20;        //Acceleration factor
int START_SPEED_TEMP=START_SPEED; //acceleration starting speed temp
bool DEBUGGING=true;       //Enable debugging mode
float DISTANCE=117;           //Number of steps per gear change for 1-4
int UPPER_DISTANCE = 99;    //Number of steps per gear change for 4-6
int OVERSHOOT=30;           //Overshoots the gear to move to the next one

RunningAverage battery_voltage(1000);

/*////////////////////////////////////////////////////////////////////////////////////////////////
VARIABLES
////////////////////////////////////////////////////////////////////////////////////////////////*/

int gear = 1;         //Current Gear
int switch1=0;        //Current switch1 state
int switch2=0;        //Current switch2 state
unsigned long CHECK_TIME = 0;
bool CHECKED = 0;
int BATTERY_PERCENTAGE = 0; 
unsigned long button_time=0;
unsigned long write_time=0;

/*////////////////////////////////////////////////////////////////////////////////////////////////
OTHER
////////////////////////////////////////////////////////////////////////////////////////////////*/

  TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS); //Driver declaration for library

/*////////////////////////////////////////////////////////////////////////////////////////////////
FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////*/

void STEP(float time){      //Takes two steps

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(time);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(time);
  
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
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(ENDSTOP, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT_PULLUP);
  pinMode(SPEED_PIN, INPUT_PULLUP);

  battery_voltage.clear();

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
  STEP(3000);
} //Close setup


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
------------------------------------------------------------------------------------------LOOP--------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void loop() {
  
  BATTERY_PERCENTAGE = (analogRead(BATTERY_PIN) -890) * 0.83;
  if (BATTERY_PERCENTAGE>100){ BATTERY_PERCENTAGE = 100;}
  if (BATTERY_PERCENTAGE < 0){ BATTERY_PERCENTAGE = 0;}

  if (BATTERY_PERCENTAGE<10 && CHECKED==0){
    CHECK_TIME = millis();
    CHECKED = 1;
  }
  battery_voltage.addValue(BATTERY_PERCENTAGE);
  float VOLTAGE_BATTERY  = battery_voltage.getAverage();
    VOLTAGE_BATTERY = map_f(VOLTAGE_BATTERY, 0.0, 4095.0, 0.0, 3.3);
    VOLTAGE_BATTERY = (VOLTAGE_BATTERY*(12.2/2.2)) + 0.535;

  if(analogRead(BATTERY_PIN)<900 && CHECKED == 1 && millis() - CHECK_TIME > 2000){
    digitalWrite(EN, HIGH);
    sleep_enable();
    sleep_cpu();
    exit(0);
  }
  
  if(millis()-CHECK_TIME>2200){CHECKED=0;}

  START_SPEED_TEMP=START_SPEED;

/*////////////////////////////////////////////////////////////////////////////////////////////////
DOUBLE BUTTON PRESS LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////*/
  if(digitalRead(SWITCH1)==HIGH || digitalRead(SWITCH2)==HIGH){

    delay(50);
/*////////////////////////////////////////////////////////////////////////////////////////////////
GEARS LOGIC
////////////////////////////////////////////////////////////////////////////////////////////////*/
      if (digitalRead(SWITCH1)== HIGH && digitalRead(SWITCH2)== HIGH && switch1==0 && switch2==0){  ///////////////////GEAR RESET
        digitalWrite(DIR_PIN, HIGH); //Reverse direction to probe at the first gear
        switch1=1; //Set single press state
        switch2=1; //Set single press state
        digitalWrite(EN, LOW); //Reset ENDSTOP (A few steps need to be done for ENDSTOP to untrigger, change gear
        delay(1);
        digitalWrite(EN, LOW);
        for (uint16_t i = 200; i > 0; i--) {
          STEP(4000/MS);
        } 
        digitalWrite(DIR_PIN, LOW);
    
        while (!digitalRead(ENDSTOP)){  /////////////////////////////////////////////////////////Takes a step until the ENDSTOP is triggered
          
          STEP(3000/MS);
    
        }
      
        gear=1; //Reset gear
      } 




      else if(digitalRead(SWITCH1)==HIGH && gear<6 && switch1==0){ //////////////////////////////////GEAR UP
        digitalWrite(DIR_PIN, HIGH); //changes direction for gear up

        for(;START_SPEED_TEMP>0; START_SPEED_TEMP=START_SPEED_TEMP-ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }
        for (int i=0; i<((DISTANCE+OVERSHOOT)*MS-(2*START_SPEED/ACCELERATION)); i++){
          STEP(SPEED/MS); 
        } //Takes steps for gear up
        for(;START_SPEED_TEMP<START_SPEED; START_SPEED_TEMP=START_SPEED_TEMP+ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }

        digitalWrite(DIR_PIN, LOW); //changes direction for gear up

        for(;START_SPEED_TEMP>0; START_SPEED_TEMP=START_SPEED_TEMP-ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }
        for (int i=0; i<(OVERSHOOT*MS-(2*START_SPEED/ACCELERATION)); i++){
          STEP(SPEED/MS); 
        } //Takes steps for gear up
        for(;START_SPEED_TEMP<START_SPEED; START_SPEED_TEMP=START_SPEED_TEMP+ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }

        gear++; //Increase gear
        switch1=1; //Set single press state
      
      }

      else if (digitalRead(SWITCH2)==HIGH && gear>1 && switch2==0) { ////////////////////////////////GEAR DOWN
        digitalWrite(DIR_PIN, LOW); //changes direction for gear down

        for(;START_SPEED_TEMP>0; START_SPEED_TEMP=START_SPEED_TEMP-ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }
        for (int i=0; i<(DISTANCE*MS-(2*START_SPEED/ACCELERATION)); i++){
          STEP(SPEED/MS); 
        } //Takes steps for gear up
        for(;START_SPEED_TEMP<START_SPEED; START_SPEED_TEMP=START_SPEED_TEMP+ACCELERATION){
          STEP((SPEED+START_SPEED_TEMP)/MS);
        }
    
        gear--; //Decreases Gear
        switch2=1; //Set single press state

      }
  }

/*////////////////////////////////////////////////////////////////////////////////////////////////
SINGLE PRESS RESET
////////////////////////////////////////////////////////////////////////////////////////////////*/
  
if(digitalRead(SWITCH1)==LOW) {switch1=0;} //Reset single press state
if(digitalRead(SWITCH2)==LOW) {switch2=0;} //Reset single press state

if(millis()-write_time>=5){
  write_time=millis();
  Serial.print("g");
  Serial.print(","); 
  Serial.print(gear); 
  Serial.print(","); 
  Serial.print(analogRead(BATTERY_PIN));
  Serial.print("\n");
  Serial.flush();
//  Serial.println(millis()-button_time);
}

} //Close loop
