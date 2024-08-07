
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <RunningAverage.h>
#include <Wire.h>
#include "esp_attr.h"
#include <AS5600.h>
#include <string.h>
#define SERIAL_BUFFER_SIZE  1024
#define RXD2 16
#define TXD2 17

/////////////////////////STEERING ANGLE///////////////////////////////////////////////////

AS5600 as5600; 

////////////////////////////////////////Libraries initialization////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RunningAverage RPM_L_RA(1);
RunningAverage RPM_R_RA(1);
RunningAverage RPM_C_RA(1);
RunningAverage RPM_CRANK_RA(1);
RunningAverage RPM_SHAFT_RA(1);

/////////////////////////////////////////////////VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////SPEED/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long time_l = 999999999;            // Calculates the time between each sector of the wheel. Time of the magnets in the left wheel
unsigned long time_r = 999999999;            // Calculates the time between each sector of the wheel. Time of the magnets in the right wheel 
unsigned long time_c = 999999999;          // Calculates the time between each sector of the wheel. Time of the magnets in the centre wheel 
unsigned long time_cr = 999999999;           // Calculates the time between each sector of the wheel. Time of the magnets in the crank wheel 
unsigned long time_shaft = 999999999;            // Calculates the time between each sector of the wheel. Time of the magnets in the centre wheel 
unsigned long time_output;
unsigned long time_last_shaft; 
unsigned long time_last_l; 
unsigned long time_last_r; 
unsigned long time_last_c;
unsigned long time_last_crank;  
unsigned long prev_l;            //prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the left wheel 
unsigned long prev_r;           // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the right wheel
unsigned long prev_c;          // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the centre wheel
unsigned long prev_cr;         // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the crank wheel
unsigned long prev_shaft;         // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the crank wheel
unsigned long prev_output;
unsigned long debounce_l;
unsigned long debounce_r;
unsigned long debounce_c;
int buttonState_l;
int lastButtonState_l=LOW;
int buttonState_r;
int lastButtonState_r=LOW;
int buttonState_c;
int lastButtonState_c=LOW;
int buttonState_shaft;
int buttonState_crank;
int lastButtonState_shaft=LOW;
int lastButtonState_crank=LOW;
bool steering_available = true;
int wire_toggle_on = 1;
int debounce_timing = 300;
unsigned long debounce_shaft;
unsigned long debounce_crank;
bool debounce_l_trigger=0;
bool debounce_r_trigger=0;
bool debounce_c_trigger=0;
bool debounce_shaft_trigger=0;
bool debounce_crank_trigger=0;
bool run_sensor = true;
float RPM_L;                     // RPM of the left wheel
float RPM_R;                    // RPM of the right wheel 
float RPM_C;                   // RPM of the centre wheel
float RPM_CRANK;              // RPM of the crank wheel
float RPM_SHAFT;              // RPM of the gear shaft
float average_rpm;            // Average rpm of 3 wheels
float total_speed;           // total speed if the bike 
float some_constant_from_gear_ratio;  // placeholder for gearratio constant
float avg_rpm_l;
float avg_rpm_r;
float avg_rpm_c;
float avg_rpm_shaft;
int samples = 0; 
int x = 0; 
int priority_wheel_int = 1;
struct ints_struct{
  int left;
  int right;
  int center;
  int shaft;
  int crank;
  int priority;
};

unsigned long print_time=0;

////////////////////////////////////////////ZERO VARIABLES//////////////////////////////////////////////////////////
float old_value_shaft; 
float old_value_l;
float old_value_r;
float old_value_c; 
float old_value_crank; 

/////////////////////////////////////////////////STEERING ANGLE VARIABLES///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float steering_angle;
float steering_angle_max;
float steering_angle_min;
float steering_angle_center;

////////////////////////////////////////////////////////PINS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int hall_pin = 25;         // Pin relates to wheel left 
const int hall_pin2 = 34;        // Pin relates to wheel right 
const int hall_pin3 = 33;      // Pin relates to wheel centre 
const int hall_pin4 = 35;      // Pin relates to gear shaft 
const int hall_pin5 = 32;      //Pin relates to Crank
// 32 = crank



//////////////////////////////////////////////////// TWEAK variables///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float wheel_circumference = 1.440; //Circumference of wheel 
const int Magnet_Number = 8; // Number of magnets on the tone wheel 
const int print_frequency = 20; //milliseconds between serial.print 

////////////////////////////////////////////INTERRUPT FUNCTIONS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

////////////////////////////////////////TIMING BETWEEN EACH MAGNET////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////left wheel //////////////

void IRAM_ATTR wheel_left(){
  detachInterrupt(digitalPinToInterrupt(hall_pin));
  unsigned long now=micros();
  time_l = now - prev_l; 
  prev_l = now;
  debounce_l_trigger=1;
}

////////right wheel ////////////

void IRAM_ATTR wheel_right(){
  detachInterrupt(digitalPinToInterrupt(hall_pin2));
  unsigned long now=micros();
  time_r = now - prev_r;
  prev_r = now;
  debounce_r_trigger=1;
}

///////////centre wheel//////////

void IRAM_ATTR wheel_centre(){
  detachInterrupt(digitalPinToInterrupt(hall_pin3));
  unsigned long now=micros();
  time_c = now - prev_c;
  prev_c = now;
  debounce_c_trigger=1;

}

/////////crank wheel////////////

void IRAM_ATTR crank(){
  detachInterrupt(digitalPinToInterrupt(hall_pin5));
  unsigned long now=micros();
  time_cr = now - prev_cr; 
  prev_cr = now;
  debounce_crank_trigger=1;
}

/////////gear shaft////////////

void IRAM_ATTR gear_shaft(){
  detachInterrupt(digitalPinToInterrupt(hall_pin4));
  unsigned long now=micros();
  time_shaft = now - prev_shaft; 
  prev_shaft = now;
  debounce_shaft_trigger=1;
}

////////////////////////////////////////////////////SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.begin(115200);
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Wire.begin();
  //Serial2.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(hall_pin, INPUT_PULLUP);       //Pin initialization
  pinMode(hall_pin2, INPUT_PULLUP);      //Pin initialization
  pinMode(hall_pin3, INPUT_PULLUP);      //Pin initialization
  pinMode(hall_pin4, INPUT_PULLUP);      //Pin initialization
  pinMode(hall_pin5, INPUT_PULLUP);     //Pin initialization
  attachInterrupt(digitalPinToInterrupt(hall_pin), wheel_left, FALLING); //Interrupt initialization
  attachInterrupt(digitalPinToInterrupt(hall_pin2), wheel_right, FALLING); //Interrupt initialization
  attachInterrupt(digitalPinToInterrupt(hall_pin3), wheel_centre, FALLING); //Interrupt initialization
  attachInterrupt(digitalPinToInterrupt(hall_pin5), crank, FALLING); //Interrupt initialization
  attachInterrupt(digitalPinToInterrupt(hall_pin4), gear_shaft, FALLING); //Interrupt initialization
  RPM_L_RA.clear();               // Clearing cache of the averaging for the left wheel 
  RPM_R_RA.clear();               // Clearing cache of the averaging for the right wheel
  RPM_C_RA.clear();               // Clearing cache of the averaging for the centre wheel
  RPM_CRANK_RA.clear();           // Clearing cache of the averaging for the crank wheel
  RPM_SHAFT_RA.clear();           // Clearing cache of the averaging for the crank wheel



//  //////////////////////////////////////STEERING ANGLE SETUP/////////////////////////////////////////
//  ////////////////////////////////////////////////////////////////////////////////////

//  as5600.begin(4);

// }
}
///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

//if(run_sensor){
//  steering_availability = as5600.detectMagnet();
//  }
//  if(steering_availability && wire_toggle_on==1){
//    run_sensor = true;
//  }
//  else if(steering_availability && wire_toggle_on==0){
//    Wire.begin();
//    wire_toggle_on = 1;
//    run_sensor = true;
//  } else if(steering_availability && wire_toggle_on==1){
//    Wire.end();
//    wire_toggle_on = 0;
//    run_sensor = false;
//  } else if(steering_availability && wire_toggle_on==0){
//    run_sensor = false;
//  }
//  else{}

//steering_available = as5600.detectMagnet();
//if(steering_available){
//  wire_toggle_on = true;
//  Wire.begin();
//    steering_angle=as5600.rawAngle()/11.37777; //////conversion to degrees 
//   if(steering_angle>180){
//     steering_angle=-360+steering_angle;
//   }         //////setting negative angles
//   if(steering_angle>steering_angle_max){
//     steering_angle_max=steering_angle;
//   } ///capturing max value
//   
//   if(steering_angle<steering_angle_min){
//     steering_angle_min=steering_angle;
//   } ///capturing min value
//   steering_angle_center=(steering_angle_max+steering_angle_min)/2; ////calculating center
////} else if(!steering_available && wire_toggle_on){
//  wire_toggle_on = false;
//  Wire.end();
//} else{}

//if(run_sensor){
//    steering_angle=as5600.rawAngle()/11.37777; //////conversion to degrees
//  }
// 
//   if(steering_angle>180){
//     steering_angle=-360+steering_angle;
//   }         //////setting negative angles
//   if(steering_angle>steering_angle_max){
//     steering_angle_max=steering_angle;
//   } ///capturing max value
//   if(steering_angle<steering_angle_min){
//     steering_angle_min=steering_angle;
//   } ///capturing min value
//   steering_angle_center=(steering_angle_max+steering_angle_min)/2; ////calculating center


int reading_l=digitalRead(hall_pin);
  if(reading_l!=lastButtonState_l){
    debounce_l=micros();
  }
  
  if((micros()-debounce_l)>debounce_timing){

    if(reading_l!=buttonState_l){
      buttonState_l=reading_l;
      if(buttonState_l==HIGH){
        attachInterrupt(digitalPinToInterrupt(hall_pin), wheel_left, FALLING);
      }
    }
  }

  lastButtonState_l=reading_l;


int reading_r=digitalRead(hall_pin2);
  if(reading_r!=lastButtonState_r){
    debounce_r=micros();
  }
  
  if((micros()-debounce_r)>debounce_timing){

    if(reading_r!=buttonState_r){
      buttonState_r=reading_r;
      if(buttonState_r==HIGH){
        attachInterrupt(digitalPinToInterrupt(hall_pin2), wheel_right, FALLING);
      }
    }
  }

  lastButtonState_r=reading_r;




  int reading_c=digitalRead(hall_pin3);
  if(reading_c!=lastButtonState_c){
    debounce_c=micros();
  }
  
  if((micros()-debounce_c)>debounce_timing){

    if(reading_c!=buttonState_c){
      buttonState_c=reading_c;
      if(buttonState_c==HIGH){
        attachInterrupt(digitalPinToInterrupt(hall_pin3), wheel_centre, FALLING);
      }
    }
  }

  lastButtonState_c=reading_c;


int reading_shaft=digitalRead(hall_pin4);
  if(reading_shaft!=lastButtonState_shaft){
    debounce_shaft=micros();
  }
  
  if((micros()-debounce_shaft)>debounce_timing){

    if(reading_shaft!=buttonState_shaft){
      buttonState_shaft=reading_shaft;
      if(buttonState_shaft==HIGH){
        attachInterrupt(digitalPinToInterrupt(hall_pin4), gear_shaft, FALLING);
      }
    }
  }

  lastButtonState_shaft=reading_shaft;
  
int reading_crank=digitalRead(hall_pin5);
  if(reading_crank!=lastButtonState_crank){
    debounce_crank=micros();
  }
  
  if((micros()-debounce_crank)>debounce_timing){

    if(reading_crank!=buttonState_crank){
      buttonState_crank=reading_crank;
      if(buttonState_crank==HIGH){
       attachInterrupt(digitalPinToInterrupt(hall_pin5), crank, RISING);
      }
    }
  }
  lastButtonState_crank=reading_crank;
  
  if(time_shaft==999999999){RPM_SHAFT=0;} //RPM Shaft
  else{RPM_SHAFT = (60000000.00/(time_shaft*Magnet_Number));}

  if(time_l==999999999){RPM_L=0;} //RPM LEFT
  else{RPM_L = (60000000.00/(time_l*Magnet_Number));}

  if(time_r==999999999){RPM_R=0;} //RPM RIGHT
  else{RPM_R = (60000000.00/(time_r*Magnet_Number));}

  if(time_cr==999999999){RPM_CRANK=0;} //RPM CRANK
  else{RPM_CRANK = (60000000.00/(time_cr*5));}

  if(time_c==999999999){RPM_C=0;} //RPM CENTRE
  else{RPM_C = (60000000.00/(time_c*Magnet_Number));}

   if (millis()- time_last_shaft >=300 && RPM_SHAFT !=0){
        if (old_value_shaft==RPM_SHAFT){
            time_shaft = 999999999;
            old_value_shaft = 0;
            RPM_SHAFT_RA.clear();
        }
        old_value_shaft = RPM_SHAFT;
        time_last_shaft=millis();
   }

   if (millis()- time_last_l >=300 && RPM_L !=0){
        if (old_value_l==RPM_L){
            time_l = 999999999;
            old_value_l = 0;
            RPM_L_RA.clear();
        }
        old_value_l = RPM_L;
        time_last_l=millis();
   }

    if (millis()- time_last_r >=300 && RPM_R !=0){
        if (old_value_r==RPM_R){
            time_r = 999999999;
            old_value_r = 0;
            RPM_R_RA.clear();
        }
        old_value_r = RPM_R;
        time_last_r=millis();
   }

   if (millis()- time_last_c >=300 && RPM_C !=0){
        if (old_value_c==RPM_C){
            time_c = 999999999;
            old_value_c = 0;
            RPM_C_RA.clear();
        }
        old_value_c = RPM_C;
        time_last_c=millis();
   }

   if (millis()- time_last_crank >=300 && RPM_CRANK !=0){
        if (old_value_crank==RPM_CRANK){
            time_cr = 999999999;
            old_value_crank = 0;
            RPM_CRANK_RA.clear();
        }
        old_value_crank = RPM_CRANK;
        time_last_crank=millis();
   }
  


  //////////////////////////////////////////////Print to screen///////////////////////////////////////////////////////////////////////////
  // order in list should always go (left right center crank shaft)
unsigned long print_now=millis();
if(print_now-print_time>print_frequency){
//Serial.println("b," + String(RPM_C,2) + "," + String(RPM_L,2) + "," + String(RPM_R,2) + "," + String(RPM_SHAFT,2) + "," + String(RPM_CRANK,2) + "," + String(0));
Serial.print("b,");
Serial.print(RPM_C,2); 
Serial.print(","); 
Serial.print(RPM_L,2); 
Serial.print(",");
//Serial.flush();
Serial.print(RPM_R,2);  
Serial.print(",");
Serial.print(RPM_SHAFT,2);
Serial.print(",");
Serial.print(RPM_CRANK,2);
Serial.print(",");
Serial.println("0");
//Serial.flush();

print_time=print_now;
  }
}
