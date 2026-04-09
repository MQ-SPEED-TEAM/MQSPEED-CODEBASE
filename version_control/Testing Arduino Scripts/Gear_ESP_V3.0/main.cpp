

#include <Arduino.h>
#include <RunningAverage.h>
#include <esp_sleep.h>
#include <TMCStepper.h>

/*PINS
////////////////////////////////////////////////////////////////////////////////////////////////*/

#define EN 4             // Enable pin
#define DIR_PIN 18       // Direction pin
#define STEP_PIN 19      // Step pin
#define ENDSTOP 2        // Stallguard pin
#define SWITCH1 23       // Gear up pin
#define SWITCH2 22       // Gear down pin
#define SW_RX 16         // RX pin
#define SW_TX 17         // TX pin
#define BATTERY_PIN 15   // battery input
#define STEPS_PER_MM 400 // Example steps per mm for the motor
#define R_SENSE 0.11f    // Stepper Driver Rsense (Don't touch if driver is not changed)
#define DRIVER_ADDRESS 0
#define INDEX 21 // REPLACE PLS
#define FORWARD HIGH
#define BACKWARD LOW

/* VARIABLE DECLARATION
////////////////////////////////////////////////////////////////////////////////////////////////*/
volatile bool home_time = true;
int current_gear = 6;
int bt1_state;
int bt1_b4 = 0;
unsigned long bt1_last_debounce = 0;
bool bt1_pressed = false;

int bt2_state = 0;
int bt2_b4 = 0;
unsigned long bt2_last_debounce = 0;
bool bt2_pressed = false;

volatile int gear = 6;

unsigned long wait_time = 0;

unsigned long CHECK_TIME = 0;
bool CHECKED = 0;

int STALL_VALUE = 30; // Stallguard sensitivity (Endstop sensitivity)

volatile bool stall_on = false;
volatile long step_count;
volatile bool done = true;
long home_pos;
bool move_max_steps = true;

// names set for state
enum state
{
  G1,
  G2,
  G3,
  G4,
  G5,
  G6,
  homing
};

state current_state = homing;

// gear motion parameters

float gearDistanceUp[6] = {
    // steps delay
    6.0, // 0
    5.0, // 1
    4.0, // 2
    4.0, // 3
    4.0, // 4
    4.0  // 5
};

float gearDistanceDown[6] = {
    // steps delay
    // steps delay
    3.0, // 0
    3.0, // 1
    3.0, // 2
    3.0, // 3
    3.0, // 4
    3.0  // 5
};

float gearOvershoot[6] = {
    2.0,
    2.0, // Gear 1
    2.0, // Gear 2
    2.0, // Gear 3
    2.0, // Gear 4
    2.0  // Gear 5
};

uint32_t gearSpeed[6] = {
    100, // Gear 1
    100, // Gear 2
    100, // Gear 3
    100, // Gear 4
    100, // Gear 5
    100  // Gear 6
};

int32_t gearStepPos[6] = {0};   // absolute step position for each gear
int32_t gearMoveSteps[6] = {0}; // steps used to go FROM gear[i] TO gear[i+1]

TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);
RunningAverage battery_voltage(100);

void IRAM_ATTR index_interrupt()
{
  if (move_max_steps == true)
  {
    step_count++;
  }
  else
  {
    step_count--;
  }
}

void IRAM_ATTR on_STALL()
{ // must use IRAM_ATTR to store the function is stored in external flash memory
  stall_on = true;
}

void stepMotor(uint32_t steps, uint32_t stepDelay)
{
  // Step the motor the specified number of steps with the given delay, could improve with timer interrupts
  for (uint32_t i = 0; i < steps; i++)
  {

    if (stall_on)
    {
      driver.VACTUAL(0); // safety stop
      break;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void moveGearUp(uint32_t stepDelay, float distanceMM, float overshootMM)
{
  stall_on = false;
  // cover main distance + overshoot
  digitalWrite(DIR_PIN, HIGH);                                      // set direction foward
  uint32_t total_steps = (distanceMM + overshootMM) * STEPS_PER_MM; // calculate the reqired steps
  stepMotor(total_steps, stepDelay);                                // gearSpeed[gearIndex] -> defines the delay between steps

  // move back by overshoot
  digitalWrite(DIR_PIN, LOW); // set direction backward
  uint32_t overshoot_steps = overshootMM * STEPS_PER_MM;
  stepMotor(overshoot_steps, stepDelay);

  gear++;
  /* I need two movements
  - motion foward,
  - overshoot

  set direction foward
  main distance that needs to be covered -> gearDistanceUp[gearIndex]
  overshoot distance -> gearOvershootUp[gearIndex]
  move motor by the main distance + overshoot distance (foward) - all taken care of in this function
  move backwards
  move motor by overshoot distance (backwards)

  set current gear to next gear
*/
}
void moveGearDown(uint32_t stepDelay, float distanceMM, float overshootMM)
{
  stall_on = false;
  // cover main distance + overshoot
  digitalWrite(DIR_PIN, LOW);                                       // set direction foward
  uint32_t total_steps = (distanceMM + overshootMM) * STEPS_PER_MM; // calculate the reqired steps
  stepMotor(total_steps, stepDelay);

  // move back by overshoot
  digitalWrite(DIR_PIN, HIGH); // set direction backward
  uint32_t overshoot_steps = overshootMM * STEPS_PER_MM;
  stepMotor(overshoot_steps, stepDelay);

  gear--;
}

void going_home()
{
  if (home_time == true)
  {
    // Serial.print("going home");
    // Serial.print(", ");
    // Serial.print(current_gear);

    if (stall_on == false)
    {
      if (current_gear == 6)
      {
        moveGearDown(gearSpeed[5], gearDistanceDown[5], gearOvershoot[5]);
        current_gear = 5;
      }
      if (current_gear == 5)
      {
        moveGearDown(gearSpeed[4], gearDistanceDown[4], gearOvershoot[4]);
        current_gear = 4;
      }
      if (current_gear == 4)
      {
        moveGearDown(gearSpeed[3], gearDistanceDown[3], gearOvershoot[3]);
        current_gear = 3;
      }
      if (current_gear == 3)
      {
        moveGearDown(gearSpeed[2], gearDistanceDown[2], gearOvershoot[2]);
        current_gear = 2;
      }
      if (current_gear == 2)
      {
        moveGearDown(gearSpeed[1], gearDistanceDown[1], gearOvershoot[1]);
        current_gear = 1;
      }
      if (current_gear == 1)
      {
        moveGearDown(gearSpeed[0], gearDistanceDown[0], gearOvershoot[0]);
        current_gear = 0;
        // current_state = G1;
      }
      if (current_gear == 0)
      {
        driver.VACTUAL(4476); // free running 4476 steps per rev
      }
    }
    else
    {
      driver.VACTUAL(0);
      current_state = G1;
      gear = 1;
      home_time = false;
      stall_on = false;
    }
  }
}

// else {
//     driver.VACTUAL(0);
//     stall_on = false;
//     current_state = G1;
//     gear = 1;
//     home_time = false;
// }

void setup()
{
  Serial.begin(115200);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(INDEX, INPUT);

  pinMode(EN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(EN, LOW); // Enable the motor driver
  digitalWrite(DIR_PIN, LOW);
  pinMode(BATTERY_PIN, INPUT_PULLUP);

  pinMode(ENDSTOP, INPUT_PULLUP);

  battery_voltage.clear();

  Serial2.begin(115200, SERIAL_8N1, SW_RX, SW_TX);

  attachInterrupt(digitalPinToInterrupt(ENDSTOP), on_STALL, RISING); // set to falling as pin is pulled up
  attachInterrupt(digitalPinToInterrupt(INDEX), index_interrupt, RISING);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.index_step(true);
  driver.I_scale_analog(false);  // disables something to use the external current sense resistors
  driver.internal_Rsense(false); // prevets use of internal resistors, and use external ones
  driver.mstep_reg_select(true); // microstep resolution used by MSTEP register and not legacy pins
  driver.rms_current(500);
  driver.microsteps(16);
  driver.TCOOLTHRS(280); // wil need to change this probably - needs to be 1.2 x higher then the max TSTEP?
  driver.TPWMTHRS(0);    // Need to disable StealthChop PWM mode
  driver.semin(0);       // turns off coolstep which isn't requied in this case
  // driver.shaft(true);
  driver.en_spreadCycle(false); // disable spread cycle so that we can use stealthChop
  driver.pdn_disable(true);     // enable UART control
  driver.SGTHRS(20);            // higher should make it more sensitive, so increase it if its not sensing properly
  driver.VACTUAL(0);
}
void debouncing()
{
  bool bt1 = digitalRead(SWITCH1);
  bt1 = !bt1; // invert the reading because of pullup

  bool bt2 = digitalRead(SWITCH2);
  bt2 = !bt2; // invert the reading because of pullup

  if (bt1 != bt1_b4)
  {
    bt1_last_debounce = millis();
  }
  if ((millis() - bt1_last_debounce) > 50)
  {
    if (bt1 != bt1_state)
    {
      bt1_state = bt1;
      if (bt1_state == LOW)
      {
        bt1_pressed = true;
      }
    }
  }

  bt1_b4 = bt1; // saves the reading and then next time in the loop that will be the last button state
  // Serial.print("Raw button press:");
  // Serial.print(bt1);
  // Serial.print(" , stored button press:");
  // Serial.print(bt1_state);
  // Serial.print(" , previousbutton press:");
  // Serial.print(bt1_b4);
  //   Serial.print(" , previousbutton press:");
  // Serial.println(bt1_pressed);

  if (bt2 != bt2_b4)
  {
    bt2_last_debounce = millis();
  }
  if ((millis() - bt2_last_debounce) > 50)
  {
    if (bt2 != bt2_state)
    {
      bt2_state = bt2;
      if (bt2_state == LOW)
      {
        bt2_pressed = true;
      }
    }
  }
  bt2_b4 = bt2;
}

float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void battery_check()
{
  battery_voltage.addValue(analogRead(BATTERY_PIN));
  float VOLTAGE_BATTERY = battery_voltage.getAverage();
  VOLTAGE_BATTERY = map_f(VOLTAGE_BATTERY, 0.0, 1024.0, 0.0, 3.3) + 0.7;
  VOLTAGE_BATTERY = (VOLTAGE_BATTERY * (4));

  while (VOLTAGE_BATTERY < 14.4)
  {
    digitalWrite(EN, HIGH);
    esp_sleep_enable_timer_wakeup(10 * 1000000); // sleep for 10 seconds
    esp_deep_sleep_start();
  }

  if (millis() - CHECK_TIME > 2200)
  {
    CHECKED = 0;
  }
}

void loop()
{
  // bool move_max_steps = true;
  debouncing();
  static bool last_bt1 = false;
  static bool last_bt2 = false;
  static unsigned long wait_time = 0;
  static bool time_running = false;
  bool bt1_changed = (bt1_pressed != last_bt1);
  bool bt2_changed = (bt2_pressed != last_bt2);

  // digitalWrite(EN, LOW);
  // Serial.println(driver.SG_RESULT());
  Serial.print(stall_on);
  Serial.print(digitalRead(ENDSTOP));
  // driver.VACTUAL(4476); // free running 4476 steps per rev

  if ((bt1_changed || bt2_changed) && time_running == false)
  {
    wait_time = millis();
    time_running = true;
  }
  switch (current_state)
  {
  case G1:
    // code for gear 1

    if (time_running && (millis() - wait_time) > 500)
    { // wait 1 seconds before evaluating buttons
      if (bt1_pressed == true)
      {
        Serial.print("In gear 1 now");
        stall_on = false;
        moveGearUp(gearSpeed[0], gearDistanceUp[0], gearOvershoot[0]); // Gear 1 → 2
        Serial.print("motor moving");
        bt1_pressed = false;
        bt2_pressed = false;
        current_state = G2;
      }

      else if (bt2_pressed == true)
      {
        bt2_pressed = false;
      }
      else if ((bt1_pressed && bt2_pressed) == true)
      {
        current_state = homing;
        current_gear = 1;
      }
      time_running = false;
    }
    break;
  case G2:
    //  code for gear 2

    if (time_running && (millis() - wait_time) > 500)
    { // wait 1 seconds before evaluating buttons

      if ((bt1_pressed && bt2_pressed) == true)
      {
        bt1_pressed = false;
        bt2_pressed = false;
        stall_on = false;
        current_gear = 2;
        home_time = true;
        current_state = homing;
      }
      else if (bt1_pressed == true)
      {
        bt1_pressed = false;
        bt2_pressed = false;
        moveGearUp(gearSpeed[1], gearDistanceUp[1], gearOvershoot[1]); // Gear 2 → 3
        current_state = G3;
      }
      else if (bt2_pressed == true)
      {

        moveGearDown(gearSpeed[0], gearDistanceDown[0], gearOvershoot[0]); // Gear 2 → 1
        bt2_pressed = false;
        bt1_pressed = false;
        current_state = G1;
      }
      time_running = false;
    }
    break;
  case G3:
    //   code for gear 3
    if (time_running && (millis() - wait_time) > 500)
    { // wait 1 seconds before evaluating buttons
      if ((bt1_pressed && bt2_pressed) == true)
      {
        bt1_pressed = false;
        bt2_pressed = false;
        stall_on = false;
        home_time = true;
        current_gear = 3;
        current_state = homing;
      }
      else if (bt1_pressed == true)
      {

        moveGearUp(gearSpeed[2], gearDistanceUp[2], gearOvershoot[2]); // Gear 3 → 4
        bt1_pressed = false;
        bt2_pressed = false;
        current_state = G4;
      }
      else if (bt2_pressed == true)
      {
        moveGearDown(gearSpeed[1], gearDistanceDown[1], gearOvershoot[1]); // Gear 3 → 2
        bt2_pressed = false;
        bt1_pressed = false;
        current_state = G2;
      }
      time_running = false;
    }
    break;
  case G4:
    // code for gear 4
    if (time_running && (millis() - wait_time) > 500)
    { // wait 2 seconds before evaluating buttons
      if ((bt1_pressed && bt2_pressed) == true)
      {
        bt1_pressed = false;
        bt2_pressed = false;
        stall_on = false;
        home_time = true;
        current_gear = 4;
        current_state = homing;
      }
      else if (bt1_pressed == true)
      {
        moveGearUp(gearSpeed[3], gearDistanceUp[3], gearOvershoot[3]); // Gear 1 → 2
        bt1_pressed = false;
        bt2_pressed = false;
        current_state = G5;
      }
      else if (bt2_pressed == true)
      {

        moveGearDown(gearSpeed[2], gearDistanceDown[2], gearOvershoot[2]); // Gear 4 → 3
        bt2_pressed = false;
        bt1_pressed = false;
        current_state = G3;
      }
      time_running = false;
    }
    break;
  case G5:
    // code for gear 5
    if (time_running && (millis() - wait_time) > 500)
    { // wait 1 seconds before evaluating buttons

      if ((bt1_pressed && bt2_pressed) == true)
      {
        bt1_pressed = false;
        bt2_pressed = false;
        stall_on = false;
        home_time = true;
        current_gear = 5;
        current_state = homing;
      }
      else if (bt1_pressed == true)
      {

        moveGearUp(gearSpeed[4], gearDistanceUp[4], gearOvershoot[4]); // Gear 5 → 6
        bt1_pressed = false;
        bt2_pressed = false;
        current_state = G6;
      }
      else if (bt2_pressed == true)
      {

        moveGearDown(gearSpeed[3], gearDistanceDown[3], gearOvershoot[3]); // Gear 5 -> 4
        bt2_pressed = false;
        bt1_pressed = false;
        current_state = G4;
      }
      time_running = false;
    }

    break;
  case G6:
    // code for gear 6
    if (time_running && (millis() - wait_time) > 500)
    { // wait 2 seconds before evaluating buttons

      if ((bt1_pressed && bt2_pressed) == true)
      {
        home_time = true;
        bt1_pressed = false;
        bt2_pressed = false;
        stall_on = false;
        current_gear = 6;
        current_state = homing;
      }
      else if (bt2_pressed == true && !bt1_pressed)
      {
        moveGearDown(gearSpeed[4], gearDistanceDown[4], gearOvershoot[4]); // Gear 6 -> 5
        bt2_pressed = false;
        bt1_pressed = false;
        current_state = G5;
      }
      else
      {
        bt1_pressed = false;
        bt2_pressed = false;
      }

      time_running = false;
    }
    break;
  case homing:

    going_home();
    //
    // if (step_count > home_pos){
    //   driver.VACTUAL(4476); //free running 4476 steps per rev
    // } else {
    //   driver .VACTUAL(0);
    //   stall_on = false;
    //   current_state = G1;
    // }
    // If the value is averaging 260, then a SGTHRS value of 260/2 (130) will trigger the stall.

    // driver.VACTUAL(4476); // free running 4476 steps per rev
    //      Serial.print("SG: ");
    //  Serial.print(driver.SG_RESULT());

    // // Multiply this obtained value by 1.2 and set the variable "set_tcools" to this value
    // Serial.print("  TSTEP: ");
    // Serial.println(driver.TSTEP());

    break;
  }
  Serial.print("Current Gear: ");
  Serial.print(gear);
  Serial.print(", State: ");
  Serial.print(current_state);
  Serial.print(", button 1 state: ");
  Serial.print(bt1_pressed);
  Serial.print(", button 2 state: ");
  Serial.println(bt2_pressed);
  last_bt1 = bt1_pressed;
  last_bt2 = bt2_pressed;
}

// while homing do you want it to go all the way back to home or only when your holding buttons? or just when you've held the buttons down?
