/********************************************************************************************
 *  ESP32 ELECTRONIC DERAILLEUR CONTROLLER — FULL TEACHING-MODE VERSION
 *
 *  This file explains the entire derailleur controller in a way that someone with *zero*
 *  prior exposure can understand. It includes:
 *
 *    • How the derailleur is modeled in millimeters
 *    • How gear positions are computed
 *    • How open-loop shifting works (overshoot → settle → snap-back)
 *    • How closed-loop correction works using Raspberry Pi feedback
 *    • How homing works using StallGuard
 *    • How the state machine processes button inputs
 *    • How battery protection works
 *    • The Pi-zero-RPM fix (critical)
 *
 *  The goal is clarity, mechanical intuition, and maintainability.
 ********************************************************************************************/

#include <Arduino.h>
#include <RunningAverage.h>
#include <esp_sleep.h>
#include <TMCStepper.h>
#include <math.h>

/********************************************************************************************
 *  PIN DEFINITIONS
 ********************************************************************************************/

#define EN           4
#define DIR_PIN      18
#define STEP_PIN     19
#define ENDSTOP      2
#define SWITCH1      23
#define SWITCH2      22
#define SW_RX        16
#define SW_TX        17
#define BATTERY_PIN  15
#define INDEX        21

/********************************************************************************************
 *  MECHANICAL CONSTANTS
 ********************************************************************************************/

#define STEPS_PER_MM 400
#define R_SENSE      0.11f
#define DRIVER_ADDRESS 0

#define FORWARD  HIGH
#define BACKWARD LOW

/********************************************************************************************
 *  GLOBAL STATE VARIABLES
 ********************************************************************************************/

volatile bool home_time = true;

int bt1_state;
int bt1_b4 = 0;
unsigned long bt1_last_debounce = 0;
bool bt1_pressed = false;

int bt2_state = 0;
int bt2_b4 = 0;
unsigned long bt2_last_debounce = 0;
bool bt2_pressed = false;

volatile bool stall_on = false;
volatile long step_count = 0;
volatile bool move_max_steps = true;

int gear          = 1;
int expected_gear = 1;
int actual_gear   = 1;

/********************************************************************************************
 *  GEAR STATE MACHINE
 ********************************************************************************************/

enum state { G1, G2, G3, G4, G5, G6, homing };
state current_state = homing;

/********************************************************************************************
 *  GEAR MOTION PARAMETERS
 ********************************************************************************************/

float gearDistanceUp[7]   = {0,3,3,3,3,3,0};
float gearDistanceDown[7] = {0,0,3,3,3,3,3};
float gearOvershoot[7]    = {0,2,2,2,2,2,2};
uint32_t gearSpeed[7]     = {0,100,100,100,100,100,100};

/********************************************************************************************
 *  ABSOLUTE GEAR POSITIONS
 ********************************************************************************************/

float gear_position_mm[7] = {0};
float current_position_mm = 0;

float MIN_POS_MM       = 0;
float MAX_POS_MM       = 0;
float SAFETY_MARGIN_MM = 3;

/********************************************************************************************
 *  CLOSED-LOOP CORRECTION PARAMETERS
 ********************************************************************************************/

int correction_attempts              = 0;
const int MAX_CORRECTIONS            = 3;
unsigned long last_correction_time   = 0;
const unsigned long CORRECTION_DELAY = 300;

float lazy_overshoot_mm[7] = {0,1,1,1,1,1,0};
float settle_factor = 0.60f;

/********************************************************************************************
 *  BATTERY + DRIVER
 ********************************************************************************************/

int STALL_VALUE = 20;

TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);
RunningAverage battery_voltage(100);

float last_battery_voltage        = 0;
unsigned long last_telemetry_time = 0;

/********************************************************************************************
 *  INTERRUPTS
 ********************************************************************************************/

void IRAM_ATTR index_interrupt() {
  if (move_max_steps) step_count++;
  else step_count--;
}

void IRAM_ATTR on_STALL() {
  stall_on = true;
}

/********************************************************************************************
 *  STEPPER MOTOR UTILITIES
 ********************************************************************************************/

void stepMotorRaw(uint32_t d) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(d);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(d);
}

void stepMotor(uint32_t steps, uint32_t d) {
  for (uint32_t i = 0; i < steps; i++) {
    if (stall_on) {
      driver.VACTUAL(0);
      break;
    }
    stepMotorRaw(d);
  }
}

/********************************************************************************************
 *  FLOAT MAP UTILITY
 ********************************************************************************************/

float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/********************************************************************************************
 *  COMPUTE ABSOLUTE GEAR POSITIONS
 ********************************************************************************************/

void computeGearPositions() {
  gear_position_mm[1] = 0;

  for (int g = 1; g <= 5; g++)
    gear_position_mm[g + 1] = gear_position_mm[g] + gearDistanceUp[g];

  MIN_POS_MM = gear_position_mm[1] - SAFETY_MARGIN_MM;
  MAX_POS_MM = gear_position_mm[6] + SAFETY_MARGIN_MM;
}

/********************************************************************************************
 *  MOVE TO ABSOLUTE POSITION (MM)
 ********************************************************************************************/

void moveToMM(float target, uint32_t d) {
  if (target < MIN_POS_MM) target = MIN_POS_MM;
  if (target > MAX_POS_MM) target = MAX_POS_MM;

  float delta = target - current_position_mm;
  if (fabsf(delta) < 1e-3f) return;

  digitalWrite(DIR_PIN, (delta > 0) ? FORWARD : BACKWARD);

  uint32_t steps = fabsf(delta) * STEPS_PER_MM;
  stepMotor(steps, d);

  current_position_mm = target;
}

/********************************************************************************************
 *  OPEN-LOOP SHIFTING
 ********************************************************************************************/

void moveGearUp(int g) {
  float dist = gearDistanceUp[g];
  float over = gearOvershoot[g];
  uint32_t d = gearSpeed[g];

  stall_on = false;

  digitalWrite(DIR_PIN, FORWARD);
  stepMotor((dist + over) * STEPS_PER_MM, d);

  digitalWrite(DIR_PIN, BACKWARD);
  stepMotor(over * STEPS_PER_MM, d);

  current_position_mm += dist;
  if (current_position_mm > MAX_POS_MM) current_position_mm = MAX_POS_MM;

  gear = g + 1;
  expected_gear = gear;
}

void moveGearDown(int g) {
  float dist = gearDistanceDown[g];
  float over = gearOvershoot[g];
  uint32_t d = gearSpeed[g];

  stall_on = false;

  digitalWrite(DIR_PIN, BACKWARD);
  stepMotor((dist + over) * STEPS_PER_MM, d);

  digitalWrite(DIR_PIN, FORWARD);
  stepMotor(over * STEPS_PER_MM, d);

  current_position_mm -= dist;
  if (current_position_mm < MIN_POS_MM) current_position_mm = MIN_POS_MM;

  gear = g - 1;
  expected_gear = gear;
}

/********************************************************************************************
 *  MICRO-CORRECTIONS (CLOSED-LOOP)
 ********************************************************************************************/

void microShiftUp(int target, int attempt) {
  int from = target - 1;

  float base = gearOvershoot[from];
  float lazy = lazy_overshoot_mm[from];

  float factor = (attempt == 1 ? 0.3f : attempt == 2 ? 0.5f : 0.8f);

  float total = (base + lazy) * factor;
  float settle = total * settle_factor;

  float ideal = gear_position_mm[target];

  moveToMM(ideal + total, gearSpeed[target]);
  moveToMM(ideal + total - settle, gearSpeed[target]);
  moveToMM(ideal, gearSpeed[target]);
}

void microShiftDown(int target, int attempt) {
  int from = target + 1;

  float base = gearOvershoot[from];
  float lazy = lazy_overshoot_mm[from];

  float factor = (attempt == 1 ? 0.3f : attempt == 2 ? 0.5f : 0.8f);

  float total = (base + lazy) * factor;
  float settle = total * settle_factor;

  float ideal = gear_position_mm[target];

  moveToMM(ideal - total, gearSpeed[target]);
  moveToMM(ideal - total + settle, gearSpeed[target]);
  moveToMM(ideal, gearSpeed[target]);
}

/********************************************************************************************
 *  HOMING USING STALLGUARD
 ********************************************************************************************/

void going_home() {
  if (!home_time) return;

  if (!stall_on) {
    if (gear == 6) moveGearDown(6);
    else if (gear == 5) moveGearDown(5);
    else if (gear == 4) moveGearDown(4);
    else if (gear == 3) moveGearDown(3);
    else if (gear == 2) moveGearDown(2);
    else if (gear == 1) driver.VACTUAL(-4476);
  } else {
    driver.VACTUAL(0);
    stall_on = false;
    home_time = false;

    gear = expected_gear = actual_gear = 1;
    correction_attempts = 0;

    current_position_mm = gear_position_mm[1];
    current_state = G1;
  }
}

/********************************************************************************************
 *  BUTTON DEBOUNCING
 ********************************************************************************************/

void debouncing() {
  bool b1 = !digitalRead(SWITCH1);
  bool b2 = !digitalRead(SWITCH2);

  if (b1 != bt1_b4) bt1_last_debounce = millis();
  if (millis() - bt1_last_debounce > 50) {
    if (b1 != bt1_state) {
      bt1_state = b1;
      if (bt1_state == LOW) bt1_pressed = true;
    }
  }
  bt1_b4 = b1;

  if (b2 != bt2_b4) bt2_last_debounce = millis();
  if (millis() - bt2_last_debounce > 50) {
    if (b2 != bt2_state) {
      bt2_state = b2;
      if (bt2_state == LOW) bt2_pressed = true;
    }
  }
  bt2_b4 = b2;
}

/********************************************************************************************
 *  BATTERY CHECK
 ********************************************************************************************/

void battery_check() {
  battery_voltage.addValue(analogRead(BATTERY_PIN));
  float raw = battery_voltage.getAverage();

  float v = map_f(raw, 0, 4095, 0, 3.3) + 0.7;
  v *= 4.0f;

  last_battery_voltage = v;

  if (v < 14.4) {
    digitalWrite(EN, HIGH);
    esp_sleep_enable_timer_wakeup(10 * 1000000);
    esp_deep_sleep_start();
  }
}

/********************************************************************************************
 *  READ ACTUAL GEAR FROM RASPBERRY PI
 *
 *  PI-ZERO-RPM FIX:
 *    If the Pi reports 0, it means "no measurement available".
 *    We IGNORE this value to prevent false corrections.
 ********************************************************************************************/

void readActualGear() {
  if (!Serial.available()) return;

  int g = Serial.parseInt();

  if (g == 0) return;  // Pi-zero-RPM fix

  if (g >= 1 && g <= 6) actual_gear = g;
}

/********************************************************************************************
 *  SETUP
 ********************************************************************************************/

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SW_RX, SW_TX);

  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(INDEX, INPUT);

  pinMode(EN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(EN, LOW);
  digitalWrite(DIR_PIN, LOW);

  pinMode(BATTERY_PIN, INPUT);
  pinMode(ENDSTOP, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENDSTOP), on_STALL, RISING);
  attachInterrupt(digitalPinToInterrupt(INDEX), index_interrupt, RISING);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.index_step(true);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.rms_current(500);
  driver.microsteps(16);
  driver.TCOOLTHRS(280);
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.SGTHRS(STALL_VALUE);
  driver.VACTUAL(0);

  computeGearPositions();

  gear = expected_gear = actual_gear = 6;
  current_position_mm = gear_position_mm[6];

  current_state = homing;
  home_time = true;
}

/********************************************************************************************
 *  MAIN LOOP
 ********************************************************************************************/

void loop() {
  debouncing();
  battery_check();
  readActualGear();

  static bool last_bt1=false, last_bt2=false;
  static unsigned long wait_time=0;
  static bool time_running=false;

  bool b1c = (bt1_pressed != last_bt1);
  bool b2c = (bt2_pressed != last_bt2);

  if ((b1c || b2c) && !time_running) {
    wait_time = millis();
    time_running = true;
  }

  switch (current_state) {
    case G1:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt1_pressed) {
          moveGearUp(1); current_state=G2;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case G2:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt1_pressed) {
          moveGearUp(2); current_state=G3;
        } else if (bt2_pressed) {
          moveGearDown(2); current_state=G1;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case G3:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt1_pressed) {
          moveGearUp(3); current_state=G4;
        } else if (bt2_pressed) {
          moveGearDown(3); current_state=G2;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case G4:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt1_pressed) {
          moveGearUp(4); current_state=G5;
        } else if (bt2_pressed) {
          moveGearDown(4); current_state=G3;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case G5:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt1_pressed) {
          moveGearUp(5); current_state=G6;
        } else if (bt2_pressed) {
          moveGearDown(5); current_state=G4;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case G6:
      if (time_running && millis()-wait_time>500) {
        if (bt1_pressed && bt2_pressed) {
          home_time=true; current_state=homing;
        } else if (bt2_pressed && !bt1_pressed) {
          moveGearDown(6); current_state=G5;
        }
        bt1_pressed=bt2_pressed=false; time_running=false;
      }
      break;

    case homing:
      going_home();
      break;
  }

  /******************************************************************************************
   *  CLOSED-LOOP CORRECTION
   *
   *  Pi-zero-RPM fix:
   *    If actual_gear == 0, skip correction entirely.
   ******************************************************************************************/

  if (actual_gear == 0) {
    correction_attempts = 0;
  }
  else if (actual_gear != expected_gear) {
    if (correction_attempts < MAX_CORRECTIONS &&
        millis() - last_correction_time > CORRECTION_DELAY) {

      correction_attempts++;
      last_correction_time = millis();

      if (actual_gear < expected_gear) {
        microShiftUp(expected_gear, correction_attempts);
      } else {
        microShiftDown(expected_gear, correction_attempts);
      }
    }
  }
  else {
    correction_attempts = 0;
  }

  /******************************************************************************************
   *  TELEMETRY
   ******************************************************************************************/

  if (millis() - last_telemetry_time >= 50) {
    last_telemetry_time = millis();
    Serial.print("g,");
    Serial.print(gear);
    Serial.print(",");
    Serial.print(current_position_mm);
    Serial.print(",");
    Serial.println(last_battery_voltage);
  }

  last_bt1 = bt1_pressed;
  last_bt2 = bt2_pressed;
  }
