// Controls for stepper motors of the robotic arm

#include <AccelStepper.h>

// for Arduino Uno + CNC shield V3
#define MOTOR_ENABLE_PIN 8

#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5

#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6

#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13

#define X_LIMIT_SWITCH_PIN 9
#define Y_LIMIT_SWITCH_PIN 10
#define Z_LIMIT_SWITCH_PIN 11

#define FAN_ENABLE_PIN 17

// Define stepper motor objects
AccelStepper motorX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper motorA(1, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);

// Define stepper motor constants
const float step_size = 0.9;
const int steps_full_rotation = 360 / step_size;
const int base_speed = 500;
const int max_speed = 1000;
const int accel_speed = 300;

const float motor_x_gear_ratio = 19.2; // 19.2:1
const float motor_y_gear_ratio = 19.2; // 19.2:1
const float motor_z_gear_ratio = 4; // 12:48
const float motor_a_gear_ratio = 1;  // Direct drive

const float home_position[3] = {-20, 40, 100};

int move_all(float th1, float th2, float th3, float delta) {
  motorX.moveTo((th1 * motor_x_gear_ratio)/step_size);
  motorX.runToPosition();
  
  motorY.moveTo((th2 * motor_y_gear_ratio)/step_size);
  motorY.runToPosition();
  
  motorZ.moveTo((th3 * motor_z_gear_ratio)/step_size);
  motorZ.runToPosition();

  motorA.moveTo((delta * motor_a_gear_ratio)/step_size);
  motorA.runToPosition();
  
  return 0;
}

void home_motors() {
  // Simple homing procedure
  // Steps in one direction until limit switch is triggered
  // Then moves into home position

//  if (digitalRead(X_LIMIT_SWITCH_PIN) == LOW) {
    while (digitalRead(X_LIMIT_SWITCH_PIN) == HIGH) {
      motorX.setSpeed(-base_speed);
      motorX.runSpeed();
    }
//  }
  motorX.setCurrentPosition(0);
  motorX.moveTo(home_position[0] * motor_x_gear_ratio / step_size);
  motorX.runToPosition();

//  if (digitalRead(Y_LIMIT_SWITCH_PIN) == LOW) {
    while (digitalRead(Y_LIMIT_SWITCH_PIN) == HIGH) {
      motorY.setSpeed(-base_speed);
      motorY.runSpeed();
    }
//  }
  motorY.setCurrentPosition(0);
  motorY.moveTo(home_position[1] * motor_y_gear_ratio / step_size);
  motorY.runToPosition();

//  if (digitalRead(Z_LIMIT_SWITCH_PIN) == LOW) {
    while (digitalRead(Z_LIMIT_SWITCH_PIN) == HIGH) {
      motorZ.setSpeed(-base_speed);
      motorZ.runSpeed();
    }
//  }
  motorZ.setCurrentPosition(0);
  motorZ.moveTo(home_position[2] * motor_z_gear_ratio / step_size);
  motorZ.runToPosition();
}

void setup() {
  Serial.begin(9600);

  // Initialize pins
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(FAN_ENABLE_PIN, OUTPUT);

  pinMode(X_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  pinMode(MOTOR_X_DIR_PIN, OUTPUT);

  digitalWrite(FAN_ENABLE_PIN, HIGH); // Enable fan

  motorX.setEnablePin(MOTOR_ENABLE_PIN);
  motorX.setPinsInverted(false, false, true); // dir invert: false, step invert: false, enable invert: true
  motorX.setAcceleration(accel_speed);
  motorX.setMaxSpeed(max_speed);
  motorX.setSpeed(base_speed);
  motorX.setCurrentPosition(0);
  motorX.enableOutputs();

  motorY.setEnablePin(MOTOR_ENABLE_PIN);
  motorY.setPinsInverted(false, false, true); // dir invert: false, step invert: false, enable invert: true
  motorY.setAcceleration(accel_speed);
  motorY.setMaxSpeed(max_speed);
  motorY.setSpeed(base_speed);
  motorY.setCurrentPosition(0);
  motorY.enableOutputs();

  motorZ.setEnablePin(MOTOR_ENABLE_PIN);
  motorZ.setPinsInverted(false, false, true); // dir invert: false, step invert: false, enable invert: true
  motorZ.setAcceleration(accel_speed);
  motorZ.setMaxSpeed(max_speed);
  motorZ.setSpeed(base_speed);
  motorZ.setCurrentPosition(0);
  motorZ.enableOutputs();

  motorA.setEnablePin(MOTOR_ENABLE_PIN);
  motorA.setPinsInverted(false, false, true); // dir invert: false, step invert: false, enable invert: true
  motorA.setAcceleration(accel_speed);
  motorA.setMaxSpeed(max_speed);
  motorA.setSpeed(base_speed);
  motorA.setCurrentPosition(0);
  motorA.enableOutputs();

  home_motors();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("MOVE ")) {
      
      float th1, th2, th3, delta;
      sscanf(command.c_str(), "MOVE %d %d %d %d", &th1, &th2, &th3, &delta);
      th1 = th1/100;
      th2 = th2/100;
      th3 = th3/100;
      delta = delta/100;
      
      int result = move_all(th1, th2, th3, delta);
      if (result == 0) {
        Serial.println("Move successful");
      } else {
        Serial.println("Move failed");
      }
    } else {
        Serial.println("Unknown command");
    }
  }
}
