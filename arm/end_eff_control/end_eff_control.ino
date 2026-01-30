#include <Servo.h>
#include <AFMotor.h>

#define PUMP_PIN 14

// =====================================================
// AXIS DIRECTION ALIAS (FLIP HERE IF NEEDED)
// =====================================================
#define AXIS_FORWARD  FORWARD
#define AXIS_BACKWARD BACKWARD

// =====================================================
// STEPPER
// =====================================================
AF_Stepper stepper(2048, 1);   // 28BYJ-48 on M1 + M2

// =====================================================
// SERVO
// =====================================================
Servo cutter;
const int SERVO_PIN = 10;      // SERVO1 on HW130
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

// =====================================================
// LIMIT SWITCHES
// =====================================================
const int endPin  = A0;        // END of travel  ✅ corrected
const int homePin = A5;        // START of travel

// =====================================================
// COUNTERS
// =====================================================
long stepCounter = 0;
long maxSteps = 0;
long targetSteps = 0;

int pump_gripper(int time_ms) {
  digitalWrite(PUMP_PIN, HIGH);
  delay(time_ms);
  digitalWrite(PUMP_PIN, LOW);
  return 0;
}

void runServoTwice() {
  for (int i = 0; i < 2; i++) {
    cutter.write(SERVO_MAX);
    delay(500);
    cutter.write(SERVO_MIN);
    delay(500);
  }
}

int home_scissors() {
    // -------------------------------------------------
    // 1) Move toward END switch (A5)
    // -------------------------------------------------
    while (digitalRead(endPin) == HIGH) {
      stepper.onestep(AXIS_FORWARD, DOUBLE);
      delay(2);
    }

    stepCounter = 0;
    delay(200); // debounce

    // -------------------------------------------------
    // 2) Reverse, count steps until HOME switch (A0)
    // -------------------------------------------------
    while (digitalRead(homePin) == HIGH) {
      stepper.onestep(AXIS_BACKWARD, DOUBLE);
      stepCounter++;
      delay(2);
    }
    maxSteps = stepCounter;
    targetSteps = maxSteps / 2;
    
    Serial.print("Max steps counted: ");
    Serial.println(maxSteps);

    delay(200); // debounce

    // -------------------------------------------------
    // 3) Move to midpoint
    // -------------------------------------------------
    for (int i = 0; i++; i <= targetSteps) {
      stepper.onestep(AXIS_FORWARD, DOUBLE);
      delay(2);
    }
    // pos_stepper = targetSteps

    // -------------------------------------------------
    // 4) Servo action: 2× back-and-forth
    // -------------------------------------------------
    runServoTwice();

    // -------------------------------------------------
    // 5) Finished
    // -------------------------------------------------
    return 1;
  }

void move_scissors(int amount, int pos_stepper) {
  // Calculation for number of steps to move
  int delta = amount - pos_stepper;
  int steps = delta; // Assuming 1 unit of amount equals 1 step

  if (steps > 0) {
    stepper.onestep(FORWARD, MICROSTEP);
  } else if (steps < 0) {
    stepper.onestep(BACKWARD, MICROSTEP);
  }
  pos_stepper = amount;
}

int move_and_close_scissors(int amount, int pos_stepper) {
  move_scissors(amount, pos_stepper);
  runServoTwice();
  return 1;
}


void setup() {
  Serial.begin(9600);

  pinMode(PUMP_PIN, OUTPUT);

  pinMode(endPin, INPUT_PULLUP);
  pinMode(homePin, INPUT_PULLUP);

  stepper.setSpeed(20);        // RPM (safe & slow)

  cutter.attach(SERVO_PIN);
  cutter.write(SERVO_MIN);

  Serial.println("HW130 homing + midpoint + servo sequence started");
  home_scissors();

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("SCISSORS ")) {
      int amount;
      sscanf(command.c_str(), "SCISSORS %d", &amount);
      int result = move_and_close_scissors(amount, 1);
      if (result == 1) {
        Serial.println("Move scissors successful");
      } else {
        Serial.println("Move scissors failed");
      }
    } else if (command.startsWith("PUMP ")) {
      int time_ms;
      sscanf(command.c_str(), "PUMP %d", &time_ms);
      int result = pump_gripper(time_ms);
      if (result == 1) {
        Serial.println("Pump operation successful");
      } else {
        Serial.println("Pump operation failed");
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}
