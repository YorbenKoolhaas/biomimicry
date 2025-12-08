#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

int position = 0; // position of stepper motor in steps

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepper_scissors = AFMS.getStepper(200, 1); // 200 steps per revolution. Using port 1 (M1 & M2) for the stepper

Adafruit_ServoMotor *servo_scissors = AFMS.getServo(1); // Using port 1 for the servo

void setup() {
    Serial.begin(9600);

    stepper_scissors->setSpeed(10); // 10 rpm

}

void move_scissors(int amount) {
    // Calculation for number of steps to move
    int delta = amount - position;
    int steps = delta; // Assuming 1 unit of amount equals 1 step

    if (steps > 0) {
        stepper_scissors->step(steps, FORWARD, MICROSTEP);
    } else if (steps < 0) {
        stepper_scissors->step(-steps, BACKWARD, MICROSTEP);
    }
    position = amount;
}

void close_scissors() {
    // Close scissors using servo
    servo_scissors->write(0); // Assuming 0 degrees is closed position
    delay(500); // Wait for servo to move
}

int move_and_close_scissors(int amount) {
    move_scissors(amount);
    close_scissors();
    return 0;
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("SCISSORS ")) {
      int amount;
      sscanf(command.c_str(), "SCISSORS %d", &amount);
      int result = move_and_close_scissors(amount);
      if (result == 0) {
        Serial.println("Move scissors successful");
      } else {
        Serial.println("Move scissors failed");
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}