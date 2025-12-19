#include <Servo.h>
#include <AFMotor.h>

#define PUMP_ONE_PIN 14
#define PUMP_TWO_PIN 15

int pos_stepper = 0; // position of stepper motor in steps
int pos_servo = 0;   // position of servo in degrees

AF_Stepper stepper_scissors(200, 1);
Servo servo_scissors;

void setup() {
  Serial.begin(9600);

  stepper_scissors.setSpeed(10);

  servo_scissors.attach(10);

  pinMode(PUMP_ONE_PIN, OUTPUT);
  pinMode(PUMP_TWO_PIN, OUTPUT);
  close_scissors();
}

int pump_gripper(int time_ms, bool direction) {
  if (direction) {
    digitalWrite(PUMP_ONE_PIN, HIGH);
    digitalWrite(PUMP_TWO_PIN, LOW);
  } else {
    digitalWrite(PUMP_ONE_PIN, LOW);
    digitalWrite(PUMP_TWO_PIN, HIGH);
  }
  delay(time_ms);
  digitalWrite(PUMP_ONE_PIN, LOW);
  digitalWrite(PUMP_TWO_PIN, LOW);
  return 0;
}

void move_scissors(int amount) {
  // Calculation for number of steps to move
  int delta = amount - pos_stepper;
  int steps = delta; // Assuming 1 unit of amount equals 1 step

  if (steps > 0) {
    stepper_scissors->step(steps, FORWARD, MICROSTEP);
  } else if (steps < 0) {
    stepper_scissors->step(-steps, BACKWARD, MICROSTEP);
  }
  pos_stepper = amount;
}

void close_scissors() {
  // Close scissors using servo
  servo_scissors.write(180);
  delay(500); // Wait for servo to move
  servo_scissors.write(0);
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
    } else if (command.startsWith("PUMP ")) {
      int time_ms;
      int direction;
      sscanf(command.c_str(), "PUMP %d %d", &time_ms, &direction);
      int result = pump_gripper(time_ms, direction != 0);
      if (result == 0) {
        Serial.println("Pump operation successful");
      } else {
        Serial.println("Pump operation failed");
      }
    } else {
      Serial.println("Unknown command");
    }
  }
}
