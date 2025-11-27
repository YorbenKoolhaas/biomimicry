#include <AccelStepper.h>

// Voor de Arduino Uno + CNC shield V3
#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5


#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6


#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

#define MOTOR_A_ENABLE_PIN 8
#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13

AccelStepper motorX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper motorA(1, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);

const float step_size = 0.9;
const int base_speed = 500;
const int max_speed = 1000;
const int accel_speed = 300;

int move_all(float th1, float th2, float th3, float delta) {
    motorX.moveTo(th1/step_size);
    motorX.runToPosition();
    
    motorY.moveTo(th2/step_size);
    motorY.runToPosition();
    
    motorZ.moveTo(th3/step_size);
    motorZ.runToPosition();

//    motorA.moveTo(delta/step_size);
//    motorA.runToPosition();
    
    return 0;
}

void setup() {
    Serial.begin(9600);
    pinMode(MOTOR_X_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_Y_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_Z_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);

    motorX.setEnablePin(MOTOR_X_ENABLE_PIN);
    motorX.setPinsInverted(false, false, true);
    motorX.setAcceleration(accel_speed);
    motorX.setMaxSpeed(max_speed);
    motorX.setSpeed(base_speed);
    motorX.setCurrentPosition(0);
    motorX.enableOutputs();

    motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
    motorY.setPinsInverted(false, false, true);
    motorY.setAcceleration(accel_speed);
    motorY.setMaxSpeed(max_speed);
    motorY.setSpeed(base_speed);
    motorY.setCurrentPosition(0);
    motorY.enableOutputs();

    motorZ.setEnablePin(MOTOR_Z_ENABLE_PIN);
    motorZ.setPinsInverted(false, false, true);
    motorZ.setAcceleration(accel_speed);
    motorZ.setMaxSpeed(max_speed);
    motorZ.setSpeed(base_speed);
    motorZ.setCurrentPosition(0);
    motorZ.enableOutputs();

    motorA.setEnablePin(MOTOR_A_ENABLE_PIN);
    motorA.setPinsInverted(false, false, true);
    motorA.setAcceleration(accel_speed);
    motorA.setMaxSpeed(max_speed);
    motorA.setSpeed(base_speed);
    motorA.setCurrentPosition(0);
    motorA.enableOutputs();
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("MOVE ")) {
            int th1, th2, th3, delta;
            sscanf(command.c_str(), "MOVE %d %d %d %d", &th1, &th2, &th3, &delta);
//            String rec = "Received: " + String(th1) + "," + String(th2) + "," + String(th3) + "," + String(delta);
//            Serial.println(rec);
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
