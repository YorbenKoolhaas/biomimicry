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

typedef struct {
    float th1;
    float th2;
    float th3;
    float delta;
} JointAngles;

const JointAngles homePosition = {90.0, 110.0, 20.0, 0.0};
JointAngles currentPosition = homePosition;

const float step_size = 0.9;
const int speed = 100;

int move_all(float th1, float th2, float th3, float delta) {
//    for (int i = 0; i < (delta/step_size); i++) {
//        motorA.runSpeed();
//        delay(10);
//    }
//  delay(10);
    for (int j = 0; j < (th1/step_size); j++) {
        motorX.runSpeed();
        delay(10);
    }
    delay(10);
    for (int k = 0; k < (th2/step_size); k++) {
        motorY.runSpeed();
        delay(10);
    }
    delay(10);
    for (int l = 0; l < (th3/step_size); l++) {
        motorZ.runSpeed();
        delay(10);
    }
//    motorX.move(th1/step_size);
//    motorY.move(th2/step_size);
//    motorZ.move(th3/step_size);
//    motorA.move(delta/step_size);
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
    motorX.setAcceleration(100);
    motorX.setMaxSpeed(100);
    motorX.setSpeed(speed);
    motorX.enableOutputs();

    motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
    motorY.setPinsInverted(false, false, true);
    motorY.setAcceleration(100);
    motorY.setMaxSpeed(100);
    motorY.setSpeed(speed);
    motorY.enableOutputs();

    motorZ.setEnablePin(MOTOR_Z_ENABLE_PIN);
    motorZ.setPinsInverted(false, false, true);
    motorZ.setAcceleration(100);
    motorZ.setMaxSpeed(100);
    motorZ.setSpeed(speed);
    motorZ.enableOutputs();

    motorA.setEnablePin(MOTOR_Z_ENABLE_PIN);
    motorA.setPinsInverted(false, false, true);
    motorA.setAcceleration(100);
    motorA.setMaxSpeed(100);
    motorA.setSpeed(speed);
    motorA.enableOutputs();

    move_all(homePosition.th1, homePosition.th2, homePosition.th3, homePosition.delta);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("MOVE ")) {
            int th1, th2, th3, delta;
            sscanf(command.c_str(), "MOVE %d %d %d %d", &th1, &th2, &th3, &delta);
            int result = move_all(th1 - currentPosition.th1, th2 - currentPosition.th2, th3 - currentPosition.th3, delta - currentPosition.delta);
            if (result == 0) {
                currentPosition.th1 = th1;
                currentPosition.th2 = th2;
                currentPosition.th3 = th3;
                currentPosition.delta = delta;
                Serial.println("Move successful");
            } else {
                Serial.println("Move failed");
            }
        } else {
            Serial.println("Unknown command");
        }
    }
}
