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

const JointAngles homePosition = {90.0, -110.0, 20.0, 0.0};
JointAngles currentPosition = homePosition;

int move(float th1, float th2, float th3, float delta) {
    // Placeholder for move_to function
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
    //motorX.setMaxSpeed(100);
    //motorX.setSpeed(100);
    motorX.enableOutputs();

    motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
    motorY.setPinsInverted(false, false, true);
    motorY.setAcceleration(100);
    //motorY.setMaxSpeed(100);
    //motorY.setSpeed(100);
    motorY.enableOutputs();

    motorZ.setEnablePin(MOTOR_Z_ENABLE_PIN);
    motorZ.setPinsInverted(false, false, true);
    motorZ.setAcceleration(100);
    //motorZ.setMaxSpeed(100);
    //motorZ.setSpeed(100);
    motorZ.enableOutputs();

    motorA.setEnablePin(MOTOR_Z_ENABLE_PIN);
    motorA.setPinsInverted(false, false, true);
    motorA.setAcceleration(100);
    //motorA.setMaxSpeed(100);
    //motorA.setSpeed(100);
    motorA.enableOutputs();

    move(homePosition.th1, homePosition.th2, homePosition.th3, homePosition.delta);
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command = command.trim();
        if (command.startsWith("MOVE ")) {
            int th1, th2, th3, delta;
            sscanf(command.c_str(), "MOVE %d %d %d %d", &th1, &th2, &th3, &delta);
            int result = move(th1 - currentPosition.th1, th2 - currentPosition.th2, th3 - currentPosition.th3, delta - currentPosition.delta);
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