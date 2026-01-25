#include <Stepper.h>

const int stepsPerRev = 2048;
Stepper stepper(stepsPerRev, 22, 24, 23, 25);

// Buttons
const int homePin = 30;   // START of screw
const int endPin  = 31;   // END of screw

enum State {
  MOVE_TO_END,
  COUNT_TO_HOME,
  MOVE_TO_MIDDLE,
  DONE
};

State state = MOVE_TO_END;

long maxSteps = 0;
long stepCounter = 0;
long targetSteps = 0;

void setup() {
  pinMode(homePin, INPUT_PULLUP);
  pinMode(endPin, INPUT_PULLUP);

  stepper.setSpeed(5);   // slow & safe
}

void loop() {

  switch (state) {

    // Go to END (no counting yet)
    case MOVE_TO_END:
      if (digitalRead(endPin) == LOW) {
        stepCounter = 0;
        state = COUNT_TO_HOME;
        delay(200);  // debounce
      } else {
        stepper.step(1);   // forward
        delay(5);
      }
      break;

    //  Move back to START and count steps
    case COUNT_TO_HOME:
      if (digitalRead(homePin) == LOW) {
        maxSteps = stepCounter;
        targetSteps = maxSteps / 2;
        state = MOVE_TO_MIDDLE;
        delay(200);  // debounce
      } else {
        stepper.step(-1);  // backward
        stepCounter++;
        delay(5);
      }
      break;

    //  Move forward to halfway point
    case MOVE_TO_MIDDLE:
      if (targetSteps <= 0) {
        state = DONE;
      } else {
        stepper.step(1);   // forward
        targetSteps--;
        delay(5);
      }
      break;

    //  Finished
    case DONE:
      // Platform is now at 50% travel
      while (true);
  }
}