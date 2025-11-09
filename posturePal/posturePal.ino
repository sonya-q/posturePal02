#include <Servo.h>

Servo servoEars;
Servo servoTail;

const int PIN_EARS = 9;
const int PIN_TAIL = 10;

// --- Servo speed settings (tune these) ---
const int SPEED_STOP     = 90;
const int SPEED_FORWARD  = 82;   // forward = tilt down
const int SPEED_BACKWARD = 98;   // backward = tilt up

// --- Timing (tune these in milliseconds) ---
const int STEP_TIME = 600;   // one "step" time (good→warn or warn→bad)
const int RESET_TIME = 1200;  // full reset to center (good)

// --- Tail timing ---
const unsigned long GOOD_INTERVAL = 120000;  // 2 minutes (adjust as needed)
const unsigned long WARN_INTERVAL = 180000;  // 3 minutes (adjust as needed)
const unsigned long TAIL_SWEEP_DURATION = 1000;  // 1 second per direction

// Tail speeds (for continuous rotation servo)
const int TAIL_STOP = 90;        // fine-tune this until servo stays still
const int TAIL_CLOCKWISE = 80;   // adjust for balanced rotation
const int TAIL_COUNTER = 100;    // adjust for balanced rotation

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;

// --- Tail-specific timing ---
unsigned long tailCycleStartTime = 0;
enum TailState {
  TAIL_IDLE,
  TAIL_STATE_CW,
  TAIL_STATE_CCW
};
TailState tailState = TAIL_IDLE;

void moveServo(int speed, int duration) {
  servoEars.write(speed);
  delay(duration);
  servoEars.write(SPEED_STOP);
}

void wagTail(int speed, int duration) {
  int pos = 0;
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servoTail.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servoTail.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
}

void applyTransition(PostureState from, PostureState to) {
  Serial.print("Transition: ");
  Serial.print(from == STATE_GOOD ? "GOOD" :
               from == STATE_WARN ? "WARN" : "BAD");
  Serial.print(" -> ");
  Serial.println(to == STATE_GOOD ? "GOOD" :
                 to == STATE_WARN ? "WARN" : "BAD");

  // downward (toward worse posture)
  if ((from == STATE_GOOD && to == STATE_WARN) ||
      (from == STATE_WARN && to == STATE_BAD)) {
    moveServo(SPEED_FORWARD, STEP_TIME);
  }
  else if (from == STATE_GOOD && to == STATE_BAD) {
    // full down: double the step
    moveServo(SPEED_FORWARD, STEP_TIME * 2);
  }

  // upward (toward better posture)
  else if ((from == STATE_BAD && to == STATE_WARN) ||
           (from == STATE_WARN && to == STATE_GOOD)) {
    moveServo(SPEED_BACKWARD, STEP_TIME);
  }
  else if (from == STATE_BAD && to == STATE_GOOD) {
    // full reset: double the step time
    moveServo(SPEED_BACKWARD, STEP_TIME * 2);
  }

  // Ensure consistent reset when returning to GOOD
  if (to == STATE_GOOD) {
    servoEars.write(SPEED_BACKWARD);
    delay(RESET_TIME / 4);  // gentle settle at center
    servoEars.write(SPEED_STOP);
  }

  Serial.print("State: ");
  Serial.println(to == STATE_GOOD ? "GOOD (centered)" :
                 to == STATE_WARN ? "WARN (slightly down)" :
                 "BAD (fully down)");
}

PostureState parseStateFromString(const String &str) {
  if (str == "GOOD") return STATE_GOOD;
  if (str == "WARN") return STATE_WARN;
  if (str == "BAD")  return STATE_BAD;
  return currentState;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal - Even Step Continuous Servo Mode");
  Serial.println("Type GOOD / WARN / BAD (then Enter) in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoEars.write(SPEED_STOP);
  
  servoTail.attach(PIN_TAIL);
  servoTail.write(TAIL_STOP);
  tailCycleStartTime = millis();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      PostureState newState = parseStateFromString(input);
      if (newState != currentState) {
        applyTransition(currentState, newState);
        currentState = newState;
      } else {
        Serial.print("State unchanged: ");
        Serial.println(input);
      }
    }
  }
  
  handleTailSweep();
}

void handleTailSweep() {
  unsigned long now = millis();
  unsigned long elapsed = now - tailCycleStartTime;
  
  // Determine cycle interval based on state
  unsigned long cycleInterval = 0;
  if (currentState == STATE_GOOD) cycleInterval = GOOD_INTERVAL;
  else if (currentState == STATE_WARN) cycleInterval = WARN_INTERVAL;
  else {
    // BAD state - make sure tail is stopped
    servoTail.write(TAIL_STOP);
    tailState = TAIL_IDLE;
    return;
  }
  
  // State machine for tail movement
  switch (tailState) {
    case TAIL_IDLE:
      // Wait for the full cycle interval
      if (elapsed >= cycleInterval) {
        // Start new wag cycle
        tailState = TAIL_STATE_CW;
        tailCycleStartTime = now;
        servoTail.write(TAIL_CLOCKWISE);
      }
      break;
      
    case TAIL_STATE_CW:
      // Rotate clockwise for 1 second
      if (elapsed >= TAIL_SWEEP_DURATION) {
        tailState = TAIL_STATE_CCW;
        tailCycleStartTime = now;
        servoTail.write(TAIL_COUNTER);
      }
      break;
      
    case TAIL_STATE_CCW:
      // Rotate counter-clockwise for 1 second
      if (elapsed >= TAIL_SWEEP_DURATION) {
        tailState = TAIL_IDLE;
        tailCycleStartTime = now;
        servoTail.write(TAIL_STOP);
      }
      break;
  }
}