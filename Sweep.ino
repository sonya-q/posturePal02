#include <Servo.h>

Servo servoEars;

const int PIN_EARS = 9;


// --- Servo speed settings ---
const int SPEED_STOP     = 90;  // no movement
const int SPEED_FORWARD  = 82;  // tilt down
const int SPEED_BACKWARD = 98;  // tilt up

// --- Timing (milliseconds) ---
const int STEP_TIME_WARN  = 250;  // small step (GOOD→WARN or WARN→GOOD)
const int STEP_TIME_BAD   = 500;  // full step (GOOD→BAD or BAD→GOOD)

// --- Fixed angles for each state ---
const int ANGLE_GOOD = 90;  // fully centered
const int ANGLE_WARN = 75;  // slightly down
const int ANGLE_BAD  = 60;  // fully down

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;


// Move servo in given direction for a certain duration
void moveServo(int speed, int duration) {
  servoEars.write(speed);
  delay(duration);
  servoEars.write(SPEED_STOP);

}

// Apply transition from one state to another
void applyTransition(PostureState from, PostureState to) {
  Serial.print("Transition: ");
  Serial.print(from == STATE_GOOD ? "GOOD" :
               from == STATE_WARN ? "WARN" : "BAD");
  Serial.print(" -> ");
  Serial.println(to == STATE_GOOD ? "GOOD" :
                 to == STATE_WARN ? "WARN" : "BAD");

  // DOWNWARD movement (toward worse posture)
  if ((from == STATE_GOOD && to == STATE_WARN) ||
      (from == STATE_WARN && to == STATE_BAD)) {
    int steps = (from == STATE_GOOD && to == STATE_WARN) ? STEP_TIME_WARN : STEP_TIME_WARN;
    moveServo(SPEED_FORWARD, steps);
  }
  else if (from == STATE_GOOD && to == STATE_BAD) {
    moveServo(SPEED_FORWARD, STEP_TIME_BAD);  // full tilt
  }

  // UPWARD movement (toward better posture)
  else if ((from == STATE_BAD && to == STATE_WARN) ||
           (from == STATE_WARN && to == STATE_GOOD)) {
    int steps = (from == STATE_BAD && to == STATE_WARN) ? STEP_TIME_WARN : STEP_TIME_WARN;
    moveServo(SPEED_BACKWARD, steps);
  }
  else if (from == STATE_BAD && to == STATE_GOOD) {
    moveServo(SPEED_BACKWARD, STEP_TIME_BAD);  // full reset
  }

  // After movement, ensure servo is at exact fixed angle for target state
  int finalAngle = (to == STATE_GOOD) ? ANGLE_GOOD :
                   (to == STATE_WARN) ? ANGLE_WARN : ANGLE_BAD;
  servoEars.write(finalAngle);

  Serial.print("State now: ");
  Serial.println(to == STATE_GOOD ? "GOOD (centered)" :
                 to == STATE_WARN ? "WARN (slightly down)" :
                 "BAD (fully down)");
}

// Convert Serial input into a state
PostureState parseStateFromString(const String &str) {
  if (str == "GOOD") return STATE_GOOD;
  if (str == "WARN") return STATE_WARN;
  if (str == "BAD")  return STATE_BAD;
  return currentState;
}

void setup() {
  Serial.begin(115200);

  Serial.println("Posture Pal - Fixed Position Servo Mode");
  Serial.println("Type GOOD / WARN / BAD (then Enter) in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoEars.write(ANGLE_GOOD);  // start at GOOD
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
}
