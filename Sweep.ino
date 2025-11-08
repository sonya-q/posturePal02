#include <Servo.h>

// ----- Servo setup -----
Servo servoEars;
const int PIN_EARS = 9;

// ----- Servo angles (tune these to your build) -----
const int ANGLE_GOOD = 90;   // center
const int ANGLE_WARN = 75;   // slightly down
const int ANGLE_BAD  = 60;   // fully down

// ----- Step delay for smooth movement (ms) -----
const int STEP_DELAY = 15;

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;

// Move servo smoothly from current position to target angle
void moveServoTo(int targetAngle) {
  int currentAngle = servoEars.read();  // current servo position

  if (currentAngle < targetAngle) {
    for (int a = currentAngle; a <= targetAngle; a++) {
      servoEars.write(a);
      delay(STEP_DELAY);
    }
  } else {
    for (int a = currentAngle; a >= targetAngle; a--) {
      servoEars.write(a);
      delay(STEP_DELAY);
    }
  }
}

// Apply transition from one state to another
void applyTransition(PostureState from, PostureState to) {
  int targetAngle;

  if (to == STATE_GOOD) targetAngle = ANGLE_GOOD;
  else if (to == STATE_WARN) targetAngle = ANGLE_WARN;
  else targetAngle = ANGLE_BAD;

  // Move servo smoothly
  moveServoTo(targetAngle);

  // Serial feedback
  Serial.print("Transition: ");
  Serial.print(from == STATE_GOOD ? "GOOD" :
               from == STATE_WARN ? "WARN" : "BAD");
  Serial.print(" -> ");
  Serial.println(to == STATE_GOOD ? "GOOD" :
                 to == STATE_WARN ? "WARN" : "BAD");

  Serial.print("Current State: ");
  Serial.println(to == STATE_GOOD ? "GOOD (centered)" :
                 to == STATE_WARN ? "WARN (slightly down)" :
                 "BAD (fully down)");
}

PostureState parseStateFromString(const String &str) {
  if (str == "GOOD") return STATE_GOOD;
  if (str == "WARN") return STATE_WARN;
  if (str == "BAD")  return STATE_BAD;
  return currentState;  // unknown input: stay in current state
}

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal - Smooth Servo Mode");
  Serial.println("Type GOOD / WARN / BAD (then Enter) in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoEars.write(ANGLE_GOOD);  // start centered
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
