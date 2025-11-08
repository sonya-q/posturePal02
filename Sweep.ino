#include <Servo.h>

Servo servoEars;

const int PIN_EARS = 9;

// For continuous servos:
// 90 = stop
// <90 = spin one way
// >90 = spin the other way
const int SPEED_FORWARD = 70;  // adjust for slow forward
const int SPEED_BACK    = 110; // adjust for slow reverse
const int SPEED_STOP    = 90;

// How long to spin (ms)
const int MOVE_TIME = 400;  // tweak to match how far it “tilts”

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;
PostureState lastState    = STATE_GOOD;

void applyTransition(PostureState from, PostureState to) {
  // From GOOD → WARN → BAD: spin forward
  // From BAD → WARN → GOOD: spin backward
  int speed = SPEED_STOP;

  if ((from == STATE_GOOD && to == STATE_WARN) ||
      (from == STATE_WARN && to == STATE_BAD)) {
    speed = SPEED_FORWARD;        // forward
  } 
  else if ((from == STATE_WARN && to == STATE_GOOD) ||
           (from == STATE_BAD && to == STATE_GOOD) ||
           (from == STATE_BAD && to == STATE_WARN)) {
    speed = SPEED_BACK;           // backward
  }

  servoEars.write(speed);
  delay(MOVE_TIME);               // spin for some time
  servoEars.write(SPEED_STOP);    // stop
}

PostureState parseStateFromString(const String &str) {
  if (str == "GOOD") return STATE_GOOD;
  if (str == "WARN") return STATE_WARN;
  if (str == "BAD")  return STATE_BAD;
  return currentState;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal (continuous servo mode)");
  Serial.println("Type GOOD / WARN / BAD in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoEars.write(SPEED_STOP); // start stopped
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
        Serial.print("Same state: ");
        Serial.println(input);
      }
    }
  }
}
