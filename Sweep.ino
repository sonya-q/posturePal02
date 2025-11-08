#include <Servo.h>

Servo servoEars;

const int PIN_EARS = 9;

// --- Servo speed settings (tune these) ---
const int SPEED_STOP     = 90;
const int SPEED_FORWARD  = 82;   // forward = tilt down
const int SPEED_BACKWARD = 85;   // backward = tilt up

// --- Timing (tune these in milliseconds) ---
const int STEP_TIME = 250;   // one "step" time (good→warn or warn→bad)
const int RESET_TIME = 600;  // full reset to center (good)

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;

void moveServo(int speed, int duration) {
  servoEars.write(speed);
  delay(duration);
  servoEars.write(SPEED_STOP);
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
