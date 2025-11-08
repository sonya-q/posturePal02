#include <Servo.h>

Servo servoEars;

const int PIN_EARS = 9;

// --- Servo speed settings (tune these) ---
const int SPEED_STOP     = 90;
const int SPEED_FORWARD  = 82;   // forward = tilt down
const int SPEED_BACKWARD = 90;   // backward = tilt up

// --- Timing (tune these in milliseconds) ---
const int MOVE_SMALL = 250;   // small movement (warn)
const int MOVE_LARGE = 500;   // large movement (bad)
const int MOVE_RESET = 700;   // go all the way back to "good" center

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

  // default: stop
  servoEars.write(SPEED_STOP);

  // going downward (toward worse posture)
  if ((from == STATE_GOOD && to == STATE_WARN)) {
    moveServo(SPEED_FORWARD, MOVE_SMALL);
  } 
  else if ((from == STATE_WARN && to == STATE_BAD)) {
    moveServo(SPEED_FORWARD, MOVE_LARGE - MOVE_SMALL);
  }
  else if ((from == STATE_GOOD && to == STATE_BAD)) {
    // full downward move at once
    moveServo(SPEED_FORWARD, MOVE_LARGE);
  }

  // going upward (toward better posture)
  else if ((from == STATE_BAD && to == STATE_WARN)) {
    moveServo(SPEED_BACKWARD, MOVE_LARGE - MOVE_SMALL);
  } 
  else if ((from == STATE_WARN && to == STATE_GOOD)) {
    moveServo(SPEED_BACKWARD, MOVE_SMALL);
  } 
  else if ((from == STATE_BAD && to == STATE_GOOD)) {
    moveServo(SPEED_BACKWARD, MOVE_RESET);
  }

  // Force reset if we somehow end at GOOD
  if (to == STATE_GOOD) {
    servoEars.write(SPEED_BACKWARD);
    delay(MOVE_RESET / 2);
  }

  servoEars.write(SPEED_STOP);

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
  Serial.println("Posture Pal - Timed Continuous Servo Mode");
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
