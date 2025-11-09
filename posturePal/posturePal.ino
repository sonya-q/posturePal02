#include <Servo.h>

Servo servoEars;
Servo servoTail;

const int PIN_EARS = 9;
const int PIN_TAIL = 10;

// --- Servo speed settings (for EARS, unchanged) ---
const int SPEED_STOP     = 90;
const int SPEED_FORWARD  = 82;   // forward = tilt down
const int SPEED_BACKWARD = 98;   // backward = tilt up

// --- Timing (for EARS, unchanged) ---
const int STEP_TIME = 250;   // one "step" time (good→warn or warn→bad)
const int RESET_TIME = 600;  // full reset to center (good)

// --- Tail timing ---
// GOOD: we want continuous wag, so interval = 0 (no idle gap)
const unsigned long GOOD_INTERVAL       = 0;      // ms between wag cycles in GOOD
// WARN: wag-then-wait (e.g., wag once every 3 seconds)
const unsigned long WARN_INTERVAL       = 3000;   // ms between wag cycles in WARN
const unsigned long TAIL_SWEEP_DURATION = 1000;   // 1 second per direction

// Tail speeds (for continuous-rotation servo)
const int TAIL_STOP            = 90;
const int TAIL_SPEED_CLOCKWISE = 82;  // adjust if needed
const int TAIL_SPEED_COUNTER   = 98;  // adjust if needed

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
  TAIL_CLOCKWISE,
  TAIL_COUNTER
};

TailState tailState = TAIL_IDLE;

// ---------- EARS MOVEMENT (UNCHANGED) ----------

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

// (Your old wagTail() with for-loops is unused; we can keep or remove it)
void wagTail(int speed, int duration) {
  int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) {
    servoTail.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    servoTail.write(pos);
    delay(15);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal - Even Step Continuous Servo Mode");
  Serial.println("Type GOOD / WARN / BAD (then Enter) in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoEars.write(SPEED_STOP);

  servoTail.attach(PIN_TAIL);
  servoTail.write(TAIL_STOP);  // make sure it starts stopped

  tailCycleStartTime = millis();
  tailState = TAIL_IDLE;
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

        // reset tail cycle timing when state changes
        tailCycleStartTime = millis();
        tailState = TAIL_IDLE;
        servoTail.write(TAIL_STOP);
      } else {
        Serial.print("State unchanged: ");
        Serial.println(input);
      }
    }
  }

  handleTailSweep();  // tail behavior lives here
}

// ------- Tail wag logic for different posture states -------
void handleTailSweep() {
  unsigned long now = millis();
  unsigned long elapsed = now - tailCycleStartTime;

  // BAD: tail fully stopped
  if (currentState == STATE_BAD) {
    servoTail.write(TAIL_STOP);
    tailState = TAIL_IDLE;
    return;
  }

  // GOOD vs WARN differ by how often we start a wag cycle
  unsigned long cycleInterval = 0;
  if (currentState == STATE_GOOD) {
    // GOOD: continuous wag, so no idle gap
    cycleInterval = GOOD_INTERVAL;  // 0 → immediate / continuous
  } else if (currentState == STATE_WARN) {
    // WARN: wag, then wait
    cycleInterval = WARN_INTERVAL;
  }

  // --- GOOD: continuous wag ---
  if (currentState == STATE_GOOD) {
    // In GOOD, we don't want long idle gaps; just alternate directions
    switch (tailState) {
      case TAIL_IDLE:
        // immediately start a wag cycle
        tailState = TAIL_CLOCKWISE;
        tailCycleStartTime = now;
        servoTail.write(TAIL_SPEED_CLOCKWISE);
        break;

      case TAIL_CLOCKWISE:
        if (elapsed >= TAIL_SWEEP_DURATION) {
          tailState = TAIL_COUNTER;
          tailCycleStartTime = now;
          servoTail.write(TAIL_SPEED_COUNTER);
        }
        break;

      case TAIL_COUNTER:
        if (elapsed >= TAIL_SWEEP_DURATION) {
          // go back to clockwise, continuous loop
          tailState = TAIL_CLOCKWISE;
          tailCycleStartTime = now;
          servoTail.write(TAIL_SPEED_CLOCKWISE);
        }
        break;
    }
    return;
  }

  // --- WARN: wag then wait ---
  if (currentState == STATE_WARN) {
    switch (tailState) {
      case TAIL_IDLE:
        // Wait for the full cycle interval before starting a wag
        if (elapsed >= cycleInterval) {
          tailState = TAIL_CLOCKWISE;
          tailCycleStartTime = now;
          servoTail.write(TAIL_SPEED_CLOCKWISE);
        }
        break;

      case TAIL_CLOCKWISE:
        // Rotate clockwise for 1 second
        if (elapsed >= TAIL_SWEEP_DURATION) {
          tailState = TAIL_COUNTER;
          tailCycleStartTime = now;
          servoTail.write(TAIL_SPEED_COUNTER);
        }
        break;

      case TAIL_COUNTER:
        // Rotate counter-clockwise for 1 second, then stop & go idle
        if (elapsed >= TAIL_SWEEP_DURATION) {
          tailState = TAIL_IDLE;
          tailCycleStartTime = now;
          servoTail.write(TAIL_STOP);  // STOP the tail
        }
        break;
    }
  }
}
