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
const int STEP_TIME = 250;   // one "step" time (good→warn or warn→bad)
const int RESET_TIME = 600;  // full reset to center (good)

// --- Tail timing ---
const unsigned long GOOD_INTERVAL = 30000;  // 30 seconds
const unsigned long WARN_INTERVAL = 60000;  // 60 seconds
const unsigned long TAIL_SWEEP_DURATION = 1000;  // 1 second per direction

// Tail positions
const int TAIL_LEFT   = 60;
const int TAIL_RIGHT  = 120;

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;

// --- Tail-specific timing ---
unsigned long lastTailStartTime = 0;
unsigned long tailSweepStartTime = 0;
bool tailSweeping = false;
bool tailGoingRight = true;

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

  servoTail.attach(PIN_TAIL);
  servoTail.write(TAIL_LEFT);

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
  handleTailSweep();
}

void handleTailSweep() {
  unsigned long now = millis();
  unsigned long interval = 0;
  
  // Determine interval based on state
  if (currentState == STATE_GOOD) interval = GOOD_INTERVAL;
  else if (currentState == STATE_WARN) interval = WARN_INTERVAL;
  else interval = 0; // BAD → no movement
  
  // Check if it's time to start a new sweep
  if (!tailSweeping && interval > 0 && now - lastTailStartTime >= interval) {
    tailSweeping = true;
    tailSweepStartTime = now;
    servoTail.write(TAIL_RIGHT);  // start sweep to the right
    tailGoingRight = true;
  }
  
  // Handle ongoing sweep
  if (tailSweeping) {
    unsigned long sweepElapsed = now - tailSweepStartTime;
    
    // After 1 second, reverse direction
    if (tailGoingRight && sweepElapsed >= TAIL_SWEEP_DURATION) {
      servoTail.write(TAIL_LEFT);  // sweep back to left
      tailGoingRight = false;
      tailSweepStartTime = now;  // reset timer for return sweep
    }
    // After another 1 second, finish the sweep
    else if (!tailGoingRight && sweepElapsed >= TAIL_SWEEP_DURATION) {
      tailSweeping = false;
      lastTailStartTime = now;  // mark when this sweep finished
    }
  }
}