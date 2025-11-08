#include <Servo.h>

Servo servoEars;
Servo servoTail;

const int PIN_EARS = 9;
const int PIN_TAIL = 10;

// --- Servo positions ---
// Ears
const int EARS_CENTER = 90;   // stop/center position
const int EARS_DOWN   = 98;   // downward position
const int EARS_UP     = 82;   // upward position

// Tail
const int TAIL_LEFT   = 45;   // left wag position
const int TAIL_CENTER = 90;   // center position
const int TAIL_RIGHT  = 135;  // right wag position

// --- Timing in milliseconds ---
const unsigned long GOOD_INTERVAL = 30000;  // 30 seconds
const unsigned long WARN_INTERVAL = 60000;  // 60 seconds

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;

// --- Timing for wagging ---
unsigned long lastMoveTime = 0;
bool earsDown = false;
bool tailRight = false;  // track tail position

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal - Periodic Servo Mode");
  Serial.println("Type GOOD / WARN / BAD (then Enter) in Serial Monitor.");
  
  servoEars.attach(PIN_EARS);
  servoTail.attach(PIN_TAIL);  // attach tail servo
  
  servoEars.write(EARS_CENTER);  // start centered
  servoTail.write(TAIL_CENTER);  // start centered
}

void loop() {
  // --- Handle serial input ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      PostureState newState = parseStateFromString(input);
      if (newState != currentState) {
        Serial.print("State changed to: ");
        Serial.println(input);
        currentState = newState;
        
        // Reset both servos to center immediately if going to GOOD or BAD
        if (currentState == STATE_GOOD || currentState == STATE_BAD) {
          servoEars.write(EARS_CENTER);
          servoTail.write(TAIL_CENTER);
          earsDown = false;
          tailRight = false;
          lastMoveTime = millis();
        }
      }
    }
  }

  // --- Handle periodic servo movement ---
  unsigned long now = millis();
  unsigned long interval = 0;
  
  if (currentState == STATE_GOOD) interval = GOOD_INTERVAL;
  else if (currentState == STATE_WARN) interval = WARN_INTERVAL;
  else interval = 0; // BAD → no movement

  if (interval > 0 && now - lastMoveTime >= interval) {
    toggleEars();
    toggleTail();  // wag tail at same time
    lastMoveTime = now;
  }
}

// --- Move ears back and forth ---
void toggleEars() {
  if (earsDown) {
    servoEars.write(EARS_CENTER);  // go back up
  } else {
    servoEars.write(EARS_DOWN);    // go down
  }
  earsDown = !earsDown;
}

// --- Move tail back and forth ---
void toggleTail() {
  if (tailRight) {
    servoTail.write(TAIL_LEFT);    // wag to left
  } else {
    servoTail.write(TAIL_RIGHT);   // wag to right
  }
  tailRight = !tailRight;
}

// --- Parse state string ---
PostureState parseStateFromString(const String &str) {
  if (str.equalsIgnoreCase("GOOD")) return STATE_GOOD;
  if (str.equalsIgnoreCase("WARN")) return STATE_WARN;
  if (str.equalsIgnoreCase("BAD"))  return STATE_BAD;
  return currentState;
}