#include <Servo.h>

// ----- Servo setup -----
Servo servoEars;   // Ears servo on D9
Servo servoTail;   // Tail/sign servo on D10

const int PIN_EARS = 9;
const int PIN_TAIL = 10;

// You can tweak these angles to match your physical build
const int EARS_GOOD_ANGLE = 90;   // centered
const int EARS_WARN_ANGLE = 120;  // slightly down
const int EARS_BAD_ANGLE  = 150;  // fully down

const int TAIL_GOOD_ANGLE = 90;   // neutral
const int TAIL_WARN_ANGLE = 110;  // small change
const int TAIL_BAD_ANGLE  = 60;   // bigger change

// ----- State machine -----
enum PostureState {
  STATE_GOOD,
  STATE_WARN,
  STATE_BAD
};

PostureState currentState = STATE_GOOD;
PostureState lastState    = STATE_GOOD;

// Apply servo angles for a given state
void applyState(PostureState s) {
  if (s == STATE_GOOD) {
    servoEars.write(EARS_GOOD_ANGLE);
    servoTail.write(TAIL_GOOD_ANGLE);
    Serial.println("State: GOOD → ears center, tail neutral");
  }
  else if (s == STATE_WARN) {
    servoEars.write(EARS_WARN_ANGLE);
    servoTail.write(TAIL_WARN_ANGLE);
    Serial.println("State: WARN → ears slightly down, tail moved");
  }
  else if (s == STATE_BAD) {
    servoEars.write(EARS_BAD_ANGLE);
    servoTail.write(TAIL_BAD_ANGLE);
    Serial.println("State: BAD → ears fully down, tail strong move"); // want continuous motion? 
  }
}

// Convert text from Serial into a state
PostureState parseStateFromString(const String &str) {
  if (str == "GOOD") return STATE_GOOD;
  if (str == "WARN") return STATE_WARN;
  if (str == "BAD")  return STATE_BAD;
  // default if unknown
  return currentState;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Posture Pal - Discrete Servo States");
  Serial.println("Type GOOD, WARN, or BAD (then Enter) in Serial Monitor.");

  servoEars.attach(PIN_EARS);
  servoTail.attach(PIN_TAIL);

  // Start at GOOD position
  applyState(STATE_GOOD);
}

void loop() {
  // 1) Read from Serial if available
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // remove spaces/newline

    if (input.length() > 0) {
      PostureState newState = parseStateFromString(input);

      // 2) Only move servos if state actually changed
      if (newState != currentState) {
        currentState = newState;
        applyState(currentState);
        lastState = currentState;
      } else {
        // Same state again → do nothing, servos stay where they are
        Serial.print("State unchanged: ");
        Serial.println(input);
      }
    }
  }

  // Nothing else in loop — no constant wiggles, no repeated movement.
}
