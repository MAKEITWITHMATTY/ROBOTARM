#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

// Arduino signal pins
constexpr uint8_t BASE_PIN     = 9;
constexpr uint8_t SHOULDER_PIN = 8;
constexpr uint8_t ELBOW_PIN    = 6;
constexpr uint8_t WRIST_PIN    = 4;
constexpr uint8_t GRIPPER_PIN  = 2;

// Defined home positions.
// These may need adjusting to suit your arm.
constexpr int BASE_HOME     = 90;
constexpr int SHOULDER_HOME = 90;
constexpr int ELBOW_HOME    = 90;
constexpr int WRIST_HOME    = 90;
constexpr int GRIPPER_HOME  = 60;

// Current commanded positions.
// These become valid after homing.
int baseAngle     = BASE_HOME;
int shoulderAngle = SHOULDER_HOME;
int elbowAngle    = ELBOW_HOME;
int wristAngle    = WRIST_HOME;
int gripperAngle  = GRIPPER_HOME;

// Safer temporary movement limits.
// Increase these only after checking the real mechanical endpoints.
constexpr int BASE_MIN     = 10;
constexpr int BASE_MAX     = 170;

constexpr int SHOULDER_MIN = 25;
constexpr int SHOULDER_MAX = 155;

constexpr int ELBOW_MIN    = 20;
constexpr int ELBOW_MAX    = 160;

constexpr int WRIST_MIN    = 15;
constexpr int WRIST_MAX    = 165;

constexpr int GRIPPER_MIN  = 25;
constexpr int GRIPPER_MAX  = 175;

// Movement per key press.
constexpr int STEP_SIZE = 5;

bool armHomed = false;

void moveServo(
  Servo &servo,
  int &angle,
  int amount,
  int minimumAngle,
  int maximumAngle
) {
  angle = constrain(
    angle + amount,
    minimumAngle,
    maximumAngle
  );

  servo.write(angle);
}

void printAngles() {
  Serial.print("Base: ");
  Serial.print(baseAngle);

  Serial.print("  Shoulder: ");
  Serial.print(shoulderAngle);

  Serial.print("  Elbow: ");
  Serial.print(elbowAngle);

  Serial.print("  Wrist: ");
  Serial.print(wristAngle);

  Serial.print("  Gripper: ");
  Serial.println(gripperAngle);
}

void homeArm() {
  Serial.println();
  Serial.println("Homing arm one joint at a time...");
  Serial.println("Keep clear and support the arm if necessary.");

  /*
    Standard servos do not provide their current positions.

    The first movement of each servo may therefore still be larger
    than expected. Homing them separately prevents all five joints
    moving at once.
  */

  Serial.println("Homing base...");
  baseServo.write(BASE_HOME);
  baseServo.attach(BASE_PIN);
  delay(1500);

  Serial.println("Homing shoulder...");
  shoulderServo.write(SHOULDER_HOME);
  shoulderServo.attach(SHOULDER_PIN);
  delay(1500);

  Serial.println("Homing elbow...");
  elbowServo.write(ELBOW_HOME);
  elbowServo.attach(ELBOW_PIN);
  delay(1500);

  Serial.println("Homing wrist...");
  wristServo.write(WRIST_HOME);
  wristServo.attach(WRIST_PIN);
  delay(1500);

  Serial.println("Homing gripper...");
  gripperServo.write(GRIPPER_HOME);
  gripperServo.attach(GRIPPER_PIN);
  delay(1500);

  baseAngle     = BASE_HOME;
  shoulderAngle = SHOULDER_HOME;
  elbowAngle    = ELBOW_HOME;
  wristAngle    = WRIST_HOME;
  gripperAngle  = GRIPPER_HOME;

  armHomed = true;

  Serial.println();
  Serial.println("Homing complete.");
  Serial.println("Arm controls are now enabled.");
}

void setup() {
  Serial.begin(115200);

  /*
    Do not attach or command any servos on startup.

    The arm will remain untouched until 0 is pressed.
  */

  Serial.println();
  Serial.println("Robot arm controller ready");
  Serial.println();
  Serial.println("Press 0 to home the arm.");
  Serial.println("Movement controls are disabled until homing is complete.");
  Serial.println();
  Serial.println("A/D = base");
  Serial.println("W/S = shoulder");
  Serial.println("R/F = elbow");
  Serial.println("T/G = wrist");
  Serial.println("Y/H = gripper");
  Serial.println("P = print angles");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  char command = Serial.read();

  // Ignore line-ending characters.
  if (command == '\r' || command == '\n') {
    return;
  }

  // Convert uppercase letters to lowercase.
  if (command >= 'A' && command <= 'Z') {
    command += 'a' - 'A';
  }

  // Homing is always allowed.
  if (command == '0') {
    if (!armHomed) {
      homeArm();
    } else {
      Serial.println("Arm is already homed.");
    }

    return;
  }

  // Print the stored angles.
  if (command == 'p') {
    if (armHomed) {
      printAngles();
    } else {
      Serial.println("Arm has not been homed yet.");
    }

    return;
  }

  // Do not allow movement before homing.
  if (!armHomed) {
    Serial.println("Press 0 to home the arm first.");
    return;
  }

  switch (command) {
    case 'a':
      moveServo(
        baseServo,
        baseAngle,
        -STEP_SIZE,
        BASE_MIN,
        BASE_MAX
      );
      break;

    case 'd':
      moveServo(
        baseServo,
        baseAngle,
        STEP_SIZE,
        BASE_MIN,
        BASE_MAX
      );
      break;

    case 'w':
      moveServo(
        shoulderServo,
        shoulderAngle,
        STEP_SIZE,
        SHOULDER_MIN,
        SHOULDER_MAX
      );
      break;

    case 's':
      moveServo(
        shoulderServo,
        shoulderAngle,
        -STEP_SIZE,
        SHOULDER_MIN,
        SHOULDER_MAX
      );
      break;

    case 'r':
      moveServo(
        elbowServo,
        elbowAngle,
        STEP_SIZE,
        ELBOW_MIN,
        ELBOW_MAX
      );
      break;

    case 'f':
      moveServo(
        elbowServo,
        elbowAngle,
        -STEP_SIZE,
        ELBOW_MIN,
        ELBOW_MAX
      );
      break;

    case 't':
      moveServo(
        wristServo,
        wristAngle,
        STEP_SIZE,
        WRIST_MIN,
        WRIST_MAX
      );
      break;

    case 'g':
      moveServo(
        wristServo,
        wristAngle,
        -STEP_SIZE,
        WRIST_MIN,
        WRIST_MAX
      );
      break;

    case 'y':
      moveServo(
        gripperServo,
        gripperAngle,
        STEP_SIZE,
        GRIPPER_MIN,
        GRIPPER_MAX
      );
      break;

    case 'h':
      moveServo(
        gripperServo,
        gripperAngle,
        -STEP_SIZE,
        GRIPPER_MIN,
        GRIPPER_MAX
      );
      break;
  }
}
