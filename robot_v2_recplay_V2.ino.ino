#include <Servo.h>
#include <EEPROM.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

// ------------------------------------------------------------
// Arduino signal pins
// ------------------------------------------------------------
constexpr uint8_t BASE_PIN     = 9;
constexpr uint8_t SHOULDER_PIN = 8;
constexpr uint8_t ELBOW_PIN    = 6;
constexpr uint8_t WRIST_PIN    = 4;
constexpr uint8_t GRIPPER_PIN  = 2;

// ------------------------------------------------------------
// Home positions
// ------------------------------------------------------------
constexpr int BASE_HOME     = 90;
constexpr int SHOULDER_HOME = 90;
constexpr int ELBOW_HOME    = 90;
constexpr int WRIST_HOME    = 90;
constexpr int GRIPPER_HOME  = 60;

// Current commanded positions.
int baseAngle     = BASE_HOME;
int shoulderAngle = SHOULDER_HOME;
int elbowAngle    = ELBOW_HOME;
int wristAngle    = WRIST_HOME;
int gripperAngle  = GRIPPER_HOME;

// ------------------------------------------------------------
// Safe movement limits
// ------------------------------------------------------------
constexpr int BASE_MIN     = 10;
constexpr int BASE_MAX     = 170;

constexpr int SHOULDER_MIN = 25;
constexpr int SHOULDER_MAX = 165;

constexpr int ELBOW_MIN    = 20;
constexpr int ELBOW_MAX    = 166;

constexpr int WRIST_MIN    = 15;
constexpr int WRIST_MAX    = 165;

constexpr int GRIPPER_MIN  = 25;
constexpr int GRIPPER_MAX  = 175;

// Movement per key press.
constexpr int STEP_SIZE = 5;

// Time between recorded steps during playback.
constexpr unsigned long PLAYBACK_STEP_DELAY = 120;

// ------------------------------------------------------------
// Record / playback storage
// ------------------------------------------------------------
struct ArmPosition {
  uint8_t base;
  uint8_t shoulder;
  uint8_t elbow;
  uint8_t wrist;
  uint8_t gripper;
};

constexpr uint16_t MAX_RECORD_STEPS = 150;

ArmPosition recordedMoves[MAX_RECORD_STEPS];

// Number of steps in the LAST COMPLETED/SAVED routine.
uint16_t recordedMoveCount = 0;

// Number of steps in the routine CURRENTLY BEING RECORDED.
uint16_t newRecordingMoveCount = 0;

bool armHomed = false;
bool recording = false;
bool playingBack = false;

// ------------------------------------------------------------
// EEPROM layout
//
// 0      magic byte 1
// 1      magic byte 2
// 2      format version
// 3-4    number of saved steps (uint16_t)
// 5-6    checksum (uint16_t)
// 7...   recorded ArmPosition data
//
// 150 positions x 5 bytes = 750 bytes, which fits in the
// Arduino Uno's 1 KB EEPROM.
// ------------------------------------------------------------
constexpr int EEPROM_MAGIC1_ADDR   = 0;
constexpr int EEPROM_MAGIC2_ADDR   = 1;
constexpr int EEPROM_VERSION_ADDR  = 2;
constexpr int EEPROM_COUNT_ADDR    = 3;
constexpr int EEPROM_CHECKSUM_ADDR = 5;
constexpr int EEPROM_DATA_ADDR     = 7;

constexpr uint8_t EEPROM_MAGIC1  = 'M';
constexpr uint8_t EEPROM_MAGIC2  = 'R';
constexpr uint8_t EEPROM_VERSION = 1;

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
void stopRecording();

// ------------------------------------------------------------
// Calculate a simple checksum for a completed routine
// ------------------------------------------------------------
uint16_t calculateChecksum(uint16_t count) {
  uint16_t checksum = count;

  for (uint16_t i = 0; i < count; i++) {
    checksum = static_cast<uint16_t>((checksum * 31U) + recordedMoves[i].base);
    checksum = static_cast<uint16_t>((checksum * 31U) + recordedMoves[i].shoulder);
    checksum = static_cast<uint16_t>((checksum * 31U) + recordedMoves[i].elbow);
    checksum = static_cast<uint16_t>((checksum * 31U) + recordedMoves[i].wrist);
    checksum = static_cast<uint16_t>((checksum * 31U) + recordedMoves[i].gripper);
  }

  return checksum;
}

// ------------------------------------------------------------
// Validate a position loaded from EEPROM
// ------------------------------------------------------------
bool positionIsValid(const ArmPosition &position) {
  return
    position.base     >= BASE_MIN     && position.base     <= BASE_MAX &&
    position.shoulder >= SHOULDER_MIN && position.shoulder <= SHOULDER_MAX &&
    position.elbow    >= ELBOW_MIN    && position.elbow    <= ELBOW_MAX &&
    position.wrist    >= WRIST_MIN    && position.wrist    <= WRIST_MAX &&
    position.gripper  >= GRIPPER_MIN  && position.gripper  <= GRIPPER_MAX;
}

// ------------------------------------------------------------
// Save the completed routine into EEPROM
// ------------------------------------------------------------
void saveRecordingToEEPROM() {
  if (recordedMoveCount == 0 || recordedMoveCount > MAX_RECORD_STEPS) {
    return;
  }

  // Save the movement data.
  for (uint16_t i = 0; i < recordedMoveCount; i++) {
    int address = EEPROM_DATA_ADDR + (i * sizeof(ArmPosition));
    EEPROM.put(address, recordedMoves[i]);
  }

  uint16_t checksum = calculateChecksum(recordedMoveCount);

  // Save metadata.
  EEPROM.put(EEPROM_COUNT_ADDR, recordedMoveCount);
  EEPROM.put(EEPROM_CHECKSUM_ADDR, checksum);
  EEPROM.update(EEPROM_VERSION_ADDR, EEPROM_VERSION);
  EEPROM.update(EEPROM_MAGIC2_ADDR, EEPROM_MAGIC2);
  EEPROM.update(EEPROM_MAGIC1_ADDR, EEPROM_MAGIC1);

  Serial.println(F("Routine saved permanently to EEPROM."));
}

// ------------------------------------------------------------
// Load the last completed routine from EEPROM
// ------------------------------------------------------------
bool loadRecordingFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC1_ADDR) != EEPROM_MAGIC1 ||
      EEPROM.read(EEPROM_MAGIC2_ADDR) != EEPROM_MAGIC2 ||
      EEPROM.read(EEPROM_VERSION_ADDR) != EEPROM_VERSION) {
    recordedMoveCount = 0;
    return false;
  }

  uint16_t savedCount = 0;
  uint16_t savedChecksum = 0;

  EEPROM.get(EEPROM_COUNT_ADDR, savedCount);
  EEPROM.get(EEPROM_CHECKSUM_ADDR, savedChecksum);

  if (savedCount == 0 || savedCount > MAX_RECORD_STEPS) {
    recordedMoveCount = 0;
    return false;
  }

  for (uint16_t i = 0; i < savedCount; i++) {
    int address = EEPROM_DATA_ADDR + (i * sizeof(ArmPosition));
    EEPROM.get(address, recordedMoves[i]);

    if (!positionIsValid(recordedMoves[i])) {
      recordedMoveCount = 0;
      return false;
    }
  }

  recordedMoveCount = savedCount;

  if (calculateChecksum(recordedMoveCount) != savedChecksum) {
    recordedMoveCount = 0;
    return false;
  }

  return true;
}

// ------------------------------------------------------------
// Permanently clear the saved EEPROM routine
// ------------------------------------------------------------
void clearEEPROMRecording() {
  EEPROM.update(EEPROM_MAGIC1_ADDR, 0);
  EEPROM.update(EEPROM_MAGIC2_ADDR, 0);
  EEPROM.update(EEPROM_VERSION_ADDR, 0);
}

// ------------------------------------------------------------
// Servo movement helper
// ------------------------------------------------------------
void moveServo(
  Servo &servo,
  int &angle,
  int amount,
  int minimumAngle,
  int maximumAngle
) {
  angle = constrain(angle + amount, minimumAngle, maximumAngle);
  servo.write(angle);
}

// ------------------------------------------------------------
// Print current joint angles
// ------------------------------------------------------------
void printAngles() {
  Serial.print(F("Base: "));
  Serial.print(baseAngle);

  Serial.print(F("  Shoulder: "));
  Serial.print(shoulderAngle);

  Serial.print(F("  Elbow: "));
  Serial.print(elbowAngle);

  Serial.print(F("  Wrist: "));
  Serial.print(wristAngle);

  Serial.print(F("  Gripper: "));
  Serial.println(gripperAngle);
}

// ------------------------------------------------------------
// Print controls
// ------------------------------------------------------------
void printHelp() {
  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F("        5-AXIS ROBOT ARM V2"));
  Serial.println(F("======================================"));
  Serial.println();
  Serial.println(F("0 = home arm"));
  Serial.println();
  Serial.println(F("MANUAL MOVEMENT"));
  Serial.println(F("A / D = base"));
  Serial.println(F("W / S = shoulder"));
  Serial.println(F("R / F = elbow"));
  Serial.println(F("T / G = wrist"));
  Serial.println(F("Y / H = gripper"));
  Serial.println();
  Serial.println(F("P = print current angles"));
  Serial.println();
  Serial.println(F("RECORD / PLAYBACK"));
  Serial.println(F("1 = start a NEW recording"));
  Serial.println(F("2 = finish recording and SAVE OVER old routine"));
  Serial.println(F("3 = replay saved routine"));
  Serial.println(F("4 = permanently clear saved routine"));
  Serial.println(F("5 = cancel new recording and keep old routine"));
  Serial.println(F("? = show this help"));
  Serial.println();
}

// ------------------------------------------------------------
// Store the current complete arm position in the NEW routine
// ------------------------------------------------------------
bool saveCurrentPosition() {
  if (!recording) {
    return false;
  }

  if (newRecordingMoveCount >= MAX_RECORD_STEPS) {
    Serial.println();
    Serial.println(F("Recording memory is full."));
    Serial.println(F("Saving this routine automatically..."));

    stopRecording();
    return false;
  }

  recordedMoves[newRecordingMoveCount].base =
    static_cast<uint8_t>(baseAngle);

  recordedMoves[newRecordingMoveCount].shoulder =
    static_cast<uint8_t>(shoulderAngle);

  recordedMoves[newRecordingMoveCount].elbow =
    static_cast<uint8_t>(elbowAngle);

  recordedMoves[newRecordingMoveCount].wrist =
    static_cast<uint8_t>(wristAngle);

  recordedMoves[newRecordingMoveCount].gripper =
    static_cast<uint8_t>(gripperAngle);

  newRecordingMoveCount++;

  Serial.print(F("Recorded step "));
  Serial.println(newRecordingMoveCount);

  return true;
}

// ------------------------------------------------------------
// Start a new recording.
//
// IMPORTANT:
// The previous completed routine is NOT erased here.
// It remains in EEPROM until command 2 finishes the new recording.
// ------------------------------------------------------------
void startRecording() {
  if (!armHomed) {
    Serial.println(F("Press 0 to home the arm first."));
    return;
  }

  if (playingBack) {
    Serial.println(F("Cannot record during playback."));
    return;
  }

  if (recording) {
    Serial.println(F("A recording is already in progress."));
    Serial.println(F("Press 2 to save it or 5 to cancel it."));
    return;
  }

  newRecordingMoveCount = 0;
  recording = true;

  // Save the starting position as the first step of the new routine.
  saveCurrentPosition();

  Serial.println();
  Serial.println(F("NEW RECORDING STARTED"));
  Serial.println(F("Your previous saved routine is still protected."));
  Serial.println(F("Move the arm using the normal controls."));
  Serial.println(F("Press 2 to replace the old routine with this one."));
  Serial.println(F("Press 5 to cancel and keep the old routine."));
}

// ------------------------------------------------------------
// Finish the new recording and overwrite the previous saved routine
// ------------------------------------------------------------
void stopRecording() {
  if (!recording) {
    Serial.println(F("Recording is not currently active."));
    return;
  }

  recording = false;

  if (newRecordingMoveCount == 0) {
    Serial.println(F("No new positions were recorded."));
    loadRecordingFromEEPROM();
    return;
  }

  // The new routine now becomes the completed routine.
  recordedMoveCount = newRecordingMoveCount;
  newRecordingMoveCount = 0;

  saveRecordingToEEPROM();

  Serial.println();
  Serial.println(F("RECORDING FINISHED"));
  Serial.print(F("New saved routine: "));
  Serial.print(recordedMoveCount);
  Serial.println(F(" steps."));
  Serial.println(F("The previous routine has now been replaced."));
  Serial.println(F("Press 3 to replay the new routine."));
}

// ------------------------------------------------------------
// Cancel a recording and restore the previous completed routine
// ------------------------------------------------------------
void cancelRecording() {
  if (!recording) {
    Serial.println(F("There is no recording to cancel."));
    return;
  }

  recording = false;
  newRecordingMoveCount = 0;

  bool restored = loadRecordingFromEEPROM();

  Serial.println();
  Serial.println(F("NEW RECORDING CANCELLED"));

  if (restored) {
    Serial.print(F("Previous routine restored: "));
    Serial.print(recordedMoveCount);
    Serial.println(F(" steps."));
  } else {
    Serial.println(F("There was no previous saved routine."));
  }
}

// ------------------------------------------------------------
// Permanently clear the saved movement sequence
// ------------------------------------------------------------
void clearRecording() {
  if (playingBack) {
    Serial.println(F("Cannot clear while playback is running."));
    return;
  }

  recording = false;
  newRecordingMoveCount = 0;
  recordedMoveCount = 0;

  clearEEPROMRecording();

  Serial.println();
  Serial.println(F("Saved movement routine permanently cleared."));
}

// ------------------------------------------------------------
// Move all five servos to a stored position
// ------------------------------------------------------------
void moveToRecordedPosition(const ArmPosition &position) {
  baseAngle = constrain(
    static_cast<int>(position.base),
    BASE_MIN,
    BASE_MAX
  );

  shoulderAngle = constrain(
    static_cast<int>(position.shoulder),
    SHOULDER_MIN,
    SHOULDER_MAX
  );

  elbowAngle = constrain(
    static_cast<int>(position.elbow),
    ELBOW_MIN,
    ELBOW_MAX
  );

  wristAngle = constrain(
    static_cast<int>(position.wrist),
    WRIST_MIN,
    WRIST_MAX
  );

  gripperAngle = constrain(
    static_cast<int>(position.gripper),
    GRIPPER_MIN,
    GRIPPER_MAX
  );

  baseServo.write(baseAngle);
  shoulderServo.write(shoulderAngle);
  elbowServo.write(elbowAngle);
  wristServo.write(wristAngle);
  gripperServo.write(gripperAngle);
}

// ------------------------------------------------------------
// Replay the completed saved routine
// ------------------------------------------------------------
void replayRecording() {
  if (!armHomed) {
    Serial.println(F("Press 0 to home the arm first."));
    return;
  }

  if (recording) {
    Serial.println(F("A new recording is in progress."));
    Serial.println(F("Press 2 to save it or 5 to cancel it first."));
    return;
  }

  if (recordedMoveCount == 0) {
    Serial.println(F("There is no saved routine to replay."));
    return;
  }

  playingBack = true;

  Serial.println();
  Serial.println(F("PLAYBACK STARTED"));

  for (uint16_t i = 0; i < recordedMoveCount; i++) {
    moveToRecordedPosition(recordedMoves[i]);

    Serial.print(F("Playing step "));
    Serial.print(i + 1);
    Serial.print(F(" of "));
    Serial.println(recordedMoveCount);

    delay(PLAYBACK_STEP_DELAY);
  }

  playingBack = false;

  Serial.println(F("PLAYBACK COMPLETE"));
  Serial.println();
}

// ------------------------------------------------------------
// Home the arm
// ------------------------------------------------------------
void homeArm() {
  Serial.println();
  Serial.println(F("Homing arm one joint at a time..."));
  Serial.println(F("Keep clear and support the arm if necessary."));

  Serial.println(F("Homing base..."));
  baseServo.write(BASE_HOME);
  baseServo.attach(BASE_PIN);
  delay(1500);

  Serial.println(F("Homing shoulder..."));
  shoulderServo.write(SHOULDER_HOME);
  shoulderServo.attach(SHOULDER_PIN);
  delay(1500);

  Serial.println(F("Homing elbow..."));
  elbowServo.write(ELBOW_HOME);
  elbowServo.attach(ELBOW_PIN);
  delay(1500);

  Serial.println(F("Homing wrist..."));
  wristServo.write(WRIST_HOME);
  wristServo.attach(WRIST_PIN);
  delay(1500);

  Serial.println(F("Homing gripper..."));
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
  Serial.println(F("Homing complete."));
  Serial.println(F("Arm controls are now enabled."));
  printAngles();
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("Robot arm controller ready."));

  // Load the last completed routine from the Uno's EEPROM.
  if (loadRecordingFromEEPROM()) {
    Serial.print(F("Saved routine found: "));
    Serial.print(recordedMoveCount);
    Serial.println(F(" steps."));
    Serial.println(F("It will remain saved until you record over it or clear it."));
  } else {
    Serial.println(F("No saved routine found."));
  }

  Serial.println(F("Press 0 to home the arm."));
  Serial.println(F("Movement controls are disabled until homing is complete."));

  printHelp();
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------
void loop() {
  if (Serial.available() == 0) {
    return;
  }

  char command = Serial.read();

  // Ignore Serial Monitor line endings and harmless whitespace.
  if (command == '\r' || command == '\n' ||
      command == ' '  || command == '\t') {
    return;
  }

  // Convert uppercase letters to lowercase.
  if (command >= 'A' && command <= 'Z') {
    command += 'a' - 'A';
  }

  if (command == '?') {
    printHelp();
    return;
  }

  if (command == '0') {
    if (!armHomed) {
      homeArm();
    } else {
      Serial.println(F("Arm is already homed."));
    }
    return;
  }

  if (command == 'p') {
    if (armHomed) {
      printAngles();
    } else {
      Serial.println(F("Arm has not been homed yet."));
    }
    return;
  }

  if (command == '1') {
    startRecording();
    return;
  }

  if (command == '2') {
    stopRecording();
    return;
  }

  if (command == '3') {
    replayRecording();
    return;
  }

  if (command == '4') {
    clearRecording();
    return;
  }

  if (command == '5') {
    cancelRecording();
    return;
  }

  if (!armHomed) {
    Serial.println(F("Press 0 to home the arm first."));
    return;
  }

  if (playingBack) {
    return;
  }

  bool armMoved = false;

  switch (command) {
    case 'a':
      moveServo(baseServo, baseAngle, -STEP_SIZE, BASE_MIN, BASE_MAX);
      armMoved = true;
      break;

    case 'd':
      moveServo(baseServo, baseAngle, STEP_SIZE, BASE_MIN, BASE_MAX);
      armMoved = true;
      break;

    case 'w':
      moveServo(shoulderServo, shoulderAngle, STEP_SIZE, SHOULDER_MIN, SHOULDER_MAX);
      armMoved = true;
      break;

    case 's':
      moveServo(shoulderServo, shoulderAngle, -STEP_SIZE, SHOULDER_MIN, SHOULDER_MAX);
      armMoved = true;
      break;

    case 'r':
      moveServo(elbowServo, elbowAngle, STEP_SIZE, ELBOW_MIN, ELBOW_MAX);
      armMoved = true;
      break;

    case 'f':
      moveServo(elbowServo, elbowAngle, -STEP_SIZE, ELBOW_MIN, ELBOW_MAX);
      armMoved = true;
      break;

    case 't':
      moveServo(wristServo, wristAngle, STEP_SIZE, WRIST_MIN, WRIST_MAX);
      armMoved = true;
      break;

    case 'g':
      moveServo(wristServo, wristAngle, -STEP_SIZE, WRIST_MIN, WRIST_MAX);
      armMoved = true;
      break;

    case 'y':
      moveServo(gripperServo, gripperAngle, STEP_SIZE, GRIPPER_MIN, GRIPPER_MAX);
      armMoved = true;
      break;

    case 'h':
      moveServo(gripperServo, gripperAngle, -STEP_SIZE, GRIPPER_MIN, GRIPPER_MAX);
      armMoved = true;
      break;

    default:
      Serial.println(F("Unknown command. Press ? for help."));
      return;
  }

  if (armMoved && recording) {
    saveCurrentPosition();
  }
}
