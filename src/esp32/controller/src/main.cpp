#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace {
constexpr uint8_t kPca9685Address = 0x40;
constexpr uint8_t kPcaFrequencyHz = 50;
constexpr uint8_t kI2cSdaPin = 21;
constexpr uint8_t kI2cSclPin = 22;
constexpr uint16_t kShoulderMinPulse = 120;
constexpr uint16_t kShoulderMaxPulse = 550;
constexpr uint16_t kElbowMinPulse = 120;
constexpr uint16_t kElbowMaxPulse = 550;
constexpr uint16_t kWristPitchMinPulse = 100;
constexpr uint16_t kWristPitchMaxPulse = 510; 
constexpr uint16_t kWristRollMinPulse = 120;
constexpr uint16_t kWristRollMaxPulse = 500;
constexpr uint16_t kGripperMinPulse = 120;
constexpr uint16_t kGripperMaxPulse = 550;
constexpr int kGripperMaxAngle = 80;
constexpr int kDefaultSpeed = 100;
constexpr uint16_t kMinStepIntervalMs = 1;
constexpr uint16_t kMaxStepIntervalMs = 8;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(kPca9685Address);

struct ServoState {
  const char *name;
  uint8_t channel;
  float angle;
  float targetAngle;
  uint16_t minPulse;
  uint16_t maxPulse;
};

ServoState servos[] = {
    {"shoulder", 0, 90, 90, kShoulderMinPulse, kShoulderMaxPulse},
    {"elbow", 1, 90, 90, kElbowMinPulse, kElbowMaxPulse},
    {"wristpitch", 2, 90, 90, kWristPitchMinPulse, kWristPitchMaxPulse},
    {"wristroll", 3, 90, 90, kWristRollMinPulse, kWristRollMaxPulse},
    {"gripper", 4, 90, 90, kGripperMinPulse, kGripperMaxPulse},
};

String inputBuffer;
int motionSpeed = kDefaultSpeed;
unsigned long lastMotionUpdateMs = 0;

String normalizeName(String value) {
  value.trim();
  value.toLowerCase();
  value.replace("_", "");
  value.replace("-", "");
  value.replace(" ", "");
  return value;
}

float maxAngleForServo(const ServoState &servo) {
  return (String(servo.name) == "gripper") ? kGripperMaxAngle : 180;
}

uint16_t angleToPulse(const ServoState &servo, float angle) {
  angle = constrain(angle, 0.0f, maxAngleForServo(servo));
  float pulse = servo.minPulse + ((angle / 180.0f) * (servo.maxPulse - servo.minPulse));
  return static_cast<uint16_t>(lroundf(pulse));
}

void writeServoImmediate(ServoState &servo, float angle) {
  servo.angle = constrain(angle, 0.0f, maxAngleForServo(servo));
  servo.targetAngle = servo.angle;
  pwm.setPWM(servo.channel, 0, angleToPulse(servo, servo.angle));
}

void setServoTarget(ServoState &servo, float angle) {
  servo.targetAngle = constrain(angle, 0.0f, maxAngleForServo(servo));
}

uint16_t motionStepIntervalMs() {
  int constrainedSpeed = constrain(motionSpeed, 0, 100);
  return map(constrainedSpeed, 0, 100, kMaxStepIntervalMs, kMinStepIntervalMs);
}

void updateMotion() {
  unsigned long now = millis();
  if (now - lastMotionUpdateMs < motionStepIntervalMs()) {
    return;
  }

  lastMotionUpdateMs = now;

  for (ServoState &servo : servos) {
    if (servo.angle < servo.targetAngle) {
      servo.angle = min(servo.angle + 0.2f, servo.targetAngle);
      pwm.setPWM(servo.channel, 0, angleToPulse(servo, servo.angle));
    } else if (servo.angle > servo.targetAngle) {
      servo.angle = max(servo.angle - 0.2f, servo.targetAngle);
      pwm.setPWM(servo.channel, 0, angleToPulse(servo, servo.angle));
    }
  }
}

ServoState *findServo(const String &name) {
  String normalized = normalizeName(name);

  for (ServoState &servo : servos) {
    if (normalized == servo.name) {
      return &servo;
    }
  }

  return nullptr;
}

ServoState *findServoByAlias(const String &name) {
  String normalized = normalizeName(name);

  if (normalized == "s") {
    return &servos[0];
  }

  if (normalized == "e") {
    return &servos[1];
  }

  if (normalized == "p") {
    return &servos[2];
  }

  if (normalized == "r") {
    return &servos[3];
  }

  if (normalized == "g") {
    return &servos[4];
  }

  return findServo(name);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  shoulder <0-180>");
  Serial.println("  elbow <0-180>");
  Serial.println("  wristPitch <0-180>");
  Serial.println("  wristRoll <0-180>");
  Serial.println("  gripper <0-80>");
  Serial.println("  s <0-180> | e <0-180> | p <0-180> | r <0-180> | g <0-80>");
  Serial.println("  joint <name> <angle>");
  Serial.println("  all <angle>");
  Serial.println("  all <shoulder> <elbow> <wristPitch> <wristRoll> <gripper>");
  Serial.println("  speed <0-100>");
}

void printStatus() {
  for (const ServoState &servo : servos) {
    Serial.print(servo.name);
    Serial.print(": ");
    Serial.println(servo.angle, 1);
  }
}

void setAllServos(int shoulder, int elbow, int wristPitch, int wristRoll, int gripper) {
  setServoTarget(servos[0], shoulder);
  setServoTarget(servos[1], elbow);
  setServoTarget(servos[2], wristPitch);
  setServoTarget(servos[3], wristRoll);
  setServoTarget(servos[4], gripper);
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String command = line;
  command.toLowerCase();

  if (command == "help") {
    printHelp();
    return;
  }

  if (command == "status") {
    printStatus();
    return;
  }

  char speedKeyword[16] = {0};
  int speedValue = 0;
  int speedParsed = sscanf(line.c_str(), "%15s %d", speedKeyword, &speedValue);
  if (speedParsed == 2 && normalizeName(String(speedKeyword)) == "speed") {
    motionSpeed = constrain(speedValue, 0, 100);
    Serial.print("Motion speed set to ");
    Serial.println(motionSpeed);
    return;
  }

  int values[5];
  char keyword[16] = {0};
  int parsed = sscanf(line.c_str(), "%15s %d %d %d %d %d", keyword, &values[0], &values[1], &values[2], &values[3], &values[4]);

  if (parsed == 2 && normalizeName(String(keyword)) == "all") {
    setAllServos(values[0], values[0], values[0], values[0], values[0]);
    printStatus();
    return;
  }

  if (parsed == 6 && normalizeName(String(keyword)) == "all") {
    setAllServos(values[0], values[1], values[2], values[3], values[4]);
    printStatus();
    return;
  }

  char jointKeyword[16] = {0};
  char jointName[16] = {0};
  int angle = 0;
  int jointParsed = sscanf(line.c_str(), "%15s %15s %d", jointKeyword, jointName, &angle);

  if (jointParsed == 3 && normalizeName(String(jointKeyword)) == "joint") {
    ServoState *servo = findServo(String(jointName));
    if (servo == nullptr) {
      Serial.println("Unknown joint. Use help for valid names.");
      return;
    }

    setServoTarget(*servo, angle);
    printStatus();
    return;
  }

  char name[16] = {0};
  int singleAngle = 0;
  int singleParsed = sscanf(line.c_str(), "%15s %d", name, &singleAngle);

  if (singleParsed == 2) {
    ServoState *servo = findServoByAlias(String(name));
    if (servo == nullptr) {
      Serial.println("Unknown joint. Use help for valid names.");
      return;
    }

    setServoTarget(*servo, singleAngle);
    printStatus();
    return;
  }

  Serial.println("Invalid command. Type help for usage.");
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(kI2cSdaPin, kI2cSclPin);

  pwm.begin();
  pwm.setPWMFreq(kPcaFrequencyHz);

  for (ServoState &servo : servos) {
    writeServoImmediate(servo, 90);
  }

  Serial.println("PCA9685 ready.");
  printHelp();
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
      continue;
    }

    inputBuffer += incoming;
  }

  updateMotion();
}