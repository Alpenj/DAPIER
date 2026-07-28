/*
 * ROS 2 serial firmware for the four-axis mini robot arm.
 *
 * Wiring:
 *   base=D3, shoulder=D5, forearm=D6, upper arm=D9
 *
 * Input protocol:
 *   A,<base>,<shoulder>,<forearm>,<upper>\n
 * Example:
 *   A,90,80,100,90
 *
 * Output protocol:
 *   READY
 *   OK,<base>,<shoulder>,<forearm>,<upper>
 */

#include <Servo.h>

constexpr uint8_t JOINT_COUNT = 4;
constexpr uint8_t SERVO_PINS[JOINT_COUNT] = {3, 5, 6, 9};
constexpr int CENTER_ANGLE = 90;
constexpr int MIN_ANGLE = 60;
constexpr int MAX_ANGLE = 120;
constexpr unsigned long STEP_INTERVAL_MS = 20;
constexpr unsigned long COMMAND_TIMEOUT_MS = 3000;

Servo servos[JOINT_COUNT];
int currentAngles[JOINT_COUNT] = {
  CENTER_ANGLE, CENTER_ANGLE, CENTER_ANGLE, CENTER_ANGLE
};
int targetAngles[JOINT_COUNT] = {
  CENTER_ANGLE, CENTER_ANGLE, CENTER_ANGLE, CENTER_ANGLE
};

char inputBuffer[64];
uint8_t inputLength = 0;
unsigned long lastStepAt = 0;
unsigned long lastCommandAt = 0;

void printAngles(const char* prefix) {
  Serial.print(prefix);
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(targetAngles[i]);
  }
  Serial.println();
}

bool parseCommand(char* line) {
  int requested[JOINT_COUNT];
  const int matched = sscanf(
    line,
    "A,%d,%d,%d,%d",
    &requested[0],
    &requested[1],
    &requested[2],
    &requested[3]
  );

  if (matched != JOINT_COUNT) {
    Serial.println("ERR,bad_command");
    return false;
  }

  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    targetAngles[i] = constrain(requested[i], MIN_ANGLE, MAX_ANGLE);
  }
  lastCommandAt = millis();
  printAngles("OK,");
  return true;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = Serial.read();

    if (incoming == '\n') {
      inputBuffer[inputLength] = '\0';
      parseCommand(inputBuffer);
      inputLength = 0;
    } else if (incoming != '\r') {
      if (inputLength < sizeof(inputBuffer) - 1) {
        inputBuffer[inputLength++] = incoming;
      } else {
        inputLength = 0;
        Serial.println("ERR,line_too_long");
      }
    }
  }
}

void moveServosOneStep() {
  const unsigned long now = millis();
  if (now - lastStepAt < STEP_INTERVAL_MS) {
    return;
  }
  lastStepAt = now;

  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    if (currentAngles[i] < targetAngles[i]) {
      ++currentAngles[i];
    } else if (currentAngles[i] > targetAngles[i]) {
      --currentAngles[i];
    }
    servos[i].write(currentAngles[i]);
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < JOINT_COUNT; ++i) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(CENTER_ANGLE);
  }
  lastCommandAt = millis();
  Serial.println("READY");
}

void loop() {
  readSerialCommands();
  moveServosOneStep();

  // 통신이 끊겨도 마지막 자세를 유지한다. 모터처럼 갑자기 0도로
  // 보내면 로봇암이 충돌할 수 있으므로 서보에는 이 방식이 더 안전하다.
  if (millis() - lastCommandAt > COMMAND_TIMEOUT_MS) {
    lastCommandAt = millis();
    Serial.println("WARN,command_timeout_holding_position");
  }
}
