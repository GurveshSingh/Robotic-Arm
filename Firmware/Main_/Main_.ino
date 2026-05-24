#include <SCServo.h>

SMS_STS servo;

#define S_RXD 18
#define S_TXD 19

#define JOINT1_ID 1
#define JOINT2_ID 2
#define JOINT3_ID 3

#define GEAR_RATIO      20.0f
#define STATE_PERIOD_MS 1000
#define MOVE_SPEED      2000
#define MOVE_ACC        10

int32_t radToTicks(float rad) {
  float deg = rad * 180.0 / 3.14159;
  return (int32_t)((deg / 360.0) * 4096.0);
}

int32_t armRadToTicks(float rad) {
  float deg = rad * GEAR_RATIO * 180.0 / 3.14159;
  return (int32_t)((deg / 360.0) * 4096.0);
}

// KEEPING YOUR ORIGINAL LOGIC
float last_cmd[3] = {0.0, 0.0, 0.0};

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  servo.pSerial = &Serial1;
  delay(1000);

  Serial.println("ARM_READY");
}

// ─────────────────────────────
void processCommand(String cmd) {

  Serial.print("RX: [");
  Serial.print(cmd);
  Serial.println("]");

  cmd.replace("\r", "");
  cmd.trim();

  if (!cmd.startsWith("CMD")) {
    Serial.println("Invalid CMD");
    return;
  }

  float p0, p1, p2;

  if (sscanf(cmd.c_str(), "CMD %f %f %f", &p0, &p1, &p2) != 3) {
    Serial.println("Parse error");
    return;
  }

  // 🔴 YOUR ORIGINAL DELTA LOGIC (unchanged)
  p0 = p0 - last_cmd[0];
  p1 = p1 - last_cmd[1];
  p2 = p2 - last_cmd[2];

  last_cmd[0] += p0;
  last_cmd[1] += p1;
  last_cmd[2] += p2;

  int32_t t1 = radToTicks(p0);
  int32_t t2 = armRadToTicks(p1);
  int32_t t3 = armRadToTicks(p2);

  servo.WritePosEx(JOINT1_ID, t1, MOVE_SPEED, MOVE_ACC);
  delay(10);
  servo.WritePosEx(JOINT2_ID, t2, MOVE_SPEED, MOVE_ACC);
  delay(10);
  servo.WritePosEx(JOINT3_ID, t3, MOVE_SPEED, MOVE_ACC);

  Serial.print("t1="); Serial.print(t1);
  Serial.print(" t2="); Serial.print(t2);
  Serial.print(" t3="); Serial.println(t3);
}

// ─────────────────────────────
void sendState() {
  Serial.print("STA ");
  Serial.print(last_cmd[0], 4); Serial.print(" ");
  Serial.print(last_cmd[1], 4); Serial.print(" ");
  Serial.print(last_cmd[2], 4); Serial.println();
}

// ─────────────────────────────
void loop() {

  // ✅ FIXED SERIAL (ONLY CHANGE THAT MATTERS)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // periodic state
  static unsigned long lastState = 0;
  if (millis() - lastState > STATE_PERIOD_MS) {
    sendState();
    lastState = millis();
  }
}