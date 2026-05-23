#include <SCServo.h>

#define S_RXD 18
#define S_TXD 19

SMS_STS st;
byte SERVO_ID = 0;
long totalSteps = 0;

void printFeedback(long commanded) {
  Serial.print("Commanded: "); Serial.print(commanded);
  Serial.print(" | Total: ");  Serial.print(totalSteps);
  Serial.print(" | Rotations: "); Serial.print((float)totalSteps / 4096.0, 2);
  Serial.print(" | Spd: ");    Serial.print(st.ReadSpeed(-1));
  Serial.print(" | Load: ");   Serial.print(st.ReadLoad(-1));
  Serial.print(" | Volt: ");   Serial.print(st.ReadVoltage(-1) * 0.1, 1);
  Serial.print("V | Temp: ");  Serial.print(st.ReadTemper(-1));
  Serial.println("C");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);

  Serial.println("Scanning for servo...");
  for (int i = 1; i <= 253; i++) {
    if (st.Ping(i) != -1) {
      SERVO_ID = i;
      Serial.print("Servo found at ID: ");
      Serial.println(SERVO_ID);
      break;
    }
  }
  if (SERVO_ID == 0) {
    Serial.println("No servo found!");
    while(1);
  }

  st.unLockEprom(SERVO_ID);
  delay(100);
  st.writeByte(SERVO_ID, SMS_STS_MODE, 3);
  delay(100);
  st.writeWord(SERVO_ID, 11, 0);
  delay(100);
  st.LockEprom(SERVO_ID);
  delay(500);
  Serial.println("Mode set to 3 (stepper)");

  // Enable torque
  st.writeByte(SERVO_ID, SMS_STS_TORQUE_ENABLE, 1);
  delay(100);
  Serial.println("Torque enabled");

  Serial.println("Enter new servo ID (1-253):");
  while (!Serial.available());
  String input = Serial.readStringUntil('\n');
  input.trim();
  byte newID = input.toInt();

  st.unLockEprom(SERVO_ID);
  st.writeByte(SERVO_ID, SMS_STS_ID, newID);
  st.LockEprom(newID);
  delay(500);
  SERVO_ID = newID;

  if (st.Ping(SERVO_ID) != -1) {
    Serial.print("ID confirmed: ");
    Serial.println(SERVO_ID);
  } else {
    Serial.println("ID change failed!");
    while(1);
  }

  // Re-enable torque on new ID
  st.writeByte(SERVO_ID, SMS_STS_TORQUE_ENABLE, 1);
  delay(100);

  totalSteps = 0;
  Serial.println("Send steps (e.g. 4096 or -4096), max +-30000");
  Serial.println("Send 'r' to reset total");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "r") {
      totalSteps = 0;
      Serial.println("Total reset to 0");
    } else {
      long steps = input.toInt();
      steps = constrain(steps, -30000, 30000);

      st.WritePosEx(SERVO_ID, (s16)steps, 1500, 50);

      // wait until stopped
      while (true) {
        st.FeedBack(SERVO_ID);
        if (st.ReadMove(-1) == 0) break;
        delay(10);
      }

      totalSteps += steps;
      printFeedback(steps);
    }
  }
}