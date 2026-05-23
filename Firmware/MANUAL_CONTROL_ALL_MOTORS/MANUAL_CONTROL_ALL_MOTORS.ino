#include <SCServo.h>
SMS_STS servo;
#define S_RXD 18
#define S_TXD 19

#define JOINT1_ID 1
#define JOINT2_ID 2
#define JOINT3_ID 3

// Home ticks - read on startup
int32_t HOME_J1 = 0;
int32_t HOME_J2 = 0;
int32_t HOME_J3 = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  servo.pSerial = &Serial1;
  delay(1000);

  // Store home positions on startup
  // ALL joints must be at home position before powering on!
  HOME_J1 = servo.ReadPos(JOINT1_ID);
  HOME_J2 = servo.ReadPos(JOINT2_ID);
  HOME_J3 = servo.ReadPos(JOINT3_ID);

  //variable torque
  servo.writeByte(3, SMS_STS_TORQUE_ENABLE, 1);
  int torqueValue = 250;
  servo.writeWord(3, SMS_STS_TORQUE_LIMIT_L, torqueValue);

  Serial.print("J1 home: "); Serial.println(HOME_J1);
  Serial.print("J2 home: "); Serial.println(HOME_J2);
  Serial.print("J3 home: "); Serial.println(HOME_J3);
  Serial.println("Type: '[id] [angle]' e.g. '1 90' or '2 -60'");
  Serial.println("Type 'POS' to read positions");
  Serial.println("Type 'HOME' before powering off");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Read positions
    if (cmd == "POS") {
      Serial.print("J1: "); Serial.println(servo.ReadPos(JOINT1_ID));
      Serial.print("J2: "); Serial.println(servo.ReadPos(JOINT2_ID));
      Serial.print("J3: "); Serial.println(servo.ReadPos(JOINT3_ID));
      return;
    }


    

    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex == -1) return;

    int id      = cmd.substring(0, spaceIndex).toInt();
    float angle = cmd.substring(spaceIndex + 1).toFloat();

    if (id >= 1 && id <= 3) {
      // All joints step mode - same conversion
      int32_t targetSteps = (int32_t)((angle / 360.0) * 4096.0);

      servo.WritePosEx(id, targetSteps, 5000, 10);
      Serial.print("J"); Serial.print(id);
      Serial.print(" → "); Serial.print(angle);
      Serial.print("° = "); Serial.print(targetSteps); Serial.println(" ticks");
      delay(1500);
      Serial.print("Actual: "); Serial.println(servo.ReadPos(id));
    } else {
      Serial.println("Invalid ID! Use 1, 2 or 3");
    }

    while(Serial.available()) Serial.read();
  }
}

