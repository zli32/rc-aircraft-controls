#include <Servo.h>

Servo servo[6];
uint16_t angle = 0;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 6; i++) {
    servo[i].attach(i+6);
    servo[i].write(90);
  }
}

void loop() {
  angle += 2;
  angle %= 180;
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 6; i++) {
    servo[i].write(angle);
    Serial.print("angle: "); Serial.println(angle, DEC);
  }
  delay(10);
}
