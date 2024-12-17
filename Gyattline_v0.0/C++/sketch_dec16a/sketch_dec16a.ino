#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  // put your setup code here, to run once:
  servo1.attach(2);
  servo2.attach(3);
}

void loop() {
  giraservo(0,20);
}

void giraservo(int DX,int SX) {
  servo1.write(90+DX);
  servo2.write(90+SX);
}
