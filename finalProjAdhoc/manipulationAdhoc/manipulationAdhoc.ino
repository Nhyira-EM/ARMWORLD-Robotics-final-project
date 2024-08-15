#include <Servo.h>

Servo base;  // create Servo object to control a servo
Servo limb1;  // create Servo object to control a servo
Servo limb2;  // create Servo object to control a servo
//Servo clawx;  // create Servo object to control a servo
Servo clawy;  // create Servo object to control a servo
Servo grip;  // create Servo object to control a servo


int pos;    // variable to store the servo position
int basepos;
int limb1pos;
int limb2pos;
int clawxpos;
int clawypos;
int grippos;
int baseend;
int limb1end;
int limb2end;
//int clawxend;
int clawyend;
int gripend;

int blue_led = 11;
int red_led = 12;

void setup() {
  base.attach(3); //Servo3
  limb1.attach(5); //Servo1
  limb2.attach(6); //Servo5
  //clawx.attach(9); //Servo4
  clawy.attach(9); //Servo2
  grip.attach(10); //Servo6

  pinMode(blue_led, OUTPUT);
  pinMode(red_led, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(blue_led, HIGH);
  delay(500);
  digitalWrite(blue_led, LOW);

  digitalWrite(red_led, HIGH);
  delay(500);
  digitalWrite(red_led, LOW);

  starting();

  digitalWrite(blue_led, HIGH);
  pickWaste();
  delay(500);
  dropWaste();
  delay(2000);
  digitalWrite(blue_led, LOW);
}

void starting(){
  linearMove(94,120,60,15,80, 50);
}

void pickWaste(){
  linearMove(94,120,60,15,120, 50);
  delay(500);
  grabWaste();
}

void dropWaste(){
  linearMove(180,120,40,40,25, 50);
  relWaste();
}

void grabWaste(){
  grippos = grip.read();
  for (pos = grippos; pos>=25; pos -= 1){
      grip.write(pos);
      delay(15);
  }
}

void relWaste(){
  grippos = grip.read();
  for (pos = grippos; pos<=120; pos += 1){
      grip.write(pos);
      delay(10);
  }
}

void linearMove(int baseEnd, int limb1End, int limb2End, int clawyEnd, int gripEnd, int steps) {
    for (int i = 0; i <= steps; i++) {
        base.write(map(i, 0, steps, base.read(), baseEnd));
        limb1.write(map(i, 0, steps, limb1.read(), limb1End));
        limb2.write(map(i, 0, steps, limb2.read(), limb2End));
        clawy.write(map(i, 0, steps, clawy.read(), clawyEnd));
        grip.write(map(i, 0, steps, grip.read(), gripEnd));
        delay(15);
    }
}
