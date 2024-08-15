#include <Servo.h>
#include <stdio.h>

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
  if (Serial.available() > 0) {
      // Read the incoming byte
      String incomingData = Serial.readStringUntil('\n');
      //delay(100);
      
      if (incomingData == "metal") {
        digitalWrite(blue_led, HIGH);
        pickWaste();
        delay(500);
        dropMetal();
        delay(2000);
        digitalWrite(blue_led, LOW);

      } else if (incomingData == "plastic") {
        digitalWrite(red_led, HIGH);
        pickWaste();
        delay(500);
        dropPlastic();
        delay(500);
        digitalWrite(red_led, LOW);
      } else if (incomingData == "other"){
        digitalWrite(red_led, HIGH);
        digitalWrite(blue_led, HIGH);
        delay(500);
        digitalWrite(red_led, LOW);
        digitalWrite(blue_led, LOW);
      }
    }
}



void starting(){
  defPos(94,120,60,15,80);
}

void pickWaste(){
  defPos(94,120,60,15,120);
  delay(500);
  grabWaste();
}

void dropPlastic(){
  defPos(180,120,40,40,25);
  relWaste();
}

void dropMetal(){
  defPos(135,120,40,40,25);
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

void defPos(int baseend,int limb1end,int limb2end,int clawyend,int gripend){
  basepos = base.read();
  if (basepos>=baseend){
    for (pos = basepos; pos>=baseend; pos -= 1){
      base.write(pos);
    }}
  else{
    for (pos = basepos; pos<=baseend; pos += 1){
      base.write(pos);
    }}

  limb1pos = limb1.read();
  if (limb1pos>=limb1end){
    for (pos = limb1pos; pos>=limb1end; pos -= 1){
      limb1.write(pos);
    }}
  else{
    for (pos = limb1pos; pos<=limb1end; pos += 1){
      limb1.write(pos);
    }}

  limb2pos = limb2.read();
  if (limb2pos>=limb2end){
    for (pos = limb2pos; pos>=limb2end; pos -= 1){
      limb2.write(pos);
      delay(10);
    }}
  else{
    for (pos = limb2pos; pos<=limb2end; pos += 1){
      limb2.write(pos);
      delay(10);
    }}

  // clawxpos = clawx.read();
  // if (clawxpos>=clawxend){
  //   for (pos = clawxpos; pos>=clawxend; pos -= 1){
  //     clawx.write(pos);
  //   }}
  // else{
  //   for (pos = clawxpos; pos<=clawxend; pos += 1){
  //     clawx.write(pos);
  //   }}

  clawypos = clawy.read();
  if (clawypos>=clawyend){
    for (pos = clawypos; pos>=clawyend; pos -= 1){
      clawy.write(pos);
    }}
  else{
    for (pos = clawypos; pos<=clawyend; pos += 1){
      clawy.write(pos);
    }}

  grippos = grip.read();
  if (grippos>=gripend){
    for (pos = grippos; pos>=gripend; pos -= 1){
      grip.write(pos);
    }}
  else{
    for (pos = grippos; pos<=gripend; pos += 1){
      grip.write(pos);
    }}
}
