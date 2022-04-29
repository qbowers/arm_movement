#include<Servo.h>
#define servopin 9

Servo servo;
void setup() {
    Serial.begin(9600);
    pinMode(servopin, OUTPUT);
    servo.attach(servopin);
}

void loop() {
    
}