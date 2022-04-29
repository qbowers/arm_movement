#include<Servo.h>
#define servopin 9

Servo servo;
void setup() {
    pinMode(servopin, OUTPUT);
    servo.attach(9);
}

void loop() {
    delay(2000);
    servo.write(0);

    
    delay(2000);
    servo.write(90);

    
    delay(2000);
    servo.write(180);
    

    delay(2000);
    servo.write(90);
}