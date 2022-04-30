#include<Servo.h>
#define servopin 9

#define opendegree 35
#define closedegree 90


Servo servo;

void setup() {
    Serial.begin(9600);
    
    pinMode(servopin, OUTPUT);
    pinMode(13, OUTPUT);
    servo.attach(servopin);

    servo.write(90);
}

void loop() {
    // Wait for serial communication
    while( Serial.available() == 0 ) {
        delay(1);
    }

    int c = Serial.read();
    Serial.println(c);
    if (c == int('c')) {
        servo.write(closedegree);
        blink(5, 200);
    } else if (c == int('o')) {
        servo.write(opendegree);
        blink(10, 100);
    }
}

void blink(int n, int d) {
    for (int i = 0; i < n; i++) {
        digitalWrite(13,HIGH);
        delay(d);
        digitalWrite(13,LOW);
        delay(d);
    }
}
