#include<Servo.h>
#define servopin 9

#define opendegree 35
#define closedegree 90


Servo servo;
int i = 0;
int[] buffer = new int[8]; // a buffer to store incoming communications in

void setup() {
    Serial.begin(9600);

    

    pinMode(servopin, OUTPUT);
    servo.attach(servopin);

    servo.write(90);
}

void loop() {
    // Wait for serial communication
    while( Serial.available() == 0 ) {
        delay(1);
    }

    int c = Serial.read();
    if (c == int('c')) {
        servo.write(closedegree)
    } else if (c == int('o')) {
        servo.write(opendegree)
    }
    // buffer[i] = c;
    // i++;
    // if (c == int('\n')) {
    //     process_message();
    // }
}

void process_message() {
    String message = "";

    for (int c:buffer) {
        if (c == int('\n')) break;

        message += char(i);
    }

    if (message == "open") {
        servo.write(opendegree);
    } else if (message == "close" {
        servo.write(closedegree);
    }

    // Clear buffer
    for (int j = 0; j<buffer.length; j++) {
        buffer[j] = 0;
    }
    i = 0
}