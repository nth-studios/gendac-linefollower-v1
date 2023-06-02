#include <Arduino.h>
#define MAXSPEED 255
#define SPEED 100
#define OFFSET_RIGHT 200
#define OFFSET_LEFT 194
#define DEBUGFLAG true

#define THRESHOLDLF 0
#define THRESHOLDLC 0
#define THRESHOLDRC 0
#define THRESHOLDRF 0

int leftSpeed, rightSpeed;

float linePos;
float kp, ki, kd;
float err, pErr, cErr;

void setMotors(int rSpeed, int lSpeed);
float getLinePosition(float prev, int lF, int lC, int rC, int rF);
void doPID(float linePos);

void setup() {
    leftSpeed = 0;
    rightSpeed = 0;

    linePos = 0;
    kp = 1;
    ki = 0;
    kd = 0;

    err = 0;
    cErr = 0;
    pErr = 0;

    pinMode(20, OUTPUT);
    pinMode(6, OUTPUT);
    ledcAttachPin(21, 0);
    ledcSetup(0, 1000, 8);
    ledcAttachPin(7, 1);
    ledcSetup(1, 1000, 8);

    if (DEBUGFLAG) {
        Serial.begin(11500);
        Serial.println();
        Serial.println();
    }
}

void loop() {
    int temp1 = analogRead(0);
    int temp2 = analogRead(1);
    int temp3 = analogRead(4);
    int temp4 = analogRead(5);

    if (DEBUGFLAG) {
        Serial.println(String(temp1) + " : " + String(temp2) + " : " + String(temp3) + " : " + String(temp4));
    }
    linePos = getLinePosition(linePos, temp1, temp2, temp3, temp4);

    delay(1000);
}

float getLinePosition(float prev, int lF, int lC, int rC, int rF) {
    int newLinePos = 0;
    pErr = err;
    err = newLinePos - prev;
    cErr += err;
    return newLinePos;
}

void doPID(int linePos) {
    float tempPID = kp * err + ki * cErr + kd * (err - pErr);
    rightSpeed = tempPID + SPEED;
    leftSpeed = tempPID + SPEED;
}

void setMotors() {
    if (DEBUGFLAG) {
        // Serial.println(String(rightSpeed) + " : " + String(leftSpeed));
    }

    if (rightSpeed > 0) {
        ledcWrite(0, OFFSET_RIGHT - rightSpeed);
        digitalWrite(20, true);
    } else {
        ledcWrite(0, OFFSET_RIGHT - (MAXSPEED - abs(rightSpeed)));
        digitalWrite(20, false);
    }

    if (leftSpeed > 0) {
        ledcWrite(0, OFFSET_RIGHT - leftSpeed);
        digitalWrite(20, true);
    } else {
        ledcWrite(0, OFFSET_RIGHT - (MAXSPEED - abs(leftSpeed)));
        digitalWrite(20, false);
    }
}