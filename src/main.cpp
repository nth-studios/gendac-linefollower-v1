#include <Arduino.h>
#define MAXSPEED 255
#define SPEED 100
#define OFFSET_RIGHT 200
#define OFFSET_LEFT 194
#define DEBUGFLAG true

int leftSpeed, rightSpeed;
float linePos;
float kp, ki, kd;
float err, pErr, cErr;

enum colours { white = 0,
               green = 1,
               red = 2,
               blue = 3,
               black = 4
};

colours racingColour = green;

int WHITE[4] = {870, 1300, 630, 700};
int RED[4] = {3200, 3050, 2700, 3020};
int BLUE[4] = {3230, 3290, 3050, 3100};
int GREEN[4] = {2620, 2950, 2680, 2570};
int BLACK[4] = {3900, 3725, 3520, 3780};
int THRESHOLDS[4] = {(WHITE[0] + GREEN[0]) / 2, (WHITE[1] + GREEN[1]) / 2, (WHITE[2] + GREEN[2]) / 2, (WHITE[3] + GREEN[3]) / 2};

void setMotors();
float getLinePosition(float prev, int s0, int s1, int s2, int s3);
void doPID();

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
    doPID();
    setMotors();
    delay(100);
}

float getLinePosition(float prev, int s[4]) {
    float newLinePos = 0;

    int lineIntensity[4] = {0, 0, 0, 0};
    colours lineColour[4] = {white, white, white, white};

    for (int i = 0; i < 4; i++) {
        if (s[i] > THRESHOLDS[i]) {
            int TOP = GREEN[i], BOT = THRESHOLDS[i];
            lineColour[i] = green;
            if (s[i] < RED[i]) {  // Must be a better way to do this, but no time
                BOT = GREEN[i];
                TOP = RED[i];
                lineColour[i] = red;
            } else if (s[i] < BLUE[i]) {
                BOT = RED[i];
                TOP = BLUE[i];
                lineColour[i] = blue;
            } else if (s[i] < BLACK[i]) {
                BOT = BLUE[i];
                TOP = BLACK[i];
                lineColour[i] = black;
            }

            lineIntensity[i] = (s[i] - BOT) / (TOP - BOT);
        }
    }

    newLinePos -= (lineIntensity[0] > 0) ? (2 * lineIntensity[0] - lineIntensity[1]) : 0;
    newLinePos -= lineIntensity[1];
    newLinePos += lineIntensity[2];
    newLinePos += (lineIntensity[3] > 0) ? (2 * lineIntensity[3] - lineIntensity[2]) : 0;

    pErr = err;
    err = newLinePos - prev;
    cErr += err;
    return newLinePos;
}

void doPID() {
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