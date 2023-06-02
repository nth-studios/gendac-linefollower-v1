#include <Arduino.h>
#define MAXSPEED 255
#define SPEED 100
#define OFFSET_RIGHT 200
#define OFFSET_LEFT 194
#define DEBUGFLAG true

enum colours { white = 0,
               green = 1,
               red = 2,
               blue = 3,
               black = 4
};

void setMotors();
float getLinePosition(int prevLinePos, int s[4]);
void doPID();
String getColour(colours c);

int leftSpeed, rightSpeed;
float linePos;
float kp, ki, kd;
float err, pErr, cErr;

colours racingColour = green;

int WHITE[4] = {700, 350, 275, 410};
int GREEN[4] = {2300, 1750, 1350, 2250};
int RED[4] = {3000, 2330, 1850, 2900};
int BLUE[4] = {2900, 2400, 2000, 2850};
int BLACK[4] = {3850, 3200, 2900, 3700};
int THRESHOLDS[4] = {(WHITE[0] + GREEN[0]) / 2, (WHITE[1] + GREEN[1]) / 2, (WHITE[2] + GREEN[2]) / 2, (WHITE[3] + GREEN[3]) / 2};

void setup() {
    leftSpeed = 0;
    rightSpeed = 0;

    linePos = 0;
    kp = 10;
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
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(4);
    int s3 = analogRead(5);

    if (DEBUGFLAG) {
        // Serial.println(String(temp1) + " : " + String(temp2) + " : " + String(temp3) + " : " + String(temp4));
    }
    int temp[4] = {s0, s1, s2, s3};
    linePos = getLinePosition(linePos, temp);
    doPID();
    setMotors();
    delay(100);
}

float getLinePosition(int prevLinePos, int s[4]) {
    float newLinePos = 0;

    float lineIntensity[4] = {0, 0, 0, 0};
    colours lineColour[4] = {white, white, white, white};

    for (int i = 0; i < 4; i++) {
        if (s[i] > THRESHOLDS[i]) {
            int TOP = GREEN[i], BOT = THRESHOLDS[i];
            if (s[i] < GREEN[i]) {
                lineColour[i] = green;
            } else if (s[i] < RED[i]) {  // Must be a better way to do this, but no time
                TOP = RED[i];
                lineColour[i] = red;
            } else if (s[i] < BLUE[i]) {
                TOP = BLUE[i];
                lineColour[i] = blue;
            } else if (s[i] < BLACK[i]) {
                TOP = BLACK[i];
                lineColour[i] = black;
            }

            lineIntensity[i] = (float)(s[i] - BOT) / (float)(TOP - BOT);
        }
    }

    newLinePos -= (lineIntensity[0] > 0) ? (lineIntensity[1] > 0) ? (2 * lineIntensity[0] - lineIntensity[1]) : 3 - lineIntensity[0] : 0;
    newLinePos -= lineIntensity[1];
    newLinePos += lineIntensity[2];
    newLinePos += (lineIntensity[3] > 0) ? (lineIntensity[2] > 0) ? (2 * lineIntensity[3] - lineIntensity[2]) : 3 - lineIntensity[3] : 0;

    // if (DEBUGFLAG) {
    //     Serial.println(String(newLinePos) +
    //                    " \t: " + String(s[0]) + " : " + String(s[1]) + " : " + String(s[2]) + " : " + String(s[3]) +
    //                    " \t: " + getColour(lineColour[0]) + " : " + getColour(lineColour[1]) + " : " + getColour(lineColour[2]) + " : " + getColour(lineColour[3]));
    // }

    newLinePos = (lineIntensity[0] == 0 && lineIntensity[1] == 0 && lineIntensity[2] == 0 && lineIntensity[3] == 0)
                     ? (prevLinePos > 0)
                           ? 5
                           : -5
                     : newLinePos;

    pErr = err;
    err = newLinePos;
    cErr += err;
    return newLinePos;
}

void doPID() {
    float tempPID = kp * err + ki * cErr + kd * (err - pErr);
    rightSpeed = -tempPID + SPEED;
    leftSpeed = tempPID + SPEED;
    // if (DEBUGFLAG) {
    //     Serial.println(String(linePos) +
    //                    " \t: " + String(rightSpeed) + " : " + String(leftSpeed) +
    //                    " \t: " + String(tempPID) +
    //                    " \t: P- " + String(kp * err) + " : I-" + String(ki * cErr) + " : D-" + String(kd * (err - pErr)));
    // }
}

void setMotors() {
    if (rightSpeed > 255)  // Cap out speed values
        rightSpeed = 255;
    else if (rightSpeed < -255)
        rightSpeed = -255;
    if (leftSpeed > 255)
        leftSpeed = 255;
    else if (leftSpeed < -255)
        leftSpeed = -255;

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

String getColour(colours c) {
    switch (c) {
        case white:
            return " ";
            break;
        case green:
            return "G";
            break;
        case red:
            return "R";
            break;
        case blue:
            return "B";
            break;
        case black:
            return "X";
            break;
        default:
            return "?";
            break;
    }
}