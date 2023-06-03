#include <Arduino.h>

#define MAXSPEED 255
#define SPEED 255
#define OFFSET_RIGHT 55
#define OFFSET_LEFT 55
#define DEBUGFLAG true
#define ACCELADD 0b0011101

#define ACCELADD 0b0011101

enum colours { white = 0,
               green = 1,
               red = 2,
               blue = 3,
               black = 4
};

void setMotors();
float getLinePosition(int prevLinePos, int s[4]);
void doPID();
void calibrate();
String getColour(colours c);

int leftSpeed, rightSpeed;
float linePos;
float kp, ki, kd;
float err, pErr, cErr;

int maxTurn;

colours racingColour = green;

int WHITE[4] = {700, 350, 275, 410};
int GREEN[4] = {2300, 1750, 1350, 2250};
int RED[4] = {3000, 2330, 1850, 2900};
int BLUE[4] = {2900, 2400, 2000, 2850};
int BLACK[4] = {3850, 3200, 2900, 3700};
int THRESHOLDS[4] = {(2 * WHITE[0] + GREEN[0]) / 3, (2 * WHITE[1] + GREEN[1]) / 3, (2 * WHITE[2] + GREEN[2]) / 3, (2 * WHITE[3] + GREEN[3]) / 3};

void setup() {
    Wire.begin();

    leftSpeed = 0;
    rightSpeed = 0;

    linePos = 0;
    kp = 75;
    ki = 0;  //.01;
    kd = 15;

    maxTurn = 5;

    err = 0;
    cErr = 0;
    pErr = 0;

    pinMode(2, INPUT_PULLUP);

    pinMode(20, OUTPUT);
    pinMode(6, OUTPUT);
    ledcAttachPin(21, 0);
    ledcSetup(0, 1000, 8);
    ledcAttachPin(7, 1);
    ledcSetup(1, 1000, 8);

    if (DEBUGFLAG) {
        Serial.begin(115200);
        Serial.println();
        Serial.println();
    }
    if (digitalRead(2) == 0) {
        calibrate();
    };
}

void loop() {
    int s0 = analogRead(0);
    int s1 = analogRead(1);
    int s2 = analogRead(4);
    int s3 = analogRead(5);

    if (DEBUGFLAG) {
        // Serial.println(String(s0) + " : " + String(s1) + " : " + String(s2) + " : " + String(s3));
    }
    int temp[4] = {s0, s1, s2, s3};
    linePos = getLinePosition(linePos, temp);
    doPID();
    setMotors();
    // delay(100);

    // if (DEBUGFLAG) {
    //     Serial.println("Button is: " + String(buttonState));
    // }
}

float getLinePosition(int prevLinePos, int s[4]) {
    float newLinePos = 0;

    float lineIntensity[4] = {0, 0, 0, 0};
    colours lineColour[4] = {white, white, white, white};

    for (int i = 0; i < 4; i++) {
        if (s[i] > THRESHOLDS[i]) {
            int TOP = GREEN[i], BOT = THRESHOLDS[i];
            // if (s[i] < GREEN[i]) {
            //     lineColour[i] = green;
            // } else if (s[i] < RED[i]) {  // Must be a better way to do this, but no time
            //     TOP = RED[i];
            //     lineColour[i] = red;
            // } else if (s[i] < BLUE[i]) {
            //     TOP = BLUE[i];
            //     lineColour[i] = blue;
            // } else if (s[i] < BLACK[i]) {
            //     TOP = BLACK[i];
            //     lineColour[i] = black;
            // }
            if (s[i] > (GREEN[i])) {
                lineIntensity[i] = 0;
            } else {
                lineIntensity[i] = ((float)(s[i] - BOT) / (float)(TOP - BOT));
            }
        }
    }
    if ((lineIntensity[0] == 0 && lineIntensity[1] == 0 && lineIntensity[2] == 0 && lineIntensity[3] == 0)) {
        newLinePos =
            (prevLinePos > 0)
                ? maxTurn
                : -maxTurn;
    } else {
        float normalizer = 0;

        for (int i = 0; i < 4; i++) {
            if (normalizer < abs(lineIntensity[i]))
                normalizer = abs(lineIntensity[i]);
        }
        for (int i = 0; i < 4; i++) {
            lineIntensity[i] = lineIntensity[i] / normalizer;
        }

        newLinePos -= (lineIntensity[0] > 0) ? (lineIntensity[1] > 0) ? (2 * lineIntensity[0] - lineIntensity[1]) : 3 - lineIntensity[0] : 0;
        newLinePos -= lineIntensity[1];
        newLinePos += lineIntensity[2];
        newLinePos += (lineIntensity[3] > 0) ? (lineIntensity[2] > 0) ? (2 * lineIntensity[3] - lineIntensity[2]) : 3 - lineIntensity[3] : 0;
    }

    // if (DEBUGFLAG) {
    //     Serial.println(String(newLinePos) +
    //                    " \t: " + String(s[0]) + " : " + String(s[1]) + " : " + String(s[2]) + " : " + String(s[3]) +
    //                    " \t: " + getColour(lineColour[0]) + " : " + getColour(lineColour[1]) + " : " + getColour(lineColour[2]) + " : " + getColour(lineColour[3]));
    // }

    pErr = err;
    err = newLinePos;
    cErr += err;
    return newLinePos;
}

void doPID() {
    float tempPID = kp * err + ki * cErr + kd * (err - pErr);
    if (tempPID == 0) {  // No change needed
        rightSpeed = SPEED;
        leftSpeed = SPEED;
    } else if (tempPID > 0) {  // PID pulling right - lower right motor speed
        rightSpeed = -tempPID + SPEED;
        leftSpeed = SPEED;
    } else {  // PID pulling left - lower left motor speed
        leftSpeed = tempPID + SPEED;
        rightSpeed = SPEED;
    }

    // if (DEBUGFLAG) {
    //     Serial.println(String(linePos) +
    //                    " \t: " + String(rightSpeed) + " : " + String(leftSpeed) +
    //                    " \t: " + String(tempPID) +
    //                    " \t: P- " + String(kp * err) + " : I-" + String(ki * cErr) + " : D-" + String(kd * (err - pErr)));
    // }
}

void setMotors() {
    if (rightSpeed > MAXSPEED)  // Cap out speed values
        rightSpeed = MAXSPEED;
    else if (rightSpeed < -MAXSPEED)
        rightSpeed = -MAXSPEED;
    if (leftSpeed > MAXSPEED)
        leftSpeed = MAXSPEED;
    else if (leftSpeed < -MAXSPEED)
        leftSpeed = -MAXSPEED;

    float adjustRight = OFFSET_RIGHT + (int)((float)rightSpeed * ((float)(MAXSPEED - OFFSET_RIGHT) / (float)MAXSPEED));
    float adjustLeft = OFFSET_LEFT + (int)((float)leftSpeed * ((float)(MAXSPEED - OFFSET_LEFT) / (float)MAXSPEED));

    // if (DEBUGFLAG) {
    //     Serial.println(String(linePos) +
    //                    " \t: " + String(leftSpeed) + " : " + String(rightSpeed) +
    //                    " \tL: " + String(adjustLeft) + " : " + ((leftSpeed >= 0) ? "F" : "B") +
    //                    " \tR: " + String(adjustRight) + " : " + ((rightSpeed >= 0) ? "F" : "B"));
    // }

    if (rightSpeed >= 0) {
        ledcWrite(1, MAXSPEED - adjustRight);
        digitalWrite(6, true);
    } else {
        ledcWrite(1, abs(adjustRight));
        digitalWrite(6, false);
    }
    if (leftSpeed >= 0) {
        ledcWrite(0, MAXSPEED - adjustLeft);
        digitalWrite(20, true);
    } else {
        ledcWrite(0, abs(adjustLeft));
        digitalWrite(20, false);
    }

    // if (DEBUGFLAG) {
    //     Serial.println("White: " + String(WHITE[0]) + " : " + String(WHITE[1]) + " : " + String(WHITE[2]) + " : " + String(WHITE[3]) +
    //                    " \tGreen: " + String(GREEN[0]) + " : " + String(GREEN[1]) + " : " + String(GREEN[2]) + " : " + String(GREEN[3]));
    // }
}

void calibrate() {
    ledcWrite(0, MAXSPEED);
    digitalWrite(20, true);
    ledcWrite(1, MAXSPEED);
    digitalWrite(6, true);

    delay(1000);

    for (int i = 0; i < 4; i++) {
        GREEN[i] = 0;
        WHITE[i] = 5000;
    }

    while (digitalRead(2) == 1) {
        int s0 = analogRead(0);
        int s1 = analogRead(1);
        int s2 = analogRead(4);
        int s3 = analogRead(5);
        int s[4] = {s0, s1, s2, s3};
        for (int i = 0; i < 4; i++) {
            if (s[i] < WHITE[i])
                WHITE[i] = s[i];
            if (s[i] > GREEN[i])
                GREEN[i] = s[i];
        }
    }
    delay(1000);
    while (digitalRead(2) == 1) {
        delay(1);
    }
    THRESHOLDS[0] = (2 * WHITE[0] + GREEN[0]) / 3;
    THRESHOLDS[1] = (2 * WHITE[1] + GREEN[1]) / 3;
    THRESHOLDS[2] = (2 * WHITE[2] + GREEN[2]) / 3;
    THRESHOLDS[3] = (2 * WHITE[3] + GREEN[3]) / 3;
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