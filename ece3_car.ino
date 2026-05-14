#include <ECE3.h>

uint16_t sensorValues[8];
float maxAdjusted[8];

float error = 0;
float prevError = 0;

int endRun = 0;
int turnFlag = 0;

// ================= MOTOR PINS =================
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ================= SENSOR WEIGHTS =================
int W0 = -15, W1 = -14, W2 = -12, W3 = -8;
int W4 = 8,   W5 = 12,  W6 = 14,  W7 = 15;

// ================= PID & SPEED =================
float Kp, Kd;
int baseSpeed;

void setup() {
    ECE3_Init();

    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);
    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, LOW);
    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(right_nslp_pin, HIGH);

    Serial.begin(9600);

    delay(2000);
}

// ================= ERROR CALCULATION =================
float readValuesandGetError() {
    ECE3_read_IR(sensorValues);

    // Subtraction of min calibration values
    float minAdj[8];
    minAdj[0] = sensorValues[0] - 872;
    minAdj[1] = sensorValues[1] - 757;
    minAdj[2] = sensorValues[2] - 688;
    minAdj[3] = sensorValues[3] - 687;
    minAdj[4] = sensorValues[4] - 596;
    minAdj[5] = sensorValues[5] - 687;
    minAdj[6] = sensorValues[6] - 734;
    minAdj[7] = sensorValues[7] - 850;

    for (int i = 0; i < 8; i++) {
        if (minAdj[i] < 0) minAdj[i] = 0;
    }

    // Gain Normalization
    maxAdjusted[0] = minAdj[0] / 1.628;
    maxAdjusted[1] = minAdj[1] / 1.743;
    maxAdjusted[2] = minAdj[2] / 1.812;
    maxAdjusted[3] = minAdj[3] / 1.442;
    maxAdjusted[4] = minAdj[4] / 1.201;
    maxAdjusted[5] = minAdj[5] / 1.813;
    maxAdjusted[6] = minAdj[6] / 1.766;
    maxAdjusted[7] = minAdj[7] / 1.650;

    // Weighted Sum
    error = 0;
    for (int i = 0; i < 8; i++) {
        int weights[] = {W0, W1, W2, W3, W4, W5, W6, W7};
        error += (maxAdjusted[i] * weights[i]);
    }

    error = (error / 8.0) / 2936.764;
    return error;
}

// ================= 180 DEGREE PIVOT =================
void performTurn() {
    // Stop briefly
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(100);

    // Pivot Left (Tank Turn)
    digitalWrite(left_dir_pin, HIGH); 
    digitalWrite(right_dir_pin, LOW);
    analogWrite(left_pwm_pin, 60);
    analogWrite(right_pwm_pin, 60);

    // PHASE 1: Pivot until we LEAVE the black bar (see white)
    while (true) {
        ECE3_read_IR(sensorValues);
        if (sensorValues[3] < 300 && sensorValues[4] < 300) {
            break;
        }
    }

    // PHASE 2: Keep pivoting through the white space until we HIT the black line
    while (true) {
        readValuesandGetError(); // This updates the maxAdjusted array
        // Wait until the center sensors see the dark line again
        if (maxAdjusted[3] > 500 || maxAdjusted[4] > 500) {
            break; 
        }
    }

    // PHASE 3: Now that we are on the line, pivot until it's perfectly centered
    while (true) {
        float turnError = readValuesandGetError();
        if (abs(turnError) < 0.15) {
            break;
        }
    }

    // Stop and Reset Direction
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    digitalWrite(left_dir_pin, LOW);
    delay(100);
}

// ================= MAIN LOOP =================
void loop() {
    if (endRun >= 2) {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        return;
    }

    readValuesandGetError();

    // Gain Scheduling based on error severity
    if (abs(error) < 0.15) {
        baseSpeed = 135; Kp = 40; Kd = 140;
    } else if (abs(error) < 0.40) {
        baseSpeed = 115; Kp = 50; Kd = 180;
    } else {
        baseSpeed = 95;  Kp = 70; Kd = 260;
    }

    // PD Calculation
    float totalPID = (Kp * error) + (Kd * (error - prevError));
    prevError = error;

    totalPID = constrain(totalPID, -80, 80);

    int left_pwm  = constrain(baseSpeed - totalPID, 0, 255);
    int right_pwm = constrain(baseSpeed + totalPID, 0, 255);

    // Lost Line Recovery (Hard Steering)
    if (error > 0.95) { left_pwm = 40; right_pwm = 140; }
    else if (error < -0.95) { left_pwm = 140; right_pwm = 40; }

    // Finish Line Detection
    // Requires center-right sensors to be dark
    bool finishDetected = (maxAdjusted[3] > 600 && maxAdjusted[4] > 600 && maxAdjusted[5] > 600);

    // Ignore detection for the first 1.5 seconds to escape the start line
    if (finishDetected && millis() > 1500) {
        turnFlag++;
    } else {
        turnFlag = 0;
    }

    // Stable detection trigger
    if (turnFlag > 8) {
        endRun++;
        if (endRun < 2) {
            performTurn();
        } else {
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
        }
        turnFlag = 0;
    } else {
        analogWrite(left_pwm_pin, left_pwm);
        analogWrite(right_pwm_pin, right_pwm);
    }
}