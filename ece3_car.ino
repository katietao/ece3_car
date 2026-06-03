#include <ECE3.h>

uint16_t sensorValues[8];
float minAdjusted[8];
float maxAdjusted[8];

float error = 0;
float prevError = 0;

int endRun = 0;
int turnFlag = 0;
bool clearedStart = false; 

// ================= MOTOR PINS =================
const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ================= SENSOR WEIGHTS =================
int W0 = -8, W1 = -4, W2 = -2, W3 = -1;
int W4 = 1,   W5 = 2,  W6 = 4,  W7 = 8;

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
    delay(2000); // Give user time to place robot
}

float readValuesandGetError() {
    ECE3_read_IR(sensorValues);

    // Calibration (Specific to your robot's min readings)
    minAdjusted[0] = sensorValues[0] - 872;
    minAdjusted[1] = sensorValues[1] - 757;
    minAdjusted[2] = sensorValues[2] - 688;
    minAdjusted[3] = sensorValues[3] - 687;
    minAdjusted[4] = sensorValues[4] - 596;
    minAdjusted[5] = sensorValues[5] - 687;
    minAdjusted[6] = sensorValues[6] - 734;
    minAdjusted[7] = sensorValues[7] - 850;

    for (int i = 0; i < 8; i++) {
        if (minAdjusted[i] < 0) minAdjusted[i] = 0;
    }

    // Gain Normalization
    maxAdjusted[0] = minAdjusted[0] / 1.628;
    maxAdjusted[1] = minAdjusted[1] / 1.743;
    maxAdjusted[2] = minAdjusted[2] / 1.812;
    maxAdjusted[3] = minAdjusted[3] / 1.442;
    maxAdjusted[4] = minAdjusted[4] / 1.201;
    maxAdjusted[5] = minAdjusted[5] / 1.813;
    maxAdjusted[6] = minAdjusted[6] / 1.766;
    maxAdjusted[7] = minAdjusted[7] / 1.650;

    // Weighted Sum
    error = (maxAdjusted[0]*W0 + maxAdjusted[1]*W1 + maxAdjusted[2]*W2 + maxAdjusted[3]*W3 +
             maxAdjusted[4]*W4 + maxAdjusted[5]*W5 + maxAdjusted[6]*W6 + maxAdjusted[7]*W7);

    error = (error / 4.0) / 2594.485;
    return error;
}

void performTurn() {
    // 1. Brief pause to settle momentum
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(40); 

    // 2. Set pins for a left tank turn (pivot)
    digitalWrite(left_dir_pin, HIGH); 
    digitalWrite(right_dir_pin, LOW);
    analogWrite(left_pwm_pin, 180);
    analogWrite(right_pwm_pin, 180);

    // PHASE 1: Blind rotation (Move off the current black bar)
    delay(330); 

    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, LOW);
    delay(10);

    // Go forward very briefly to go past the black line
    analogWrite(left_pwm_pin, 100);
    analogWrite(right_pwm_pin, 100);
    delay(300);

    // 3. Stop and reset direction for forward driving
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    delay(40);
}

void loop() {
    // STOP PERMANENTLY after two finish line detections
    if (endRun >= 2) {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        return;
    }

    readValuesandGetError();

    // GAIN SCHEDULING: Adjust PID based on how far off we are
    if (abs(error) < 0.08) { // straight
        baseSpeed = 240; Kp = 90; Kd = 1300;
    } 
    else if (abs(error) < 0.13) { // transition to  curves
        baseSpeed = 210; Kp = 260; Kd = 3000;
    }
    else { // curves
        baseSpeed = 165; Kp = 330; Kd = 4200;
    }

    float totalPID = (Kp * error) + (Kd * (error - prevError));
    prevError = error;
    totalPID = constrain(totalPID, -150, 150);

    int left_pwm  = constrain(baseSpeed - totalPID, 0, 255);
    int right_pwm = constrain(baseSpeed + totalPID, 0, 255);

    // LOST LINE RECOVERY: If totally off, turn hard
    if (error > 0.95) { left_pwm = 20; right_pwm = 190; }
    else if (error < -0.95) { left_pwm = 190; right_pwm = 20; }

    // =====================================================
    // LOGIC: START vs. TURN vs. FINISH
    // =====================================================

    // 1. Look for white floor to confirm we've left the starting line
    if (!clearedStart) {
        if (maxAdjusted[3] < 400 && maxAdjusted[4] < 400) {
            clearedStart = true;
        }
    }

    // 2. Detection of the Black Cross-Bar (Finish/Turn Bar)
    bool finishDetected = (maxAdjusted[2] > 600 && maxAdjusted[3] > 800 && 
                           maxAdjusted[4] > 800 && maxAdjusted[5] > 600);

    if (finishDetected) {
        turnFlag++;
    } else {
        turnFlag = 0;
    }

    // 3. Action based on detection
    if (turnFlag >= 2) { // Fast response (adjust to 4 or 8 if needed)
        endRun++;
        if (endRun < 2) {
            performTurn();
            clearedStart = false; // Must see white space after U-turn before finishing
        } else {
            // This is the second bar (Actual Finish)
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
        }
        turnFlag = 0;
    } else {
        // Normal Driving
        analogWrite(left_pwm_pin, left_pwm);
        analogWrite(right_pwm_pin, right_pwm);
    }

//   // 1. Refresh sensor data
//   readValuesandGetError(); 

//   // 2. Print RAW values (to check for dead sensors)
//   Serial.print("RAW: ");
//   for (int i = 0; i < 8; i++) {
//     Serial.print(sensorValues[i]);
//     Serial.print("\t");
//   }

//   // 3. Print ADJUSTED values (to check calibration & 800 threshold)
//   Serial.print("| ADJ: ");
//   for (int i = 0; i < 8; i++) {
//     Serial.print((int)maxAdjusted[i]); // Cast to int to make it easier to read
//     Serial.print("\t");
//   }

//   // 4. Print Error (to check PID logic)
//   Serial.print("| Err: ");
//   Serial.println(error, 3); // Print with 3 decimal places

//   delay(200);
}