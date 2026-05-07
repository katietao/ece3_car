
#include <ECE3.h>

uint16_t sensorValues[8];

float minAdjusted[8];
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

int W0 = -15;
int W1 = -14;
int W2 = -12;
int W3 = -8;
int W4 = 8;
int W5 = 12;
int W6 = 14;
int W7 = 15;

// ================= PID =================

float Kp = 45;
float Kd = 180;

// Conservative testing speed
int baseSpeed = 120;

// =====================================================

void setup()
{
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

    // delay(1000);
}

// =====================================================

float readValuesandGetError()
{
    ECE3_read_IR(sensorValues);

    // ===== SENSOR CALIBRATION =====

    minAdjusted[0] = sensorValues[0] - 872;
    minAdjusted[1] = sensorValues[1] - 757;
    minAdjusted[2] = sensorValues[2] - 688;
    minAdjusted[3] = sensorValues[3] - 687;
    minAdjusted[4] = sensorValues[4] - 596;
    minAdjusted[5] = sensorValues[5] - 687;
    minAdjusted[6] = sensorValues[6] - 734;
    minAdjusted[7] = sensorValues[7] - 850;

    // Prevent negatives
    for (int i = 0; i < 8; i++)
    {
        if (minAdjusted[i] < 0)
            minAdjusted[i] = 0;
    }

    // ===== GAIN NORMALIZATION =====

    maxAdjusted[0] = minAdjusted[0] / 1.628;
    maxAdjusted[1] = minAdjusted[1] / 1.743;
    maxAdjusted[2] = minAdjusted[2] / 1.812;
    maxAdjusted[3] = minAdjusted[3] / 1.442;
    maxAdjusted[4] = minAdjusted[4] / 1.201;
    maxAdjusted[5] = minAdjusted[5] / 1.813;
    maxAdjusted[6] = minAdjusted[6] / 1.766;
    maxAdjusted[7] = minAdjusted[7] / 1.650;

    // ===== RAW WEIGHTED SUM =====

    error =
        (maxAdjusted[0] * W0) +
        (maxAdjusted[1] * W1) +
        (maxAdjusted[2] * W2) +
        (maxAdjusted[3] * W3) +
        (maxAdjusted[4] * W4) +
        (maxAdjusted[5] * W5) +
        (maxAdjusted[6] * W6) +
        (maxAdjusted[7] * W7);

    error /= 8.0;

    // Empirical normalization
    error /= 2936.764;

    return error;
}

// =====================================================

void performTurn()
{

    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);

    delay(150);

    // pivot left
    digitalWrite(left_dir_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);

    analogWrite(left_pwm_pin, 55);
    analogWrite(right_pwm_pin, 55);


    // wait until robot leaves track
    while (true)
    {
        ECE3_read_IR(sensorValues);

        bool stillOnFinish =
            sensorValues[2] > 2000 &&
            sensorValues[3] > 2000 &&
            sensorValues[4] > 2000 &&
            sensorValues[5] > 2000;

        if (!stillOnFinish)
        {
            break;
        }
    }

    // turn until sensing line again
    while (true)
    {
        ECE3_read_IR(sensorValues);

        bool lineDetected =
            sensorValues[3] > 2000 ||
            sensorValues[4] > 2000;

        if (lineDetected)
        {
            break;
        }
    }

    // Stop turning
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);

    // Restore forward direction
    digitalWrite(left_dir_pin, LOW);
    digitalWrite(right_dir_pin, LOW);

    delay(100);
}
// =====================================================

void loop()
{
    // Stop after second finish
    if (endRun >= 2)
    {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        return;
    }

    // Read sensors
    readValuesandGetError();

    if (abs(error) < 0.15)
    {
        baseSpeed = 130;
        Kp = 40;
        Kd = 140;
    }
    else if (abs(error) < 0.40)
    {
        baseSpeed = 110;
        Kp = 50;
        Kd = 180;
    }
    else
    {
        baseSpeed = 90;
        Kp = 65;
        Kd = 250;
    }

    float propTerm = Kp * error;
    
    float derivTerm = Kd * (error - prevError);

    float totalPID = propTerm + derivTerm;

    prevError = error;

    // Limit steering
    totalPID = constrain(totalPID, -70, 70);

    // Motor PWM
    int left_pwm  = baseSpeed - totalPID;
    int right_pwm = baseSpeed + totalPID;

    // Clamp PWM
    left_pwm = constrain(left_pwm, 0, 255);
    right_pwm = constrain(right_pwm, 0, 255);

    // =====================================================
    // LOST LINE RECOVERY
    // =====================================================

    if (error > 0.95)
    {
        left_pwm = 50;
        right_pwm = 140;
    }
    else if (error < -0.95)
    {
        left_pwm = 140;
        right_pwm = 50;
    }

    bool finishDetected =
        (maxAdjusted[3] > 500 &&
        maxAdjusted[4] > 500 &&
        maxAdjusted[5] > 500 &&
        maxAdjusted[6] > 500);

    if (finishDetected)
    {
        turnFlag++;
    }
    else
    {
        turnFlag = 0;
    }

    // Require stable detection
    if (turnFlag > 25)
    {
        endRun++;

        if (endRun < 2)
        {
            performTurn();
        }
        else
        {
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
        }

        turnFlag = 0;
    }
    else
    {
        analogWrite(left_pwm_pin, left_pwm);
        analogWrite(right_pwm_pin, right_pwm);
    }

    // // =====================================================
    // // DEBUG
    // // =====================================================

    // Serial.print("Error: ");
    // Serial.print(error, 4);

    // Serial.print("  Left PWM: ");
    // Serial.print(left_pwm);

    // Serial.print("  Right PWM: ");
    // Serial.println(right_pwm);

    // delay(5);
}
