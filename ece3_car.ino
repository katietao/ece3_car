#include <ECE3.h>

uint16_t sensorValues[8];

float minAdjusted[8];
float maxAdjusted[8];

float error = 0;
float prevError = 0;

unsigned long prevTime = 0;

int endRun = 0;
int turnFlag = 0;

const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// Sensor weights
int weights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// PID gains
float Kp = 120;
float Kd = 900;

// Base speed
int baseSpeed = 50;

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

    prevTime = millis();

    delay(1000);
}

float readValuesandGetError()
{
    ECE3_read_IR(sensorValues);

    // Calibrate sensors
    minAdjusted[0] = sensorValues[0] - 872;
    minAdjusted[1] = sensorValues[1] - 757;
    minAdjusted[2] = sensorValues[2] - 688;
    minAdjusted[3] = sensorValues[3] - 687;
    minAdjusted[4] = sensorValues[4] - 596;
    minAdjusted[5] = sensorValues[5] - 687;
    minAdjusted[6] = sensorValues[6] - 734;
    minAdjusted[7] = sensorValues[7] - 850;

    maxAdjusted[0] = max(0.0f, minAdjusted[0] / 1.628f);
    maxAdjusted[1] = max(0.0f, minAdjusted[1] / 1.743f);
    maxAdjusted[2] = max(0.0f, minAdjusted[2] / 1.812f);
    maxAdjusted[3] = max(0.0f, minAdjusted[3] / 1.442f);
    maxAdjusted[4] = max(0.0f, minAdjusted[4] / 1.201f);
    maxAdjusted[5] = max(0.0f, minAdjusted[5] / 1.813f);
    maxAdjusted[6] = max(0.0f, minAdjusted[6] / 1.766f);
    maxAdjusted[7] = max(0.0f, minAdjusted[7] / 1.650f);

    float weightedSum = 0;
    float total = 0;

    for (int i = 0; i < 8; i++)
    {
        weightedSum += maxAdjusted[i] * weights[i];
        total += maxAdjusted[i];
    }

    // Prevent divide by zero
    if (total < 1)
        return prevError;

    error = weightedSum / total;

    // Normalize roughly to [-1,1]
    error /= 15.0;

    return error;
}

void loop()
{
    if (endRun >= 2)
    {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
        return;
    }

    readValuesandGetError();

    // Dynamic speed adjustment
    if (abs(error) < 0.25)
    {
        baseSpeed = 50;
        Kp = 120;
        Kd = 1050;
    }
    else
    {
        baseSpeed = 25;
        Kp = 70;
        Kd = 650;
    }

    // Time step
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;

    // Prevent divide by zero
    if (dt <= 0)
        dt = 0.001;

    prevTime = currentTime;

    // PID terms
    float P = Kp * error;
    float D = Kd * (error - prevError) / dt;

    float correction = P + D;

    prevError = error;

    int left_pwm = baseSpeed - correction;
    int right_pwm = baseSpeed + correction;

    left_pwm = constrain(left_pwm, 0, 255);
    right_pwm = constrain(right_pwm, 0, 255);

    // Finish detection
    bool finishDetected =
        maxAdjusted[2] > 800 &&
        maxAdjusted[3] > 800 &&
        maxAdjusted[4] > 800 &&
        maxAdjusted[5] > 800;

    if (finishDetected)
    {
        turnFlag++;
    }
    else
    {
        turnFlag = 0;
    }

    // Detect finish line consistently
    if (turnFlag > 10)
    {
        endRun++;

        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);

        delay(100);

        if (endRun < 2)
        {
            // Turn
            digitalWrite(left_dir_pin, HIGH);

            analogWrite(left_pwm_pin, 70);
            analogWrite(right_pwm_pin, 70);

            delay(300);

            digitalWrite(left_dir_pin, LOW);

            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);

            delay(100);

            // Move forward
            analogWrite(left_pwm_pin, 70);
            analogWrite(right_pwm_pin, 70);

            delay(400);
        }

        turnFlag = 0;
    }
    else
    {
        analogWrite(left_pwm_pin, left_pwm);
        analogWrite(right_pwm_pin, right_pwm);
    }

        // Debugging
    Serial.print("Error: ");
    Serial.print(error);

    Serial.print("  Left: ");
    Serial.print(left_pwm);

    Serial.print("  Right: ");
    Serial.println(right_pwm);
}