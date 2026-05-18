#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

// Motor pins
#define STEP_L 3
#define DIR_L  4
#define STEP_R 5
#define DIR_R  6

// -------- PID PARAMETERS (TUNE IF NEEDED) --------
float Kp = 18.0;
float Ki = 0.6;
float Kd = 1.2;

// -------- VARIABLES --------
float angleY;
float error, lastError = 0;
float integral = 0;
float derivative;
float pidOutput;

unsigned long lastTime;
int speedDelay;

void setup() {
  Serial.begin(9600);

  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  Wire.begin();
  mpu.begin();
  delay(2000);                 // keep robot still
  mpu.calcGyroOffsets(true);

  lastTime = millis();

  Serial.println("FINAL PID Self Balancing Robot");
}

void loop() {

  // -------- TIME CALCULATION --------
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // -------- READ MPU --------
  mpu.update();
  angleY = mpu.getAngleY();

  // -------- PID CALCULATION --------
  error = angleY;               // target angle = 0
  integral += error * dt;
  integral = constrain(integral, -10, 10);   // anti-windup
  derivative = (error - lastError) / dt;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // -------- DIRECTION CONTROL --------
  if (pidOutput > 0) {
    digitalWrite(DIR_L, HIGH);
    digitalWrite(DIR_R, HIGH);
  } else {
    digitalWrite(DIR_L, LOW);
    digitalWrite(DIR_R, LOW);
  }

  // -------- SPEED CONTROL --------
  pidOutput = abs(pidOutput);
  pidOutput = constrain(pidOutput, 0, 400);

  speedDelay = map(pidOutput, 0, 400, 2000, 350);
  speedDelay = constrain(speedDelay, 350, 2000);

  // -------- STEP PULSE --------
  digitalWrite(STEP_L, HIGH);
  digitalWrite(STEP_R, HIGH);
  delayMicroseconds(speedDelay);
  digitalWrite(STEP_L, LOW);
  digitalWrite(STEP_R, LOW);
  delayMicroseconds(speedDelay);

  // -------- DEBUG --------
  Serial.print("Angle: ");
  Serial.print(angleY);
  Serial.print(" | PID: ");
  Serial.println(pidOutput);
}