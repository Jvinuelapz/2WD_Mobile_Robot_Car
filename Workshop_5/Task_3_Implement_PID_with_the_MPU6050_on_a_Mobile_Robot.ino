#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MOTOR1_PIN1 4  // Motor 1 control pin 1
#define MOTOR1_PIN2 5  // Motor 1 control pin 2
#define MOTOR2_PIN1 25 // Motor 2 control pin 1
#define MOTOR2_PIN2 26 // Motor 2 control pin 2

// PID constants (ajustados para suavizar)
#define Kp 0.5
#define Ki 0.05
#define Kd 0.05
#define DT 100  // Loop interval in ms

// PWM mínimos para que los motores se muevan realmente
#define MIN_PWM 145
#define MAX_PWM 255

Adafruit_MPU6050 mpu;

float motorSpeed = 0;
float prevError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
}

void loop() {
  float currentAngle = getAngleX();
  float error = 0.0 - currentAngle;

  // Dead Zone
  if (abs(error) < 2.0) {
    motorSpeed = 0;
    integral = 0;
  } else {
    float P = Kp * error;
    integral += Ki * error * (DT / 1000.0);
    float D = Kd * (error - prevError) / (DT / 1000.0);
    motorSpeed = constrain(P + integral + D, -255, 255);
  }

  prevError = error;
  setMotorSpeed((int)motorSpeed);

  delay(DT);
}

float getAngleX() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  float offset = 0.45; 
  angleX += offset;

  Serial.print("AngleX: ");
  Serial.print(angleX);
  Serial.print(" | ");

  return angleX;
}

// Función que ajusta el PWM según el valor del PID con umbral mínimo
int getEffectivePWM(int speed, int minPWM, int maxPWM) {
  int absSpeed = abs(speed);
  if (absSpeed == 0) return 0;

  // Escalado proporcional de PID a rango PWM útil
  int effective = map(absSpeed, 1, 255, minPWM, maxPWM);
  return constrain(effective, minPWM, maxPWM);
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  int pwm = getEffectivePWM(speed, MIN_PWM, MAX_PWM);

  Serial.print("Speed: ");
  Serial.print(speed);

  if (speed > 0) {
    analogWrite(MOTOR1_PIN1, pwm);
    analogWrite(MOTOR1_PIN2, 0);
    analogWrite(MOTOR2_PIN1, pwm);
    analogWrite(MOTOR2_PIN2, 0);
    Serial.print(" | Forward: ");
    Serial.println(pwm);
  } else if (speed < 0) {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, pwm);
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, pwm);
    Serial.print(" | Backward: ");
    Serial.println(pwm);
  } else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, 0);
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, 0);
    Serial.println(" | Motors Stopped");
  }
}

