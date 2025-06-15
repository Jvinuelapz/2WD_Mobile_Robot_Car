#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MOVING_AVG_SIZE 10

Adafruit_MPU6050 mpu;

float pitch_buffer[MOVING_AVG_SIZE] = {0};
float roll_buffer[MOVING_AVG_SIZE] = {0};
int buffer_index = 0;

float pitch_cum_avg = 0;
float roll_cum_avg = 0;
unsigned long sample_count = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  float roll  = atan2(-ax, az) * 180 / PI;

  // Moving Average
  pitch_buffer[buffer_index] = pitch;
  roll_buffer[buffer_index] = roll;

  float pitch_movavg = 0, roll_movavg = 0;
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    pitch_movavg += pitch_buffer[i];
    roll_movavg += roll_buffer[i];
  }
  pitch_movavg /= MOVING_AVG_SIZE;
  roll_movavg  /= MOVING_AVG_SIZE;

  buffer_index = (buffer_index + 1) % MOVING_AVG_SIZE;

  // Cumulative Average
  sample_count++;
  pitch_cum_avg += (pitch - pitch_cum_avg) / sample_count;
  roll_cum_avg  += (roll - roll_cum_avg) / sample_count;

  Serial.print("Pitch:");
  Serial.print(pitch); 
  Serial.print(",");

  Serial.print("pitch_movavg:");
  Serial.print(pitch_movavg); 
  Serial.print(",");

  Serial.print("pitch_cum_avg:");
  Serial.print(pitch_cum_avg); 
  Serial.print(",");

  Serial.print("roll:");
  Serial.print(roll); 
  Serial.print(",");

  Serial.print("roll_movavg:");
  Serial.print(roll_movavg); 
  Serial.print(",");

  Serial.print("roll_cum_avg:");
  Serial.println(roll_cum_avg);

  delay(100);
}



