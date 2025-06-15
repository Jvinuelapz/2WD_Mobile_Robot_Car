const int INPUT_PIN = 33;       // Analog pin for reading sensor input (ADC1_CH5)
const int OUTPUT_PIN = 32;      // Digital pin for output control

double dt, last_time;           // Time interval variables
double integral, previous, output = 0; // PID variables
double kp, ki, kd;              // PID constants
double setpoint = 80.00;        // Desired setpoint

void setup() {
  kp = 0.5;                     // Proportional constant
  ki = 0.20;                    // Integral constant
  kd = 0.002;                   // Derivative constant
  last_time = 0;                // Initialize last time

  Serial.begin(115200);           // Start serial communication
  analogWrite(OUTPUT_PIN, 0);   // Initialize output pin (PWM to 0)

  // Print initial setpoint and simulated sensor value
  for (int i = 0; i < 50; i++) {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }

  delay(100);                   // Delay before starting loop
}

void loop() {
  double now = millis();                          // Current time
  dt = (now - last_time) / 1000.0;                // Time interval in seconds
  last_time = now;                                // Update last time

  double actual = map(analogRead(INPUT_PIN), 0, 4095, 0, 255); // ESP32 ADC range is 0–4095
  double error = setpoint - actual;              // Calculate error

  output = pid(error);                           // Compute PID output
  output = constrain(output, 0, 255);            // Ensure output stays within PWM range

  analogWrite(OUTPUT_PIN, output);               // Set output value (PWM)

  // Print Setpoint VS Actual value
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  delay(100);                                     // Delay before next loop iteration
}

double pid(double error) {
  double proportional = error;                   // Proportional term
  integral += error * dt;                        // Integral term
  double derivative = (error - previous) / dt;   // Derivative term
  previous = error;                              // Update previous error

  double output = (kp * proportional) + (ki * integral) + (kd * derivative); // Compute PID output
  return output;
}
