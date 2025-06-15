// RIGHT WHEEL
int MOTOR_B1 = 25;  
int MOTOR_B2 = 26;  

// LEFT WHEEL
int MOTOR_A1 = 4; 
int MOTOR_A2 = 5;  

// Motor Speeds (140 - 255)
int PWM_SPEED = 140;
int dir = 0;       // Direction: 0 -> Forward | 1 -> Backward

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
}

void loop() {

  Serial.print("PWM_SPEED: ");
  Serial.print(PWM_SPEED);
  Serial.print(" | Direction: ");
  Serial.println(dir);

  if (dir == 0) {
      analogWrite(MOTOR_A1, PWM_SPEED);
      analogWrite(MOTOR_A2, 0);
      analogWrite(MOTOR_B1, PWM_SPEED);
      analogWrite(MOTOR_B2, 0); 
  } else {
      analogWrite(MOTOR_A1, 0);
      analogWrite(MOTOR_A2, PWM_SPEED);
      analogWrite(MOTOR_B1, 0);
      analogWrite(MOTOR_B2, PWM_SPEED);
  }

  PWM_SPEED += 10;

  // Speed limit, change of direction
  if (PWM_SPEED >= 260) {

    PWM_SPEED = 0;

    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);

    Serial.println("Stopping motors...");
    delay(500);

    // Direction change
    if (dir == 0) {
      dir = 1; 
    } else {
      dir = 0; 
    }

    Serial.println("Change of direction...");
    delay(500);

    PWM_SPEED = 140;
  }

  delay(500);
}
