// RIGHT WHEEL
int B1A = 4; // GPIO Digital Pin 4
int B2B = 5; // GPIO Digital Pin 5

// LEFT WHEEL
int A1A = 25; // GPIO Digital Pin 32
int A2B = 26; // GPIO Digital Pin 33

void setup() {
  pinMode(B1A, OUTPUT);
  pinMode(B2B, OUTPUT);
  pinMode(A1A, OUTPUT); 
  pinMode(A2B, OUTPUT); 
}

void loop() {
  digitalWrite(B1A, LOW); 
  digitalWrite(B2B, HIGH); 
  digitalWrite(A1A, LOW);  
  digitalWrite(A2B, HIGH); 
}

// LOW HIGH HIGH LOW --> CLOCKWISE ROTATION
// LOW HIGH LOW HIGH --> FORWARD
// HIGH LOW HIGH LOW --> BACKWARD
// HIGH LOW LOW HIGH --> ANTI-CLOCKWISE ROTATION

