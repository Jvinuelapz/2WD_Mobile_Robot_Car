// Define the pins 
const int trigPin = 19;
const int echoPin = 23;

int distance = 0;

// Filter Variables
const int numReadings = 10; 
int readings[numReadings];  
int readIndex = 0;          
int total = 0;              
int average = 0;           

void setup() {
  // Set the trig pin to output mode and the echo pin to input mode
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Start the serial communication
  Serial.begin(115200);

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
// Send a pulse to the trig pin to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  distance = duration / 58;

  total = total - readings[readIndex];   // Delete oldest reading
  readings[readIndex] = distance;         // Save new reading
  total = total + readings[readIndex];    // Add new reading
  readIndex = (readIndex + 1) % numReadings; 

  average = total / numReadings; // Average

  // Print the distance to the serial monitor
  Serial.print("Raw(cm):");
  Serial.print(distance); 
  Serial.print(",");

  Serial.print("Filtered(cm):");
  Serial.println(average);

  // Wait for 500 milliseconds before taking another measurement
  delay(100);
  }