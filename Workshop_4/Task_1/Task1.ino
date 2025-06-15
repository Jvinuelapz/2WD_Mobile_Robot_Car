#include "MyLibrary.h"

MyLibrary myLib;

void setup() {
  Serial.begin(115200);
  delay(1000); 
}

void loop() {
  int analogValue = analogRead(36);  
  int result = myLib.addNumbers(analogValue, 100);

  Serial.println(result);  

  delay(200); 
}
