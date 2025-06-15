#include <MyLibrary.h>  

MyLibrary myLib;  

void setup() {
  Serial.begin(115200);  
  
  int result = myLib.addNumbers(350, 80);  

  Serial.print("The result of 350 + 80 is: ");
  Serial.println(result);
}

void loop() {
  
}

