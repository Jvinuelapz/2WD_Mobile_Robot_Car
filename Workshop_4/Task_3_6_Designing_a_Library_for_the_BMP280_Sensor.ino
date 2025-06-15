#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> // Header del sensor BMP280

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // Hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK); // Software SPI

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100); 

  Serial.println(F("BMP280 test"));

  unsigned status;
  status = bmp.begin(0x76);

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    Serial.println("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    Serial.println("ID of 0x56-0x58 represents a BMP 280,");
    Serial.println("ID of 0x60 represents a BME 280.");
    Serial.println("ID of 0x61 represents a BME 680.");
    while (1) delay(10);
  }


  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,     
    Adafruit_BMP280::SAMPLING_X2,     
    Adafruit_BMP280::SAMPLING_X16,    
    Adafruit_BMP280::FILTER_X16,      
    Adafruit_BMP280::STANDBY_MS_500  
  );
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1015)); // Ajustado a la presión local
  Serial.println(" m");

  Serial.println();
  delay(2000);
}


