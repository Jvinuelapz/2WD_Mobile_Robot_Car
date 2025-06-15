#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Credenciales del AP
const char* ssid = "ESP32-Car";
const char* password = "12345678";

long timeToUpdate;

WiFiUDP udp;
WebServer server(80);
const unsigned int localPort = 4210;

// Pines motores
#define MOTOR_A1 4    
#define MOTOR_A2 5    
#define MOTOR_B1 25    
#define MOTOR_B2 26   

// Pines sensor HC-SR04
const int trigPin = 19;
const int echoPin = 23;

// PWM velocidades
#define PWM_SPEED       200
#define PWM_SPEED_GIRO  140
#define PWM_AUTO        140

// Estado del coche
bool movingForward = false;
bool movingBackward = false;
bool turningLeft = false;
bool turningRight = false;
bool modoAuto = false;
bool modoIncli = false;

unsigned long lastCommandTime = 0;
const unsigned long timeoutInterval = 500;

float distanciaActual = 0;
unsigned long lastDistanceCheck = 0;

float angleX = 0.0; 

const int filterSize = 10;
float distanceBuffer[filterSize];
int bufferIndex = 0;
bool bufferFull = false;

// Web (la misma del canvas)
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Car Control</title>
    <style>
        body { text-align: center; font-family: Arial, sans-serif; background: #f4f4f4; }
        .controls { display: grid; grid-template-columns: repeat(3, 80px); gap: 10px; justify-content: center; margin-top: 20px; }
        button { width: 80px; height: 80px; font-size: 24px; border-radius: 8px; border: none; box-shadow: 2px 2px 5px rgba(0,0,0,0.2); cursor: pointer; }
        #autoButtons { margin-top: 30px; }
    </style>
</head>
<body>
    <h2>ESP32 Car Control</h2>
    <p><strong>Distance:</strong> <span id="dist">-</span> cm</p>

    <div class="controls">
        <div></div><button id="btnW" onmousedown="sendCommand('W', true)" onmouseup="sendCommand('W', false)">W</button><div></div>
        <button id="btnA" onmousedown="sendCommand('A', true)" onmouseup="sendCommand('A', false)">A</button>
        <button id="btnS" onmousedown="sendCommand('S', true)" onmouseup="sendCommand('S', false)">S</button>
        <button id="btnD" onmousedown="sendCommand('D', true)" onmouseup="sendCommand('D', false)">D</button>
    </div>

    <div id="autoButtons">
        <button onclick="toggleAuto(true)" style="background-color: #4CAF50; color: white; width:160px; height:60px; font-size:20px;">Start Auto Mode</button>
        <button onclick="toggleAuto(false)" style="background-color: #f44336; color: white; width:160px; height:60px; font-size:20px;">Stop Auto Mode</button>
    </div>

    <div id="InclinationButtons">
        <button onclick="toggleIncli(true)" style="background-color: #4CAF50; color: white; width:160px; height:60px; font-size:20px;">Start Inclination Mode</button>
        <button onclick="toggleIncli(false)" style="background-color: #f44336; color: white; width:160px; height:60px; font-size:20px;">Stop Inclination Mode</button>
    </div>

    <script>
        let autoMode = false;
        let IncliMode = false

        function sendCommand(cmd, state) {
            if (autoMode) return;
            fetch(`/cmd?key=${cmd}&state=${state}`);
        }

        function toggleAuto(start) {
            autoMode = start;
            fetch(`/auto?start=${start ? 1 : 0}`);
        }

        function toggleIncli(start) {
            IncliMode = start;
            fetch(`/incli?start=${start ? 1 : 0}`);
        }

        document.addEventListener("keydown", (event) => {
            const key = event.key.toUpperCase();
            if (["W", "A", "S", "D"].includes(key) && !autoMode) {
                sendCommand(key, true);
            }
        });

        document.addEventListener("keyup", (event) => {
            const key = event.key.toUpperCase();
            if (["W", "A", "S", "D"].includes(key) && !autoMode) {
                sendCommand(key, false);
            }
        });

        setInterval(() => {
            fetch("/dist").then(res => res.text()).then(d => {
                document.getElementById("dist").innerText = d;
            });
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Starting!");

    Serial.println("Initializing IMU...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    WiFi.softAP(ssid, password);
    Serial.println("AP iniciado: " + WiFi.softAPIP().toString());

    udp.begin(localPort);

    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    stopMotors();

    server.on("/", []() {
        server.send_P(200, "text/html", webpage);
    });

    server.on("/cmd", []() {
        if (server.hasArg("key") && server.hasArg("state")) {
            modoAuto = false; // desactiva modo auto al tocar controles
            char cmd = server.arg("key")[0];
            bool state = server.arg("state") == "true";
            updateMovement(cmd, state);
            lastCommandTime = millis();
        }
        server.send(204, "text/plain", "");

        Serial.print("Comando recibido: ");
        Serial.print(server.arg("key"));
        Serial.print(" -> ");
        Serial.println(server.arg("state"));

    });

    server.on("/auto", []() {
        if (server.hasArg("start")) {
            modoAuto = (server.arg("start") == "1");
            Serial.println(modoAuto ? "🚗 AUTO mode ON" : "🛑 AUTO mode OFF");
            stopMotors();
            server.send(200, "text/plain", "OK");
        }
    });

    server.on("/incli", []() {
        if (server.hasArg("start")) {
            // Aquí activas o desactivas tu modo inclinación
            modoIncli = (server.arg("start") == "1");
            Serial.println(modoIncli ? "🎢 INCLINATION mode ON" : "🛑 INCLINATION mode OFF");
   
            if (!modoIncli) {
                stopMotors();
            }

            server.send(200, "text/plain", "OK");
        }
    });

    server.on("/dist", []() {
        server.send(200, "text/plain", String(distanciaActual, 1));
    });

    server.begin();
}

// --- Loop ---
void loop() {
    server.handleClient();

    if (millis() - lastDistanceCheck > 400) {
        distanciaActual = measureDistance();
        lastDistanceCheck = millis();
    }

    // Modo automático
    if (modoAuto) {
        if (distanciaActual >= 10.0) {
            analogWrite(MOTOR_A1, PWM_AUTO);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, PWM_AUTO);
            analogWrite(MOTOR_B2, 0);
        } else {
            stopMotors();
        }
    }

    accelerometerRead();

    if (modoIncli) {
        if (angleX < -10.0) {
            analogWrite(MOTOR_A1, PWM_SPEED+50);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, PWM_SPEED+50);
            analogWrite(MOTOR_B2, 0);
        } 
        else if (angleX > 10.0) {
            analogWrite(MOTOR_A1, PWM_SPEED-50);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, PWM_SPEED-50);
            analogWrite(MOTOR_B2, 0);
        } 
        else {
            analogWrite(MOTOR_A1, PWM_SPEED);
            analogWrite(MOTOR_A2, 0);
            analogWrite(MOTOR_B1, PWM_SPEED);
            analogWrite(MOTOR_B2, 0);
        }
    }  
}

// --- Funciones ---
void accelerometerRead(){
    if(timeToUpdate < millis()){
        timeToUpdate = millis() + 100;
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
        float angleY = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;

        Serial.print("Inclination angle X: ");
        Serial.println(angleX, 1);
    }

}

void updateMovement(char command, bool state) {
    if (state) {
        movingForward = false;
        movingBackward = false;
        turningLeft = false;
        turningRight = false;
    }

    switch (command) {
        case 'W': movingForward = state; break;
        case 'S': movingBackward = state; break;
        case 'A': turningLeft = state; break;
        case 'D': turningRight = state; break;
        case 'X': stopMotors(); return;
    }

    applyMotorMovement();
}

void applyMotorMovement() {
    if (movingForward && !movingBackward && !turningLeft && !turningRight) {
        analogWrite(MOTOR_A1, PWM_SPEED);
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, PWM_SPEED);
        analogWrite(MOTOR_B2, 0);
    } else if (movingBackward) {
        analogWrite(MOTOR_A1, 0);
        analogWrite(MOTOR_A2, PWM_SPEED);
        analogWrite(MOTOR_B1, 0);
        analogWrite(MOTOR_B2, PWM_SPEED);
    } else if (turningLeft && movingForward) {
        analogWrite(MOTOR_A1, PWM_SPEED_GIRO);
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, PWM_SPEED);
        analogWrite(MOTOR_B2, 0);
    } else if (turningRight && movingForward) {
        analogWrite(MOTOR_A1, PWM_SPEED);
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, PWM_SPEED_GIRO);
        analogWrite(MOTOR_B2, 0);
    } else if (turningLeft) {
        analogWrite(MOTOR_A1, 0);
        analogWrite(MOTOR_A2, PWM_SPEED_GIRO);
        analogWrite(MOTOR_B1, PWM_SPEED_GIRO);
        analogWrite(MOTOR_B2, 0);
    } else if (turningRight) {
        analogWrite(MOTOR_A1, PWM_SPEED_GIRO);
        analogWrite(MOTOR_A2, 0);
        analogWrite(MOTOR_B1, 0);
        analogWrite(MOTOR_B2, PWM_SPEED_GIRO);
    } else {
        stopMotors();
    }
}

void stopMotors() {
    analogWrite(MOTOR_A1, 0);
    analogWrite(MOTOR_A2, 0);
    analogWrite(MOTOR_B1, 0);
    analogWrite(MOTOR_B2, 0);
}

float filteredDistance = 0; // global

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  float raw = duration * 0.034 / 2;

  // Limitar valores inválidos
  if (raw < 2 || raw > 400) raw = 999;

  // Aplicar filtro EMA
  const float alpha = 0.4; // entre 0.3 y 0.5 va bien
  filteredDistance = (alpha * raw) + (1 - alpha) * filteredDistance;

  return raw; 
}

