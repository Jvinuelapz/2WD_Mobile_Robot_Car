#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32_Servo";
const char* password = "12345678";

WebServer server(80);

const int servoPin = 2;
int currentAngle = 0;

void handleRoot() {
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <title>ESP32 Servo Control</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
            button { padding: 15px 30px; font-size: 20px; cursor: pointer; }
        </style>
    </head>
    <body>
        <h1>SERVO CONTROL</h1>
        <button onclick="move()">Move servo</button>
        <button onclick="fullRotation()">Full Rotation</button>

        <script>
            function move() {
                fetch("/move");
            }
            function fullRotation() {
                fetch("/fullrotation");
            }
        </script>
    </body>
    </html>
    )rawliteral";

    server.send(200, "text/html", html);
}

void handleMove() {
    moveServo(90);
    delay(500);
    moveServo(180);
    server.send(200, "text/plain", "");
}

void handleFullRotation() {
    for (int angle = 0; angle <= 180; angle += 9) {
        moveServo(angle);
        delay(20); // 100ms entre cada movimiento
    }
    server.send(200, "text/plain", "Rotación completa realizada.");
}

void setup() {
    Serial.begin(115200);
    pinMode(servoPin, OUTPUT);

    WiFi.softAP(ssid, password);
    Serial.println("Hotspot created.");
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/fullrotation", handleFullRotation);
    
    server.begin();
}

void loop() {
    server.handleClient();
}

// Función para mover el servo
void moveServo(int angle) {
    int pulseWidth = map(angle, 0, 180, 500, 2500); // 500-2500 µs
    unsigned long startTime = millis();
    while (millis() - startTime < 500) { 
        digitalWrite(servoPin, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(servoPin, LOW);
        delay(20); //  ~50Hz
    }
    currentAngle = angle;
    Serial.println("Current Angle: " + String(currentAngle) + "°");
}
