#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "ultrasonic.h"
#include "encoders.h"
#include "Motor.h"
#include "Odometry.h"
#include "Navigation.h"

SteeringConfig navCfg;
SteeringState navState; 

const char* ssid = "Irving’s iPhone";  //Phone credentials
const char* pass = "romero22";
const char* hostIP = "172.20.10.4";  // <-- Phone IP

// const char* ssid = "Verizon_6FB9JL";  //PC credentials
// const char* pass = "dub7not7protean";
// const char* hostIP = "192.168.1.156";  // <-- your PC IP
const uint16_t hostPort = 5005;

// --- Objects ---
WiFiUDP Udp;
Ultrasonic frontUS(US1_TRIG, US1_ECHO);
Ultrasonic leftUS(US2_TRIG, US2_ECHO);
Ultrasonic rightUS(US3_TRIG, US3_ECHO);
Motor leftMotor(M1_PWM, M1_INA, M1_INB, M1_EN, CH_M1);
Motor rightMotor(M2_PWM, M2_INA, M2_INB, M2_EN, CH_M2);
Encoder leftEnc(L_ENCA, L_ENCB);
Encoder rightEnc(R_ENCA, R_ENCB);
Odometry odo(&leftEnc, &rightEnc);

float Kside   = 20.0;
float Kfront  = 5.0;
int   baseSpeed = 200;
int   maxSpeed  = 500;
float Kopen = 0.5;   // Strength of "turn toward open space"



float sanitizeUS(float d) {
    if (d <= 0 || d > 300) return 300;  
    return d;
}

void smoothNavigate(float frontDist, float leftDist, float rightDist) {

    // Clean noise
    frontDist = sanitizeUS(frontDist);
    leftDist  = sanitizeUS(leftDist);
    rightDist = sanitizeUS(rightDist);

    // Cap around robot
    float L = min(leftDist, 20.0f);
    float R = min(rightDist, 20.0f);

    // Speed control
    float speedFactor = 1.0;
    if (frontDist < 30) speedFactor = frontDist / 30.0;

    int forwardSpeed = baseSpeed * speedFactor;
    if (forwardSpeed < 200) forwardSpeed = 200;

    // --- BASE steering from walls (avoidance) ---
    float diff = L - R;
    if (fabs(diff) < 5) diff = 0;

    float steer = diff * (Kside * 1.2);

    // --- NEW: Open-space steering ---
    // If right side is more open → steer right
    // If left side is more open → steer left
    float openSteer = (rightDist - leftDist) * Kopen;
    steer += openSteer;

    // --- Close front boost ---
    if (frontDist < 100)
        steer += (100 - frontDist) * (Kfront * 0.8);

    // Motor mixing
    int leftSpeed  = forwardSpeed + steer;
    int rightSpeed = forwardSpeed - steer;

    leftSpeed  = constrain(leftSpeed,  -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    if (leftSpeed >= 0) leftMotor.forward(leftSpeed);
    else                leftMotor.reverse(-leftSpeed);

    if (rightSpeed >= 0) rightMotor.forward(rightSpeed);
    else                 rightMotor.reverse(-rightSpeed);
}


// --- Telemetry task (runs on Core 0) ---
void telemetryTask(void* pvParameters) {
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      char buf[128];
      snprintf(buf, sizeof(buf), "%f,%f,%f,%f,%f,%f\n",
               odo.getX(), odo.getY(), odo.getTheta(),
               sanitizeUS(frontUS.getDistanceCM()), sanitizeUS(leftUS.getDistanceCM()), sanitizeUS(rightUS.getDistanceCM()));
      Udp.beginPacket(hostIP, hostPort);
      Udp.write((uint8_t*)buf, strlen(buf));
      Udp.endPacket();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  // send every 0.5 s
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // --- Wi-Fi ---
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting...");
    delay(200);   // <-- REQUIRED, otherwise WDT reset
    yield();      // <-- helps Core 1 reset the watchdog
  }

  delay(300);     // <-- VERY important for S3 WiFi stability

  Udp.begin(hostPort);

  // --- Hardware ---
  leftEnc.begin();
  rightEnc.begin();
  frontUS.begin();
  leftUS.begin();
  rightUS.begin();
  leftMotor.begin();
  rightMotor.begin();

  // --- Telemetry task ---
  xTaskCreatePinnedToCore(telemetryTask, "Telemetry", 4096, NULL, 1, NULL, 0);

  Serial.println("Setup complete!");
}


// --- Main loop (Core 1) ---
void loop() {
    odo.update();


    
    float frontDist = sanitizeUS(frontUS.getDistanceCM());
    float leftDist  = sanitizeUS(leftUS.getDistanceCM());
    float rightDist = sanitizeUS(rightUS.getDistanceCM());

    smoothNavigate(frontDist, leftDist, rightDist);
  // --- Debug print ---
  Serial.printf("X: %.2f Y: %.2f θ: %.2f\n", odo.getX(), odo.getY(), odo.getTheta());
  delay(20);  // fast control loop (~50 Hz)
}