#include <Arduino.h>
// =====================================================
// SMART WASTE PRO - PHASE 1 (ESP32 CODE)
// 3 Ultrasonic Sensors + LEDs + Buzzer
// PlatformIO (Arduino Framework)
// =====================================================


// ===================== PIN DEFINITIONS =====================

// Ultrasonic Sensors
#define TRIG1 5    // Plastic
#define ECHO1 18

#define TRIG2 19   // Food
#define ECHO2 23

#define TRIG3 26   // Metal
#define ECHO3 27

// LEDs (one per bin)
#define LED1 12
#define LED2 13
#define LED3 25

// Buzzer
#define BUZZER 14


// ===================== THRESHOLDS =====================
// Adjust based on your bin height

#define FULL_LEVEL 10
#define LEVEL_75   15
#define LEVEL_50   20


// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);

  // Sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  // LED pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // Buzzer
 ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER, 0);


  Serial.println("=== SMART WASTE PRO SYSTEM STARTED ===");
}


// ===================== FUNCTION: GET DISTANCE =====================

float getDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  float distance = duration * 0.034 / 2;

  return distance;
}


// ===================== FUNCTION: CHECK LEVEL =====================

String getLevel(float distance) {

  if (distance < FULL_LEVEL) {
    return "FULL";
  } 
  else if (distance < LEVEL_75) {
    return "75%";
  } 
  else if (distance < LEVEL_50) {
    return "50%";
  } 
  else {
    return "EMPTY";
  }
}


// ===================== FUNCTION: LED CONTROL =====================

void controlLED(int ledPin, String level) {

  if (level == "FULL") {
    digitalWrite(ledPin, HIGH);
  } 
  else {
    digitalWrite(ledPin, LOW);
  }
}


// ===================== MAIN LOOP =====================

void loop() {

  // ----------- READ DISTANCES -----------
  float d1 = getDistance(TRIG1, ECHO1); // Plastic
  float d2 = getDistance(TRIG2, ECHO2); // Food
  float d3 = getDistance(TRIG3, ECHO3); // Metal


  // ----------- GET LEVEL STATUS -----------
  String level1 = getLevel(d1);
  String level2 = getLevel(d2);
  String level3 = getLevel(d3);


  // ----------- PRINT OUTPUT -----------
  Serial.println("\n===== BIN STATUS =====");

  Serial.print("Plastic: ");
  Serial.print(d1);
  Serial.print(" cm -> ");
  Serial.println(level1);

  Serial.print("Food: ");
  Serial.print(d2);
  Serial.print(" cm -> ");
  Serial.println(level2);

  Serial.print("Metal: ");
  Serial.print(d3);
  Serial.print(" cm -> ");
  Serial.println(level3);


  // ----------- LED CONTROL -----------
  controlLED(LED1, level1);
  controlLED(LED2, level2);
  controlLED(LED3, level3);


  // ----------- BUZZER LOGIC -----------

if (level1 == "FULL" || level2 == "FULL" || level3 == "FULL") {
  ledcWriteTone(0, 2000);  // ON sound
} 
else {
  ledcWriteTone(0, 0);     // OFF
}


  // ----------- DELAY -----------
  delay(2000);
}