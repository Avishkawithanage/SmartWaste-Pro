#include "HX711.h"


#define LEVEL_SENSOR 34   // ADC pin

// HX711 load cell pins
#define HX711_DOUT 32
#define HX711_SCK  33

// Status LED
#define LED_PIN 2



HX711 scale;


void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // HX711 initialization (5kg load cell)
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(420);   // Logical calibration for simulation
  scale.tare();           // Zero reference

  Serial.println("SmartWaste Pro - IoT Simulator Started");
}



void loop() {

 
  int levelRaw = analogRead(LEVEL_SENSOR);
  int levelPercent = map(levelRaw, 0, 4095, 0, 100);

  
  float weightKg = scale.get_units(10);   // Averaged reading

  // Noise handling (no negative weight allowed)
  if (weightKg < 0) {
    weightKg = 0;
  }

  
  String status;

  // Rule 1: Completely empty bin
  if (levelPercent == 0) {
    status = "EMPTY";
    digitalWrite(LED_PIN, LOW);
  }
  // Rule 2: Low level and low weight
  else if (levelPercent < 40 && weightKg <= 1.0) {
    status = "EMPTY";
    digitalWrite(LED_PIN, LOW);
  }
  // Rule 3: Bin in use but not ready for collection
  else if (levelPercent < 90 && weightKg < 3.5) {
    status = "PARTIAL";
    digitalWrite(LED_PIN, LOW);
  }
  // Rule 4: Bin ready for collection
  else {
    status = "FULL";
    digitalWrite(LED_PIN, HIGH);
  }

 
  Serial.println("----------------------------------");
  Serial.println("Bin ID        : SWP-001");
  Serial.println("Compartment   : Plastic");
  Serial.print("Fill Level    : ");
  Serial.print(levelPercent);
  Serial.println("%");
  Serial.print("Weight        : ");
  Serial.print(weightKg, 2);
  Serial.println(" kg");
  Serial.print("Status        : ");
  Serial.println(status);

  delay(2000);
}
