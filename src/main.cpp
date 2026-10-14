#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HX711.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

//  WIFI 

#define WIFI_SSID     "Avii"
#define WIFI_PASSWORD "Avishka04"

//  FIREBASE 

#define DATABASE_SECRET "bzJ1X7QqxRNoMdQY8rQmiADwZrwe40k4r6EBFYPq"
#define API_KEY         "AIzaSyAE2jjgeKgqIiHy4NhzVbcHSr-qoBrFUc4"
#define DATABASE_URL    "smartwastepro-1085e-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

//  PIN DEFINITIONS 

#define TRIG1 5
#define ECHO1 18
#define TRIG2 19
#define ECHO2 23
#define TRIG3 26
#define ECHO3 27

#define LED1 12
#define LED2 13
#define LED3 25

#define BUZZER 14

#define DT1  36
#define SCK1 33
#define DT2  4
#define SCK2 16
#define DT3  17
#define SCK3 21

// GPIO 34 & 35 are INPUT-ONLY  i added external 10kΩ pullup to 3.3V

#define BTN1 34
#define BTN2 35
#define BTN3 32

//  FILL  level for bin hight 

#define FULL_LEVEL  8
#define LEVEL_75   14
#define LEVEL_50   22

// TIMING 

#define SENSOR_INTERVAL   500
#define FIREBASE_INTERVAL 3000
#define BUZZER_BEEP_ON    200
#define BUZZER_BEEP_OFF   800
#define DEBOUNCE_MS        50

//  WEIGHT FILTER 

#define WEIGHT_SAMPLES   10       // moving-average window (10 samples = about 1 s at 10 samples/s)
#define WEIGHT_DEADBAND   5.0f
#define OVERWEIGHT_LIMIT 550.0f   // g

//  LOAD CELL CALIBRATION


#define CAL_PLASTIC 430.0002f   // 250 g calibration factor for Plastic load cell
#define CAL_FOOD    438.0265f   //  for Food load cell 
#define CAL_METAL   423.4634f   //  for Metal load cell 

// LOAD CELL FILTER SETTINGS 

#define ZERO_THRESHOLD_G        2.0f   // |weight| below this is treated as empty (0 g)
#define STABLE_BAND_G           0.8f  
#define NEGATIVE_FAULT_G       10.0f   // average below -10 g (with NO tare active) is NOT just noise -> warn once on Serial
#define OUTLIER_JUMP_G         30.0f   
#define OUTLIER_CONFIRM            3   
#define MIN_SAMPLES_FOR_OUTLIER    3   

#define TARE_SAMPLES              10   
#define TARE_ATTEMPTS              3   // startup/button tare retries if readings are unstable
#define TARE_SAMPLE_TIMEOUT_MS  1000   // max wait for ONE HX711 sample (never waits forever)
#define TARE_MAX_PP_G            3.0f  
#define TARE_RETRY_MS           5000  

// Two-level tare (see "LOAD CELL / WEIGHT" section)

#define TARE_REZERO_BAND_G      10.0f 
#define TARE_RECOVER_BAND_G      5.0f
#define TARE_RECOVER_HOLD_MS    1000   

#define HX711_RAW_LIMIT      8300000L 
#define HX711_STALL_MS          3000   // no new HX711 data for this long -> warning
#define LOADCELL_WARN_MS        5000   // minimum time between repeated warnings per load cell

// LONG PRESS DURATION — 2000ms = 2 seconds  ( for buttons to trare rests to 0)

#define LONG_PRESS_MS 2000

// OVERWEIGHT ALERT SETTINGS 
#define OVERWEIGHT_ALERT_REPEAT_MS 5000 
#define OVERWEIGHT_SIREN_ON_MS      300   // siren tone length
#define OVERWEIGHT_SIREN_OFF_MS     100   // short gap between siren tones
#define OVERWEIGHT_SIREN_FREQ_HI   2800   
#define OVERWEIGHT_SIREN_FREQ_LO   2000   
#define OVERWEIGHT_LED_BLINK_MS     250   

//  BUZZER TONES 
#define FULL_BEEP_FREQ     2000   //  existing bin-full beep
#define TARE_BEEP_MS        150   // single tare confirmation beep length
#define TARE_BEEP_FREQ     3000   
#define WIFI_BEEP_MS        150   // each of the 3 Wi-Fi beeps (and each gap)
#define WIFI_BEEP_FREQ     3000   


//  load cell 3  separate controllers

HX711 scale1, scale2, scale3;

// One independent filter per load cell (samples are never mixed between cells)
struct WeightFilter {
  const char   *name;                 // used in Serial messages
  float         buf[WEIGHT_SAMPLES];  // recent valid weight samples in grams
  uint8_t       count;                // how many samples are in buf
  uint8_t       head;                 // next write position
  float         average;              // moving average of the SHOWN (net) weight in grams (internal, not rounded)
  float         finalWeight;          // stable, rounded weight used by the rest of the firmware
  uint8_t       outlierStreak;        // consecutive suspicious samples
  bool          tared;                // true once a valid empty baseline exists
  unsigned long lastSampleMs;         // time of last valid HX711 sample
  unsigned long lastWarnMs;           // rate-limits Serial warnings
  unsigned long lastTareTryMs;        // rate-limits tare retries

  // temporary (user) tare state

  float         tareGrams;            // load that the user "zeroed out" with the button (0 when no temporary tare)
  bool          tareActive;           // true while a temporary tare is in effect
  bool          recovering;           // true while total load is near 0 and we wait TARE_RECOVER_HOLD_MS
  unsigned long recoverSinceMs;       // when the recovering period began
  bool          negWarned;            // large-negative warning already printed for this episode
};

WeightFilter plasticFilter = { "Plastic" };
WeightFilter foodFilter    = { "Food" };
WeightFilter metalFilter   = { "Metal" };

unsigned long prevSensorMillis   = 0;
unsigned long prevFirebaseMillis = 0;

String lastL1 = "INIT", lastL2 = "INIT", lastL3 = "INIT";
float  lastW1 = -999.0f, lastW2 = -999.0f, lastW3 = -999.0f;

// Last overweight flags sent to Firebase (used only to send the overweight field when it changes)
bool lastOw1 = false, lastOw2 = false, lastOw3 = false;

// Latest fill levels (updated every SENSOR_INTERVAL). LED / buzzer code runs on every loop pass
// and uses these cached values so blinking and beeping stay smooth.
String curL1 = "INIT", curL2 = "INIT", curL3 = "INIT";

// Overweight state per compartment 
bool          ow1 = false, ow2 = false, ow3 = false;
unsigned long owLastPrint1 = 0, owLastPrint2 = 0, owLastPrint3 = 0;

// Overweight LED blink phase (shared timing, each compartment still follows its own ow flag)
bool overLedPhase = false;

// Requested buzzer tones (0 = silent). Every buzzer feature only writes its OWN variable;
// applyBuzzer() is the ONLY place that touches the buzzer hardware.
uint16_t fullTone  = 0;   // bin-full beep
uint16_t alarmTone = 0;   // overweight siren
uint16_t tareTone  = 0;   // tare confirmation beep
uint16_t wifiTone  = 0;   // Wi-Fi connected beeps

bool     tareBeepActive = false;   // tare beep owns the buzzer while true
bool     wifiBeepActive = false;   // Wi-Fi beeps own the buzzer while true
unsigned long tareBeepStartMs = 0;


// Button state with debouncing + long-press detection
struct ButtonState {
  uint8_t       pin;
  bool          rawLast;      // last raw reading
  bool          stable;       
  unsigned long changeMs;     // when the raw reading last changed
  unsigned long pressMs;      // when the debounced press started
  bool          handled;      // long-press action already done for this press
};

ButtonState btn1 = { BTN1, HIGH, HIGH, 0, 0, false };
ButtonState btn2 = { BTN2, HIGH, HIGH, 0, 0, false };
ButtonState btn3 = { BTN3, HIGH, HIGH, 0, 0, false };


//  Reset-Based Weight Removal Alert - START
#define REMOVAL_ALERT_MIN_G          8.0f   // removed weight (g) that counts as a real removal (ignores noise)
#define REMOVAL_ALERT_DURATION_MS    5000   
#define REMOVAL_LED_BLINK_MS          200  
#define REMOVAL_BEEP_FREQ            2500   
#define REMOVAL_BEEP_MS               120  
#define REMOVAL_BEEP_CYCLE_MS        1000   

// One independent tracker per compartment
struct RemovalTracker {
  float         hidden;           // weight (g) hidden by Reset that is still on the load cell
  bool          lastTareActive;   // previous tareActive (to detect a new Reset)
  float         lastTareGrams;    // previous tareGrams (to detect a new Reset)
  bool          alertActive;      // LED blink + beep-beep running
  unsigned long alertStartMs;     // when the alert started
};

RemovalTracker removalPlastic = {};
RemovalTracker removalFood    = {};
RemovalTracker removalMetal   = {};

uint16_t removalTone = 0;         
//  Reset-Based Weight Removal Alert - END

//  FORWARD DECLARATIONS

float  getDistance(int trigPin, int echoPin);
String getLevel(float d);
bool   tareLoadCell(HX711 &scale, WeightFilter &f, uint8_t attempts);
bool   userTareLoadCell(HX711 &scale, WeightFilter &f);
void   updateLoadCell(HX711 &scale, WeightFilter &f);
void   processButton(ButtonState &b, HX711 &scale, WeightFilter &f, const char *fbPath, float &lastW);
void   handleButtons();
void   updateOverweightState();
void   updateOverweightLEDs();
void   controlLED(int pin, const String &level, bool overweight);
void   updateOverweightAlarm();
void   updateBinFullBeep();
void   playTareConfirmationBeep();
void   updateTareBeep();
void   updateWiFiConnectionBeep();
void   applyBuzzer();
void   sendToFirebase(const String &l1, const String &l2, const String &l3,
                     float w1, float w2, float w3);


//  ULTRASONIC — single  reading for fill level 

float getDistance(int trigPin, int echoPin) {

  // Send a clean 10µs trigger pulse

  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for echo, 30ms timeout

  long duration = pulseIn(echoPin, HIGH, 30000UL);

  // If no echo received, sensor is blocked or disconnected

  if (duration == 0) return 999.0f;

  // Convert to cm: speed of sound = 0.0343 cm/µs, divide by 2 (there and back)

  float distance = duration * 0.0343f / 2.0f;
  return distance;
}


//  FILL LEVEL STRING

String getLevel(float d) {
  if (d <= 0.0f || d > 400.0f) return "ERROR";
  if (d < FULL_LEVEL)           return "FULL";
  if (d < LEVEL_75)             return "75%";
  if (d < LEVEL_50)             return "50%";
  return "EMPTY";
}

// Print a load-cell warning at most once per LOADCELL_WARN_MS for each load cell
bool warnAllowed(WeightFilter &f) {
  unsigned long now = millis();
  if (f.lastWarnMs != 0 && (now - f.lastWarnMs) < LOADCELL_WARN_MS) return false;
  f.lastWarnMs = now;
  return true;
}

// Clear the moving-average history of ONE load cell (final weight becomes 0 g)
void resetFilter(WeightFilter &f) {
  f.count         = 0;
  f.head          = 0;
  f.average       = 0.0f;
  f.finalWeight   = 0.0f;
  f.outlierStreak = 0;
  f.lastSampleMs  = millis();
}

// Turn the moving average into the final, stable weight.
//  - Small values near zero are treated as empty (0 g)
//    (STABLE_BAND_G), so 249.4 / 250.3 / 250.6 keep showing 250 g
float makeFinalWeight(float average, float previousFinal) {
  if (fabsf(average) < ZERO_THRESHOLD_G) return 0.0f;   // empty
  if (average < 0.0f)                    return 0.0f;   // negative noise / drift
  if (fabsf(average - previousFinal) < STABLE_BAND_G) return previousFinal;  // hold stable value
  return roundf(average);                               // normal whole-gram rounding
}

// Add one weight sample (grams) to the moving average of ONE load cell
void addWeightSample(WeightFilter &f, float grams) {

  if (f.count >= MIN_SAMPLES_FOR_OUTLIER && fabsf(grams - f.average) > OUTLIER_JUMP_G) {
    f.outlierStreak++;
    if (f.outlierStreak < OUTLIER_CONFIRM) return;
    f.count = 0;
    f.head  = 0;
  }
  f.outlierStreak = 0;

  // Moving average over the last WEIGHT_SAMPLES valid samples
  f.buf[f.head] = grams;
  f.head = (f.head + 1) % WEIGHT_SAMPLES;
  if (f.count < WEIGHT_SAMPLES) f.count++;

  float sum = 0.0f;
  for (uint8_t i = 0; i < f.count; i++) sum += f.buf[i];
  f.average = sum / static_cast<float>(f.count);

  f.finalWeight = makeFinalWeight(f.average, f.finalWeight);
}

bool measureRawAverage(HX711 &scale, uint8_t attempts, long &offsetOut, long &spreadOut, long &maxSpreadOut) {
  long bestOffset = 0;
  long bestSpread = -1;                                    // -1 = no complete attempt yet
  long maxSpread  = static_cast<long>(TARE_MAX_PP_G * fabsf(scale.get_scale()));

  for (uint8_t a = 0; a < attempts; a++) {
    long sum = 0, minRaw = 0, maxRaw = 0;
    bool ok  = true;

    for (uint8_t i = 0; i < TARE_SAMPLES; i++) {
      unsigned long t0 = millis();
      while (!scale.is_ready()) {
        if (millis() - t0 >= TARE_SAMPLE_TIMEOUT_MS) { ok = false; break; }
        delay(1);
      }
      if (!ok) break;

      long raw = scale.read();                             // HX711 is ready, so this does not block
      if (raw >= HX711_RAW_LIMIT || raw <= -HX711_RAW_LIMIT) { ok = false; break; }
      if (i == 0 || raw < minRaw) minRaw = raw;
      if (i == 0 || raw > maxRaw) maxRaw = raw;
      sum += raw;
    }
    if (!ok) continue;                                     // communication problem: try again

    long spread = maxRaw - minRaw;
    if (bestSpread < 0 || spread < bestSpread) {
      bestSpread = spread;
      bestOffset = sum / TARE_SAMPLES;
    }
    if (spread <= maxSpread) break;                        // stable enough
  }

  if (bestSpread < 0) return false;
  offsetOut    = bestOffset;
  spreadOut    = bestSpread;
  maxSpreadOut = maxSpread;
  return true;
}

// Measure the EMPTY-BIN BASELINE of ONE load cell (used at startup and for the HX711 retry).

bool tareLoadCell(HX711 &scale, WeightFilter &f, uint8_t attempts) {
  long offset, spread, maxSpread;
  if (!measureRawAverage(scale, attempts, offset, spread, maxSpread)) {
    f.tared = false;
    Serial.printf("[SCALE] %s: TARE FAILED - HX711 not answering or at ADC limit. Check wiring.\n", f.name);
    return false;
  }

  scale.set_offset(offset);                                // baseline is measured, never hard-coded
  f.tareGrams  = 0.0f;
  f.tareActive = false;
  f.recovering = false;
  f.negWarned  = false;
  resetFilter(f);
  f.tared = true;
  float spreadG = spread / fabsf(scale.get_scale());
  Serial.printf("[SCALE] %s: tare OK (offset=%ld, noise=%.2f g peak-to-peak)\n", f.name, offset, spreadG);
  if (spread > maxSpread)
    Serial.printf("[SCALE] %s: WARNING readings were unstable during tare - keep the bin still and re-tare.\n", f.name);
  return true;
}

// Button tare for ONE load cell 

bool userTareLoadCell(HX711 &scale, WeightFilter &f) {
  if (!f.tared) return tareLoadCell(scale, f, TARE_ATTEMPTS);   // no baseline yet: this IS the first zero

  long avgRaw, spread, maxSpread;
  if (!measureRawAverage(scale, TARE_ATTEMPTS, avgRaw, spread, maxSpread)) {
    Serial.printf("[SCALE] %s: TARE FAILED - HX711 not answering or at ADC limit. Nothing changed.\n", f.name);
    return false;
  }
  if (spread > maxSpread) {
    Serial.printf("[SCALE] %s: TARE REFUSED - weight is moving. Keep it still and try again. Nothing changed.\n", f.name);
    return false;
  }

  // Total load on the compartment relative to the stored EMPTY baseline
  float total = static_cast<float>(avgRaw - scale.get_offset()) / scale.get_scale();

  if (total <= TARE_REZERO_BAND_G) {
    // Bin is empty (or the empty baseline drifted / is too high): re-measure the empty baseline.
    scale.set_offset(avgRaw);
    f.tareGrams  = 0.0f;
    f.tareActive = false;
    Serial.printf("[SCALE] %s: empty baseline re-zeroed (was %.1f g).\n", f.name, total);
  } else {
    // Something is on the bin: remember it as a TEMPORARY tare. The empty baseline stays untouched.
    f.tareGrams  = total;
    f.tareActive = true;
    Serial.printf("[SCALE] %s: temporary tare of %.1f g set. It clears automatically when the bin is emptied.\n", f.name, total);
  }
  f.recovering = false;
  f.negWarned  = false;
  resetFilter(f);            // shown weight is now 0 g
  return true;
}

// It only reads the HX711 when a new sample is ready (about 10 times per second).

void updateLoadCell(HX711 &scale, WeightFilter &f) {
  unsigned long now = millis();

  // No valid baseline yet (HX711 was not answering at startup): try again later
  if (!f.tared) {
    if (scale.is_ready() && (now - f.lastTareTryMs) >= TARE_RETRY_MS) {
      f.lastTareTryMs = now;
      tareLoadCell(scale, f, 1);
    } else if (warnAllowed(f)) {
      Serial.printf("[SCALE] %s: waiting for HX711 / tare...\n", f.name);
    }
    return;
  }

  if (scale.is_ready()) {
    long raw = scale.read();                               // HX711 is ready, so this does not block
    if (raw >= HX711_RAW_LIMIT || raw <= -HX711_RAW_LIMIT) {
      if (warnAllowed(f))
        Serial.printf("[SCALE] %s: HX711 at ADC limit (raw=%ld) - check load cell wiring.\n", f.name, raw);
      return;                                              // invalid sample is not used
    }
    f.lastSampleMs = now;

    // Total load vs the EMPTY baseline: same formula as the library's get_units(): (raw - OFFSET) / SCALE
    float total = static_cast<float>(raw - scale.get_offset()) / scale.get_scale();
    // Shown weight = total minus the temporary tare (0 when no temporary tare)
    addWeightSample(f, total - f.tareGrams);

    //Temporary tare recovery: the tared load was removed 

    if (f.tareActive) {
      float totalAvg = f.average + f.tareGrams;            // total load, smoothed
      if (totalAvg <= TARE_RECOVER_BAND_G) {
        if (!f.recovering) {
          f.recovering     = true;
          f.recoverSinceMs = now;
        } else if ((now - f.recoverSinceMs) >= TARE_RECOVER_HOLD_MS) {
          f.tareGrams  = 0.0f;
          f.tareActive = false;
          f.recovering = false;
          resetFilter(f);                                  // total load is ~0, so shown weight is 0 g
          Serial.printf("[SCALE] %s: tared load removed - temporary tare cleared, empty baseline restored.\n", f.name);
        }
      } else {
        f.recovering = false;  
      }
    }

    // Negative warning 
    // Without one, a large negative total means the empty baseline is wrong. Warn ONCE per episode.

    if (!f.tareActive && f.average < -NEGATIVE_FAULT_G) {
      if (!f.negWarned) {
        f.negWarned = true;
        Serial.printf("[SCALE] %s: reads %.1f g with no tare active - empty baseline looks wrong (drift/mounting?). Output held at 0 g. Empty the compartment and long-press its tare button to re-zero.\n",
                      f.name, f.average);
      }
    } else if (f.average > -NEGATIVE_FAULT_G / 2.0f) {
      f.negWarned = false;                                 // recovered: allow a future warning
    }
  } else if ((now - f.lastSampleMs) > HX711_STALL_MS && warnAllowed(f)) {
    Serial.printf("[SCALE] %s: no HX711 data for %lu ms - holding last weight. Check wiring/power.\n",
                  f.name, static_cast<unsigned long>(now - f.lastSampleMs));
  }
}


// BUTTONS — long press to tare weight to 0 for each bin

void processButton(ButtonState &b, HX711 &scale, WeightFilter &f, const char *fbPath, float &lastW) {
  unsigned long now = millis();
  bool raw = static_cast<bool>(digitalRead(b.pin));


  if (raw != b.rawLast) {
    b.rawLast  = raw;
    b.changeMs = now;
  }
  if ((now - b.changeMs) >= DEBOUNCE_MS && raw != b.stable) {
    b.stable  = raw;
    b.handled = false;                                     // new press or release: allow one action per press
    if (b.stable == LOW) b.pressMs = now;
  }

  // Long press -> one tare per press
  if (b.stable == LOW && !b.handled && (now - b.pressMs) >= LONG_PRESS_MS) {
    b.handled = true;                                      // set first: no repeated tare while still held
    Serial.printf("[BTN] Long press: Tare %s\n", f.name);
    if (userTareLoadCell(scale, f)) {                      // safe tare (has timeouts), independent per load cell
      lastW = 0.0f;
      playTareConfirmationBeep();                          // exactly one beep per successful tare
      if (WiFi.status() == WL_CONNECTED && Firebase.ready())
        Firebase.RTDB.setFloat(&fbdo, fbPath, 0.0f);
    } else {
      Serial.printf("[BTN] Tare %s was NOT completed.\n", f.name);
    }
  }
}

void handleButtons() {
  processButton(btn1, scale1, plasticFilter, "/bins/bin001/compartments/plastic/weight", lastW1);
  processButton(btn2, scale2, foodFilter,    "/bins/bin001/compartments/food/weight",    lastW2);
  processButton(btn3, scale3, metalFilter,   "/bins/bin001/compartments/metal/weight",   lastW3);
}


// OVERWEIGHT DETECTION (Serial message only)


void checkOverweight(const char *name, float w, bool &active, unsigned long &lastPrintMs) {
  unsigned long now = millis();
  bool over = (w >= OVERWEIGHT_LIMIT);

  if (over && (!active || (now - lastPrintMs) >= OVERWEIGHT_ALERT_REPEAT_MS)) {
    Serial.printf("[ALERT] %s Overweight (Over %.1f g)!\n", name, OVERWEIGHT_LIMIT);
    lastPrintMs = now;
  }
  if (!over && active) {
    Serial.printf("[ALERT] %s overweight cleared.\n", name);
  }
  active = over;
}

void updateOverweightState() {
  checkOverweight("Plastic", plasticFilter.finalWeight, ow1, owLastPrint1);
  checkOverweight("Food",    foodFilter.finalWeight,    ow2, owLastPrint2);
  checkOverweight("Metal",   metalFilter.finalWeight,   ow3, owLastPrint3);
}


// LED


// Overweight blink timing 
void updateOverweightLEDs() {
  static unsigned long phaseMs = 0;
  unsigned long now = millis();

  if (!(ow1 || ow2 || ow3)) {
    overLedPhase = false;
    phaseMs      = now;
    return;
  }
  if (!overLedPhase && phaseMs == 0) phaseMs = now;
  if ((now - phaseMs) >= OVERWEIGHT_LED_BLINK_MS) {
    overLedPhase = !overLedPhase;
    phaseMs      = now;
  }
}

// Bin-full logic is unchanged: ON only when FULL. Overweight blink takes priority while overweight.
void controlLED(int pin, const String &level, bool overweight) {
  if (overweight) digitalWrite(pin, overLedPhase ? HIGH : LOW);
  else            digitalWrite(pin, (level == "FULL") ? HIGH : LOW);
}


// BUZZER 


// Overweight siren: repeated ON/OFF tones with alternating pitch, for as long as any compartment is overweight.
void updateOverweightAlarm() {
  static bool          active = false;
  static bool          on     = false;
  static bool          high   = false;
  static unsigned long ms     = 0;
  unsigned long now = millis();

  if (!(ow1 || ow2 || ow3)) {
    active    = false;
    alarmTone = 0;
    return;
  }

  if (!active) {                                           
    active = true;
    on     = true;
    high   = true;
    ms     = now;
  } else if (on && (now - ms) >= OVERWEIGHT_SIREN_ON_MS) {
    on = false;
    ms = now;
  } else if (!on && (now - ms) >= OVERWEIGHT_SIREN_OFF_MS) {
    on   = true;
    high = !high;
    ms   = now;
  }

  alarmTone = on ? (high ? OVERWEIGHT_SIREN_FREQ_HI : OVERWEIGHT_SIREN_FREQ_LO) : 0;
}


void updateBinFullBeep() {
  static bool          active = false;
  static bool          state  = false;
  static unsigned long ms     = 0;
  unsigned long now = millis();

  bool anyFull = (curL1 == "FULL" || curL2 == "FULL" || curL3 == "FULL");

  if (!anyFull) {
    active   = false;
    state    = false;
    fullTone = 0;
    return;
  }

  if (!active) {                                        
    active = true;
    state  = true;
    ms     = now;
  } else if (state) {
    if (now - ms >= BUZZER_BEEP_ON)  { state = false; ms = now; }
  } else {
    if (now - ms >= BUZZER_BEEP_OFF) { state = true;  ms = now; }
  }

  fullTone = state ? FULL_BEEP_FREQ : 0;
}


bool tareBeepRequested = false;

void playTareConfirmationBeep() {
  tareBeepRequested = true;
}

void updateTareBeep() {
  if (tareBeepRequested) {
    tareBeepRequested = false;
    tareBeepActive    = true;
    tareBeepStartMs   = millis();
  } else if (tareBeepActive && (millis() - tareBeepStartMs) >= TARE_BEEP_MS) {
    tareBeepActive = false;
  }
  tareTone = tareBeepActive ? TARE_BEEP_FREQ : 0;
}



void updateWiFiConnectionBeep() {
  static bool          wasConnected = false;
  static uint8_t       phase        = 0;      
  static unsigned long phaseMs      = 0;
  unsigned long now = millis();

  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {                       
    wifiBeepActive = true;
    phase          = 0;
    phaseMs        = now;
  }
  if (!connected) wifiBeepActive = false;                  
  wasConnected = connected;

  if (!wifiBeepActive) {
    wifiTone = 0;
    return;
  }

  if ((now - phaseMs) >= WIFI_BEEP_MS) {
    phase++;
    phaseMs = now;
    if (phase >= 5) {                                
      wifiBeepActive = false;
      wifiTone       = 0;
      return;
    }
  }
  wifiTone = (phase % 2 == 0) ? WIFI_BEEP_FREQ : 0;
}


void applyBuzzer() {
  static uint16_t current = 0;
  uint16_t want;

  if (tareBeepActive)      want = tareTone;                
  else if (wifiBeepActive) want = wifiTone;
  else if (alarmTone)      want = alarmTone;


  //  Reset-Based Weight Removal Alert - START
  else if (removalTone)    want = removalTone;               // already 0 while any existing alert is active
  // Reset-Based Weight Removal Alert - END


  else                     want = fullTone;

  if (want != current) {
    ledcWriteTone(0, want);
    current = want;
  }
}

//  Reset-Based Weight Removal Alert - START

void updateRemovalTracker(WeightFilter &f, RemovalTracker &t, const char *name) {
  unsigned long now = millis();

  if (t.alertActive && (now - t.alertStartMs) >= REMOVAL_ALERT_DURATION_MS) {
    t.alertActive = false;                                 // normal operation resumes
  }

  // New Reset with weight on the load cell: remember it, cancel old alert of THIS compartment
  if (f.tareActive && (!t.lastTareActive || f.tareGrams != t.lastTareGrams)) {
    t.hidden      = f.tareGrams;
    t.alertActive = false;
  }
  t.lastTareActive = f.tareActive;
  t.lastTareGrams  = f.tareGrams;

  if (!f.tareActive || !f.tared) { t.hidden = 0.0f; return; }   // nothing hidden to watch

  float removed = t.hidden - (f.average + f.tareGrams);
  if (removed >= REMOVAL_ALERT_MIN_G) {
    float total   = f.average + f.tareGrams;
    t.hidden      = (total > 0.0f) ? total : 0.0f;         // what is still hidden (no duplicate alert)
    t.alertActive = true;
    t.alertStartMs = now;
    Serial.printf("[ALERT] %s: reset-hidden waste removed (about %.0f g).\n", name, removed);
  }
}

void updateResetRemovalAlerts() {
  updateRemovalTracker(plasticFilter, removalPlastic, "Plastic");
  updateRemovalTracker(foodFilter,    removalFood,    "Food");
  updateRemovalTracker(metalFilter,   removalMetal,   "Metal");
}



void applyResetRemovalLEDs() {
  const int pins[3]          = { LED1, LED2, LED3 };
  const RemovalTracker *t[3] = { &removalPlastic, &removalFood, &removalMetal };
  const bool over[3]         = { ow1, ow2, ow3 };
  unsigned long now = millis();

  for (uint8_t i = 0; i < 3; i++) {
    if (!t[i]->alertActive || over[i]) continue;
    bool on = (((now - t[i]->alertStartMs) / REMOVAL_LED_BLINK_MS) % 2) == 0;
    digitalWrite(pins[i], on ? HIGH : LOW);
  }
}


void updateResetRemovalBeep() {
  const RemovalTracker *t[3] = { &removalPlastic, &removalFood, &removalMetal };
  unsigned long now = millis();

  removalTone = 0;

  bool existingBuzzerBusy = (ow1 || ow2 || ow3 ||
                             curL1 == "FULL" || curL2 == "FULL" || curL3 == "FULL" ||
                             tareBeepActive || wifiBeepActive);
  if (existingBuzzerBusy) return;

  for (uint8_t i = 0; i < 3; i++) {
    if (!t[i]->alertActive) continue;
    unsigned long phase = (now - t[i]->alertStartMs) % REMOVAL_BEEP_CYCLE_MS;
    bool beepOn = (phase < REMOVAL_BEEP_MS) ||
                  (phase >= 2UL * REMOVAL_BEEP_MS && phase < 3UL * REMOVAL_BEEP_MS);
    if (beepOn) removalTone = REMOVAL_BEEP_FREQ;
  }
}
//  Reset-Based Weight Removal Alert - END

//  FIREBASE

void sendToFirebase(const String &l1, const String &l2, const String &l3,
                    float w1, float w2, float w3) {
  unsigned long now = millis();

  bool levelChanged  = (l1 != lastL1 || l2 != lastL2 || l3 != lastL3);
  bool weightChanged = (fabsf(w1 - lastW1) > WEIGHT_DEADBAND ||
                        fabsf(w2 - lastW2) > WEIGHT_DEADBAND ||
                        fabsf(w3 - lastW3) > WEIGHT_DEADBAND);
  bool overChanged   = (ow1 != lastOw1 || ow2 != lastOw2 || ow3 != lastOw3);   
  bool forceUpdate   = (now - prevFirebaseMillis >= FIREBASE_INTERVAL);

  if (!levelChanged && !weightChanged && !overChanged && !forceUpdate) return;

  prevFirebaseMillis = now;
  lastL1 = l1; lastL2 = l2; lastL3 = l3;
  lastW1 = w1; lastW2 = w2; lastW3 = w3;
  lastOw1 = ow1; lastOw2 = ow2; lastOw3 = ow3;

  Serial.println("[FB] Sending update...");

  FirebaseJson json;
  json.set("compartments/plastic/level",      l1);
  json.set("compartments/plastic/weight",     w1);
  json.set("compartments/plastic/overweight", ow1);   
  json.set("compartments/food/level",         l2);
  json.set("compartments/food/weight",        w2);
  json.set("compartments/food/overweight",    ow2);   // NEW
  json.set("compartments/metal/level",        l3);
  json.set("compartments/metal/weight",       w3);
  json.set("compartments/metal/overweight",   ow3);   // NEW
  json.set("lastUpdated/.sv",                 "timestamp");

  if (!Firebase.RTDB.updateNode(&fbdo, "/bins/bin001", &json)) {
    Serial.print("[FB] Error: ");
    Serial.println(fbdo.errorReason());
  }
}


//  SETUP

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);
  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  digitalWrite(TRIG3, LOW);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT_PULLUP);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER, 0);
  ledcWriteTone(0, 0);

  scale1.begin(DT1, SCK1);
  scale2.begin(DT2, SCK2);
  scale3.begin(DT3, SCK3);

  // Calibration factors (counts per gram) - defined once at the top of the file
  scale1.set_scale(CAL_PLASTIC);
  scale2.set_scale(CAL_FOOD);
  scale3.set_scale(CAL_METAL);

  Serial.println("[SCALE] Taring — keep bins empty...");
  delay(200);

  tareLoadCell(scale1, plasticFilter, TARE_ATTEMPTS);
  tareLoadCell(scale2, foodFilter,    TARE_ATTEMPTS);
  tareLoadCell(scale3, metalFilter,   TARE_ATTEMPTS);
  Serial.println("[SCALE] Tare done.");

  Serial.print("[WiFi] Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WiFi] Connected — IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Failed — running offline.");
  }

  config.api_key                    = API_KEY;
  config.database_url               = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("[FB] Firebase ready.");
}


//  LOOP 

void loop() {
  unsigned long now = millis();

  handleButtons();

  // Load cells are polled on EVERY pass (non-blocking). The HX711 delivers a new

  updateLoadCell(scale1, plasticFilter);
  updateLoadCell(scale2, foodFilter);
  updateLoadCell(scale3, metalFilter);

  // Alerts
  updateOverweightState();                 
  updateOverweightLEDs();                
  controlLED(LED1, curL1, ow1);           
  controlLED(LED2, curL2, ow2);
  controlLED(LED3, curL3, ow3);



  //  Reset-Based Weight Removal Alert - START
  updateResetRemovalAlerts();              // removal detection + alert timing
  applyResetRemovalLEDs();                 // blink only the affected compartment's LED while its alert runs
  // Reset-Based Weight Removal Alert - END


  updateOverweightAlarm();              
  updateBinFullBeep();                    
  updateTareBeep();       
  updateWiFiConnectionBeep();              


  // Reset-Based Weight Removal Alert - START
  updateResetRemovalBeep();                // beep-beep request (silent while another buzzer feature is active)
  // Reset-Based Weight Removal Alert - END
  applyBuzzer();                           // sole writer of the buzzer, applies priority


  if (now - prevSensorMillis < SENSOR_INTERVAL) return;
  prevSensorMillis = now;

  float d1 = getDistance(TRIG1, ECHO1);
  float d2 = getDistance(TRIG2, ECHO2);
  float d3 = getDistance(TRIG3, ECHO3);

  String l1 = getLevel(d1);
  String l2 = getLevel(d2);
  String l3 = getLevel(d3);

  curL1 = l1; curL2 = l2; curL3 = l3;      // cached for the per-pass LED / buzzer

  // Final stable weights (filtered, zero-protected, rounded) - same variables as before
  float w1 = plasticFilter.finalWeight;
  float w2 = foodFilter.finalWeight;
  float w3 = metalFilter.finalWeight;

  Serial.println("\n===== SMART BIN =====");
  Serial.printf("Plastic : %-6s | %6.1f g | %.1f cm\n", l1.c_str(), w1, d1);
  Serial.printf("Food    : %-6s | %6.1f g | %.1f cm\n", l2.c_str(), w2, d2);
  Serial.printf("Metal   : %-6s | %6.1f g | %.1f cm\n", l3.c_str(), w3, d3);

  if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
    sendToFirebase(l1, l2, l3, w1, w2, w3);
  }
}
