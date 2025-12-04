#include <Wire.h>
#include "RTClib.h"
#include <DHT.h>
#include <TimeLib.h>
#include <JC_Sunrise.h>
#include <Servo.h>

// --- Pin setup ---
#define DHTPIN 4
#define DHTTYPE DHT11
#define RELAY_PIN 9
#define SERVO_PIN1 5
#define SERVO_PIN2 6

// --- Objects ---
DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;
Servo servo1;
Servo servo2;

// --- Location (Panama City) ---
constexpr float myLat {8.9833};
constexpr float myLon {-79.5167};
JC_Sunrise sun {myLat, myLon, JC_Sunrise::officialZenith};

int utcOffset = -300; // UTC -5 hours * 60

// --- Variables ---
float temp = 0.0;
float hum = 0.0;
float idealTemp = 26.0;
int sunrise, sunset;
bool isDay = false; // Track current state (day or night)
bool acOn = false;  // Track AC relay state

// --- Timing for AC ---
unsigned long lastACOnTime = 0;
const unsigned long AC_DELAY = 5UL * 60UL * 1000UL; // 5 minutes in ms

// --- Servo angle limits ---
const int servo1Day = 35;   // 90 → 35 (sunrise)
const int servo1Night = 90; // 35 → 90 (sunset)
const int servo2Day = 15;   // 115 → 15
const int servo2Night = 115;

// --- Timing for servo motion ---
const int steps = 100;
const int stepDelay = 50; // ms between steps (100×50 = 5 s total)

// -------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(__DATE__, __TIME__)); 
  dht.begin();

  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("System Initialized");

  calculateSunTimes();
}

// -------------------------------------------------------------------

void loop() {
  DateTime now = rtc.now();
  showRTCTime(now);
  TempHumMeasurement();
  checkServoPosition(now);
  ACSwitch();
  delay(5000);
}

// -------------------------------------------------------------------

void showRTCTime(DateTime t) {
  Serial.print("Time: ");
  if (t.hour() < 10) Serial.print("0");
  Serial.print(t.hour());
  Serial.print(":");
  if (t.minute() < 10) Serial.print("0");
  Serial.print(t.minute());
  Serial.print(":");
  if (t.second() < 10) Serial.print("0");
  Serial.println(t.second());
}

// -------------------------------------------------------------------

void TempHumMeasurement() {
  hum = dht.readHumidity();
  temp = dht.readTemperature();

  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" °C  Humidity: ");
  Serial.print(hum);
  Serial.println(" %");
}

// -------------------------------------------------------------------

void calculateSunTimes() {
  DateTime now = rtc.now();

  tmElements_t tm;
  tm.Year = now.year() - 1970;
  tm.Month = now.month();
  tm.Day = now.day();
  tm.Hour = now.hour();
  tm.Minute = now.minute();
  tm.Second = now.second();

  time_t localTime = makeTime(tm);
  sun.calculate(localTime, utcOffset, sunrise, sunset);

  Serial.print("Sunrise: ");
  Serial.print(sunrise);
  Serial.print(" | Sunset: ");
  Serial.println(sunset);
}

// -------------------------------------------------------------------

void moveServosSmooth(int from1, int to1, int from2, int to2) {
  for (int i = 0; i <= steps; i++) {
    float ang1 = from1 + (to1 - from1) * (i / 100.0);
    float ang2 = from2 + (to2 - from2) * (i / 100.0);
    servo1.write((int)ang1);
    servo2.write((int)ang2);
    delay(stepDelay);
  }
}

// -------------------------------------------------------------------

void checkServoPosition(DateTime now) {
  int currentMinutes = now.hour() * 60 + now.minute();

  // Sunrise and sunset are in minutes (e.g., 360 = 6:00 AM)
  if (currentMinutes >= sunrise && currentMinutes < sunset && !isDay) {
    Serial.println("Sunrise detected - moving servos to DAY position...");
    moveServosSmooth(servo1Night, servo1Day, servo2Night, servo2Day);
    isDay = true;
  }
  else if ((currentMinutes >= sunset || currentMinutes < sunrise) && isDay) {
    Serial.println("Sunset detected - moving servos to NIGHT position...");
    moveServosSmooth(servo1Day, servo1Night, servo2Day, servo2Night);
    isDay = false;
  }
}

// -------------------------------------------------------------------

void ACSwitch() {
  unsigned long now = millis();

  // If AC is OFF and temperature is above 26°C → Turn ON (with delay)
  if (!acOn && temp > 23.0) {
    digitalWrite(RELAY_PIN, HIGH);
    acOn = true;
    lastACOnTime = now;
    Serial.println("AC TURNED ON (Temp > 26°C)");
  }

  // If AC is ON and at least 5 minutes have passed
  if (acOn && (now - lastACOnTime >= AC_DELAY)) {
    if (temp <= 22.0) {
      digitalWrite(RELAY_PIN, LOW);
      acOn = false;
      Serial.println("AC TURNED OFF (Temp ≤ 22°C, after 5 min)");
    } else if (temp > 25.0) {
      Serial.println("AC remains ON (Temp > 25°C)");
    }
  }
}
