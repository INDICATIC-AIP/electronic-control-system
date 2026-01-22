// INDICATIC AIP
// PROJECT: First monitoring station for light pollution in Panama: integrating terrestrial and remote techniques.
// Dome control and air conditioning system for astronomical data acquisition capsules, within the Monitoring
// and Measurement Station for Light Pollution.
// Date: 12/18/2025
// Variante sin MQTT (hardware y NTP intactos). Cambia USE_MQTT a 1 para reactivar MQTT.

#define USE_MQTT 0

// Libraries
#include <WiFi.h>
#include <time.h>
#include <DHT.h>
#include <WiFiClient.h>
#if USE_MQTT
#include <PubSubClient.h>
#endif
#include <ESP32Servo.h>
// Temperature Sensor and Relay for Air Conditioning Control
#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float temp = 0.0;
float humd = 0.0;
unsigned long lastdht22 = 0;
#define RELAYPIN 26
bool air_ON  = false;
unsigned long lastAir_time = 0 ;
const unsigned long Air_delay = 5UL * 60UL * 60UL * 1000UL;
// NTP and WiFi section's
const char* ssid     = "";
const char* password = "";
#if USE_MQTT
const char* mqtt_server = ""; // <- change to your broker IP
const char* mqtt_user = ""; // optional
const char* mqtt_password = ""; // optional
const int mqtt_port = 1883;
#endif
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;
unsigned long lastNtpSync = 0;
struct tm timeinfo;
// MQTT
#if USE_MQTT
const char* MQTT_TOPIC_BROADCAST = "domo/command";
const char* MQTT_TOPIC_TEMPLATE = "domo/%s/command";
const char* MQTT_TOPIC_STATUS_TEMPLATE = "domo/%s/status";
const char* MQTT_TOPIC_TEMPERATURE_TEMPLATE = "domo/%s/temperature";
const char* DEVICE_ID = "qhy"; // change per device
WiFiClient espClient;
PubSubClient client(espClient);
#endif
// Servo setting
#define SERVO_PIN1 19
#define SERVO_PIN2 18
Servo servo1;
Servo servo2;
bool isDay = false;
const int servo1Day = 35;
const int servo1Night = 90;
const int servo2Day = 15;
const int servo2Night = 155;
const int steps = 100;
const int step_delay = 50;
// SETUP
void setup(){
  Serial.begin(115200);
  log_message("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    log_message(".");
  }
  log_message("");
  log_message("WiFi connected.");
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime(true);
  log_message("DHT22 test!");
  dht.begin();
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW); // Apaga el relay al iniciar
  log_message("Relay turn off");
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
#if USE_MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback([](char* topic, byte* payload, unsigned int length){
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
    msg.toLowerCase();
    log_message("MQTT msg [" + String(topic) + "] " + msg);
    if (msg == "open") {
      open();
      log_message("Command: open -> executed");
      publish_ack("open");
    } else if (msg == "close") {
      close();
      log_message("Command: close -> executed");
      publish_ack("close");
    } else {
      log_message("Unknown command");
    }
  });
#endif
  close();
  log_message("System Initialized");
}

void loop(){
  delay(1000);
  printLocalTime(false);
#if USE_MQTT
  if (!client.connected()) {
    while (!client.connected()) {
      log_message("Attempting MQTT connection...");
      String clientId = "ESP32Client-";
      clientId += String((uint32_t)esp_random(), HEX);
      bool ok;
      if (mqtt_user && strlen(mqtt_user) > 0) {
        ok = client.connect(clientId.c_str(), mqtt_user, mqtt_password);
      } else {
        ok = client.connect(clientId.c_str());
      }
      if (ok) {
        log_message("connected");
        client.subscribe(MQTT_TOPIC_BROADCAST);
        char topic[64];
        snprintf(topic, sizeof(topic), MQTT_TOPIC_TEMPLATE, DEVICE_ID);
        client.subscribe(topic);
      } else {
        log_message("failed, rc=" + String(client.state()) + " try again in 5 seconds");
        delay(5000);
      }
    }
  }
  client.loop();
#endif
  dht22(false);
  air(); // <-- comenta esta línea para que no se apague el relay
}

void printLocalTime(bool force){
  unsigned long now = millis();
  if (!force && now - lastNtpSync < 60000) return;
  if(!getLocalTime(&timeinfo)){
    log_message("Failed to obtain time");
    return;
  }
  char buf[64];
  strftime(buf, sizeof(buf), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  log_message(String(buf));
  lastNtpSync = now;
}

void dht22(bool force) {
  temp =  dht.readTemperature();
  humd = dht.readHumidity();
  unsigned long now = millis();
  if (!force && now - lastdht22 < 60000) return;
  if (isnan(humd) || isnan(temp)){
    log_message("Failed to read from DHT22 sensor!");
    return;
  }
  log_message("Humidity: " + String(humd) + "% , Temp: " + String(temp) + "°C");
#if USE_MQTT
  if (client.connected()) {
    char ttopic[64];
    snprintf(ttopic, sizeof(ttopic), MQTT_TOPIC_TEMPERATURE_TEMPLATE, DEVICE_ID);
    char payload[32];
    snprintf(payload, sizeof(payload), "%.2f", temp);
    client.publish(ttopic, payload);
    log_message("Published temperature " + String(payload) + " to " + String(ttopic));
  } else {
    log_message("MQTT not connected: temperature not published");
  }
#endif
  lastdht22 = now;
}

#if USE_MQTT
void publish_ack(const char* action) {
  char topic[64];
  snprintf(topic, sizeof(topic), MQTT_TOPIC_STATUS_TEMPLATE, DEVICE_ID);
  if (client.connected()) {
    client.publish(topic, action);
    log_message("Published ack '" + String(action) + "' to " + String(topic));
  } else {
    log_message("Cannot publish ack: MQTT not connected");
  }
}
#endif
// 3. Relay control
void air() {
  unsigned long now = millis();

  if (!air_ON && temp >= 22.0 ) {
    digitalWrite(RELAYPIN, HIGH); // Cambia a LOW si tu relay es activo bajo
    air_ON = true;
    lastAir_time = now;
    log_message("Air A. is turn on");
  }
  if (air_ON && (now - lastAir_time >= Air_delay)) {
    if (temp <= 21.0) {
      digitalWrite(RELAYPIN, LOW); // Cambia a HIGH si tu relay es activo bajo
      air_ON = false;
      log_message("Air. A. is turn off");
    }
  }
}
// 4. Movement function
void mov_Servos(int from1, int to1, int from2, int to2) {
  for (int i=0; i<= steps; i++){
    float ang1 = from1 + (to1 - from1) * (i/100.0);
    float ang2 = from2 + (to2 - from2) * (i/100.0);
    servo1.write((int)ang1);
    servo2.write((int)ang2);
    delay(step_delay);
  }
}
// 5. Astronomical functions removed (control now via MQTT)
// 7. Open and Close
void open() {
  mov_Servos(servo1Night, servo1Day, servo2Night, servo2Day);
  isDay = false;   // night
}
void close(){
  mov_Servos(servo1Day, servo1Night, servo2Day, servo2Night);
  isDay = true;    // day
}

// Log function: prints to Serial and optionally publishes to MQTT log topic
void log_message(const String& msg) {
  Serial.println(msg);
#if USE_MQTT
  if (client.connected()) {
    client.publish("domo/log", msg.c_str());
  }
#endif
}
