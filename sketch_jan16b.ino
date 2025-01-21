
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define ACS712_PIN 34 // Broche analogique pour le capteur ACS712
#define SUPPLY_VOLTAGE 3.3 // Tension d'alimentation (en volts)
#define PIN_MQ2 35  // Broche analogique pour lecture A0
//#define GAS_SENSOR_DIGITAL_PIN 14 // Broche numérique pour lecture D0

// Wi-Fi credentials
const char* ssid = "Redmi 13C";
const char* password = "99142992";
const int pirPin = 13;   // Broche du capteur PIR
const int ledPin = 12;   // Broche de la LED et la lampe
const int led1Pin = 14;


// Sensibilité en fonction du modèle ACS712
// 0.185 pour ACS712 5A, 0.1 pour 20A, 0.066 pour 30A
float sensitivity = 0.185;
int value;
// MQTT broker details private
const char* mqtt_broker = "63f344ae04a040319da6d1da195a3803.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "mymqtt";
const char* mqtt_password = "mymqttIOT1";

// MQTT topics
const char* topic_publish_ir = "subscribe1";
const char* topic_publish_gaz = "subscribe";
const char* topic_subscribe = "publish"; // Topic to receive messages



// Create instances
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Variables for timing
long previous_time = 0;

// Callback function to handle incoming messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin, LOW);

}

void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker.");

      // Subscribe to the control topic
      mqttClient.subscribe(topic_subscribe);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT); // Configure le capteur PIR comme entrée
  pinMode(ledPin, OUTPUT); // Configure la LED comme sortie
  digitalWrite(ledPin, LOW); // Éteint la LED au démarrage
  digitalWrite(led1Pin, HIGH);
  analogReadResolution(12); // Résolution ADC 12 bits
  pinMode(PIN_MQ2, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");

  // Initialize secure WiFiClient
  wifiClient.setInsecure(); // Use this only for testing, it allows connecting without a root certificate
  
  setupMQTT();


}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  long now = millis();
  static long last_publish_time = 0;
  static float total = 0.0;
  if (now - previous_time > 1000) { // Publish every 10 seconds
    
    previous_time = now;
    int pirState = digitalRead(pirPin);
    if (pirState == HIGH){
      Serial.println("Présence détectée ! LED allumée.");
      digitalWrite(ledPin, HIGH); // Allume la LED
      delay(3000);
    

    }else{
      Serial.println("Aucune présence détectée. LED éteinte.");
      digitalWrite(ledPin, LOW); // Éteint la LED
      delay(1000);
    }
    int rawValue = analogRead(ACS712_PIN);
    float voltage = abs(rawValue) * (3.3 / 4095.0);
    float current = (voltage - 2) / 0.185;
    float power = current * SUPPLY_VOLTAGE;
    float duration = (now - last_publish_time) / 1000.0;
    total += power * (duration / 3600.0);
    last_publish_time = now;
    // Publication des données d'énergie
      char energy_message[50];
      snprintf(energy_message, sizeof(energy_message), " %.2f ", total);
      mqttClient.publish(topic_publish_ir, energy_message);

      Serial.println(energy_message); // Affichage en série


    
    value = analogRead(PIN_MQ2);
    Serial.println("VALUE - " + String(value));
    if (value > 1300){
      Serial.println("Gaz detected !!");
      mqttClient.publish(topic_publish_gaz, "Gaz detected !! ");
    }else {
      Serial.println("No Gaz detected");
    }
    delay(3000);
  }
} 
