#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

// Pins
#define redPin 5
#define greenPin 4
#define bluePin 14
#define pumpPin 12
#define fanPin 15
#define tankLevelPin 16
#define moisturePin A0
// sensor temp - humidity
#define DHTPIN 13
#define DHTTYPE DHT11 // DHT 11
// Constants
#define T_MS 1
#define MAX_PUMP_PERC_POWER 10
#define TH_SENSOR_READING_PERIOD_MS 30000 //How often to publish Humidity, temperature measurement
#define MOISTURE_READING_PERIOD_MS 30000 //How often to publish moisture measurement
#define MOISTURE_TARGET_REACHED_MS 15000 // Time to wait to ensure that the target moisture has been effectively reached
#define MAX_MOISTURE_ATTEMPT_TIME_MS 30000 // If the moisture target is not reached after a fixed amount of time, stop the closed loop control
#define PUB_STATUS_PERIOD_MS 2000

typedef struct {
  int moistureLevel;
  unsigned long timestamp;
} MoistureTarget;

typedef struct {
  float lightRGBStatus[3];
  float pumpStatus;
  float fanStatus;
  bool tankFull;
} GhCStatus;

// WiFi
const char *ssid = "CasermaBis"; // Enter your WiFi name
const char *password = "A$pr0m0nt345";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.1.52";
const int mqtt_port = 1881;
const char* brokerID = "GhController";
const char* brokerPsw = "3t54*4^0VtR%"; 

const char *logTopic = "GhC/log";
const char *humTopic = "GhC/hum";
const char *tempTopic = "GhC/temp";
const char *lightRGBTopic = "GhC/RGB";
const char *pumpTopic = "GhC/pump";
const char *pumpDefaultValTopic = "GhC/pump/val";
const char *fanTopic = "GhC/fan";
const char *moistureTopic = "GhC/moist";
const char *moistureTargetTopic = "GhC/moist/target";
const char *moistureReqTopic = "GhC/moisture/req";

unsigned long lastTHRead = 0; 
unsigned long lastMoistureRead = 0;
unsigned long lastPubStatus = 0;
long moistureTargetReachedTS = 0;
short pumpDefaultValue = 4; // Power value to set the pump during the moisture closed loop control
bool tankFull = false;
GhCStatus status;
MoistureTarget *moistureTarget;

DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {

  lastTHRead = 0; 
  lastMoistureRead = 0;
  pumpDefaultValue = 4;
  moistureTargetReachedTS = -1;
  moistureTarget = NULL;
  // init status
  status.lightRGBStatus[0] = 0;
  status.lightRGBStatus[1] = 0;
  status.lightRGBStatus[2] = 0;
  status.pumpStatus = 0;
  status.fanStatus = 0;
  status.tankFull = false;

  // Set software serial baud to 115200;
  Serial.begin(115200);
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    connectToBroker();
  }
  // Subscribers
  client.subscribe(lightRGBTopic);
  client.subscribe(pumpTopic);
  client.subscribe(fanTopic);
  client.subscribe(moistureTargetTopic);
  
  // Pins  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(moisturePin, INPUT);
  pinMode(tankLevelPin, INPUT);

  dht.begin();

  delay(1500);
  client.publish(logTopic, "Init GhC"); // "Init GhC" is the string that enables the initialization from node-red 
}

/***
  * Convert an array of byte in a string.
  *
  * @payload: array of byte containing the original input 
  * @length: size of the payload array
  * @payloadString: ampety array of string of size equal to (length + 1) where to save the string 
  */

void convertPayloadInString(byte* payload, unsigned int length, char* payloadString) {
  for (int i = 0; i < length; i++) {
      payloadString[i] =(char) payload[i];
  }
  payloadString[length] = '\0';
}

uint32_t getPercentagefromPayload(byte* payload, unsigned int length) {
  uint32_t percentage = 0;
  char payloadString[length + 1];
  convertPayloadInString(payload, length, payloadString);
  int n = atoi(payloadString);

  if (n > 100) return 100;
  if (n < 0) return 0;
  return n;
}

void connectToBroker() {
  String client_id = "Greenhouse-client-";
  client_id += String(WiFi.macAddress());
  Serial.printf("The client %s is trying to connect to the mqtt broker\n", client_id.c_str());
  if (client.connect(client_id.c_str(), brokerID, brokerPsw)) {
      Serial.println("GhC connected to mqtt broker");
      client.publish(logTopic, "GhC connected to mqtt broker");
  } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
  }
}

void checkConnection() {
  if(WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      Serial.print("GhC internet connection lost: trying to reconnect to WIFI.");
  }
  if  (!client.connected()){
    connectToBroker();
    Serial.print("Connection with MQTT broker lost: trying to reconnect to the broker.");
  }
}

void setMoistureTarget(int moistureLevel) {
  if (moistureTarget != NULL) {
    resetMoistureTarget();
  }
  Serial.println("Set moisture target: ");
  Serial.println(String(moistureLevel));
  moistureTarget = (MoistureTarget*) malloc(sizeof(MoistureTarget));
  moistureTarget->moistureLevel = moistureLevel;
  moistureTarget->timestamp = millis();
}

void resetMoistureTarget() {
  free(moistureTarget);
  moistureTarget = NULL;
  moistureTargetReachedTS = -1;
  Serial.println("Reset moisture target.");
}

bool reachMoistureLevel() {
  // if moisture closed loop control is running
  if(moistureTarget) {
    // Get the moisture measurement
    float moistMeas = getMoistureMeasurement();
    unsigned long now = millis();

    // if the moisture level is too low the sensor should be turn off
    // for security reason do not activate the moisture closed loop control
    if (moistMeas < 2) {
      Serial.print("Moisture sensor NOT connected.");
      resetMoistureTarget();
      return false;
    }     

    // Moisture closed loop control's final conditions
    if (moistureTargetReachedTS != -1 && now - moistureTargetReachedTS > MOISTURE_TARGET_REACHED_MS ) {
      // reaching target moisture level success
      setPumpLevel(0);
      Serial.print("Moisture closed loop control finished succesfully.");
      client.publish(logTopic, "INFO: Moisture closed loop control finished succesfully.");
      resetMoistureTarget();
      return false;
    } else if ( now - moistureTarget->timestamp > MAX_MOISTURE_ATTEMPT_TIME_MS) {
      // reaching target moisture level failure
      setPumpLevel(0);
      Serial.print("Moisture control FAILED: the requested moisture level can NOT be reached.");
      client.publish(logTopic, "Moisture Control FAILED: the requested moisture level can NOT be reached.");
      resetMoistureTarget();
      return false;
    }

    // Moisture target level is reached, watering can be stopped
    if(moistMeas > moistureTarget->moistureLevel) {
      Serial.print("Moisture level reached.\n");
      setPumpLevel(0);
      if (moistureTargetReachedTS == -1) {
        // if target is reached now, save the timestamp
        moistureTargetReachedTS = millis();
      }      
    } else if (moistMeas < moistureTarget->moistureLevel - 2 /*add an offset for double threshold control*/) {
      // moisture target is not reached, watering must be activated
      setPumpLevel(pumpDefaultValue);
      moistureTargetReachedTS = -1;
      Serial.print("Pump activated.");

    }
    return true;
  }
  // no moisture target is set, moisture closed loop control is not running
  return false;
}

void setNewMoistureTarget(int moistureTarget) {
  setMoistureTarget(moistureTarget);
}

float getMoistureMeasurement() {
  return  100 * (1024 - analogRead(moisturePin) ) / 1024;
}

void publishMoistureMeasurement() { 
  // recover from overflow
  if (millis() - lastTHRead < 0){
    lastMoistureRead = millis();
  }

  if(millis() - lastMoistureRead > MOISTURE_READING_PERIOD_MS) {
    lastMoistureRead = millis();

    // Reade moisture sensor value
    float moisPerc = getMoistureMeasurement();
    char buffer[5] = "";
    dtostrf(moisPerc, 5, 2, buffer);

    Serial.println("Moisture: ");
    Serial.println(buffer);
    client.publish(moistureTopic, buffer);
  }
}

void publishTempHumMeasurements() {
  // recover from overflow
  if (millis() - lastTHRead < 0){
    lastTHRead = millis();
  }

  if(millis() - lastTHRead > TH_SENSOR_READING_PERIOD_MS) {
    lastTHRead = millis();

    float h = dht.readHumidity();
    // temperature is measured
    float t = dht.readTemperature();
    
    // Checking if the measurements have passed without errors
    // if an error is detected, a error message is displayed here
    if (isnan(h) || isnan(t)) {
      Serial.println("Error reading the KY-015 sensor");
    } else {
      
      char buffer[5] = "";
      dtostrf(t, 5, 2, buffer);
      Serial.println("Temperature: ");
      Serial.println(buffer);
      client.publish(tempTopic, buffer);

      dtostrf(h, 5, 2, buffer);
      Serial.println("Humidity: ");
      Serial.println(buffer);
      client.publish(humTopic, buffer);
    }
  }
}

void getTankLevelMeasurement() {
  bool isTankFullNow = isTankFull();
  // STOP water pump if tank is empty
  if (!isTankFullNow) analogWrite(pumpPin, 0);

  if (isTankFullNow != status.tankFull) {
    if (isTankFullNow){
      Serial.println("Water tank full. ");
    } else {
          Serial.println("Water tank empty. ");
    }
    status.tankFull = isTankFullNow;
  }
}

bool isTankFull() {
  return digitalRead(tankLevelPin) != HIGH;
}

void setLightColors(byte* payload, unsigned int length) {
  char payloadString[length + 1];
  convertPayloadInString(payload, length, payloadString);

  char *delim = ",";
  int count = 0;
  
  // Get the first value before the comma  
  char *token = strtok(payloadString, delim);
  count++;
  while ( token != NULL) {
    if (count > 3 || count - 1 < 0) {
      Serial.println("ERROR RGB: too many arguments");
      return;
    }
    status.lightRGBStatus[count - 1] = atoi(token);
    // Take the next value between commas
    token = strtok(NULL, delim);
    count++;
  }

  // Set the RGB Led values  
  analogWrite(redPin, status.lightRGBStatus[0] * 255 / 100);
  analogWrite(greenPin, status.lightRGBStatus[1] * 255 / 100);
  analogWrite(bluePin, status.lightRGBStatus[2] * 255 / 100);
}

void setFan(int fanPercentage){
  status.fanStatus = (float) fanPercentage;
  analogWrite(fanPin, status.fanStatus * 255 / 100);
}

void setPumpLevel(int intensityPerc) {
  if(intensityPerc > 0) {
    
    // check if there is water in the tank
    if(status.tankFull && isTankFull()) {
      status.pumpStatus = (float) intensityPerc;
      analogWrite(pumpPin, status.pumpStatus * 255 / 100);
      Serial.print("Pump activated with pwm: ");
      Serial.print(String(status.pumpStatus * 255 / 100));
      if (intensityPerc > MAX_PUMP_PERC_POWER) {
        Serial.print("Pump activated with an high value.");
        client.publish(logTopic, "WARNING: pump activated with an high power level.");
      }
    } else {
      Serial.print("WARNING: the water tank is empty, please refill with water.");
      client.publish(logTopic, "WARNING: the water tank is empty, please refill with water.");
      status.pumpStatus = 0;
      analogWrite(pumpPin, 0);
    }
  } else {
    status.pumpStatus = 0;
    analogWrite(pumpPin, 0);
    Serial.print("Pump STOPPED.\n");
  }  
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
  }
  Serial.println();
  
  if (!strcmp(topic, lightRGBTopic)) {
    setLightColors(payload, length);
  } else if (!strcmp(topic, pumpTopic)) {
    // If a manual pump command is sent, reset the moisture closed loop control
    resetMoistureTarget();
    setPumpLevel(getPercentagefromPayload(payload, length));
  } else if (!strcmp(topic, fanTopic)) {
    setFan(getPercentagefromPayload(payload, length));
  } else if (!strcmp(topic, moistureTargetTopic)) {
    setNewMoistureTarget(getPercentagefromPayload(payload, length));
  } else if (!strcmp(topic, moistureReqTopic)) {
    publishMoistureMeasurement();
  } else if (!strcmp(topic, pumpDefaultValTopic)) {
    pumpDefaultValue = getPercentagefromPayload(payload, length);
  }
}

void publishStatus() {
  if (millis() - lastPubStatus < 0){
    lastPubStatus = millis();
  }

  if(millis() - lastPubStatus > PUB_STATUS_PERIOD_MS) {
    lastPubStatus = millis();
    client.publish((String(pumpTopic)+String("/status")).c_str(), String(status.pumpStatus).c_str());
    client.publish((String(fanTopic)+String("/status")).c_str(), String(status.fanStatus).c_str());
    client.publish((String(lightRGBTopic)+String("/status")).c_str(), (String(status.lightRGBStatus[0]) + String(", ") +
                                                                     String(status.lightRGBStatus[1]) + String(", ") +
                                                                     String(status.lightRGBStatus[2]) ).c_str());    
  }
}

void loop() {
  checkConnection();
  // Update MQTT client
  client.loop();
  getTankLevelMeasurement();
  if (reachMoistureLevel()) {
    // When a moisture target is reaching the computation frequency is higher
    delay(T_MS/2);
  } else {
    // not reaching a moisture target, the computation frequency can be lower
    delay(T_MS);
  }
  // Publish sensors measurements
  publishTempHumMeasurements();
  publishMoistureMeasurement();
  // publish current status of the actuators
  publishStatus();
}

