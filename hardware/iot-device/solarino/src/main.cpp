#include <Arduino.h>
#include <WiFi.h>
#include "WiFiHelper.h"
#include <PubSubClient.h>
#include <Wire.h>
#include "INA3221.h"
#include "secrets.h"

#define SOLAR_CELL_CHANNEL 3
#define LIPO_BATTERY_CHANNEL 2
#define OUTPUT_CHANNEL 1
#define SOLAR_SENSOR_PIN 35
#define PIR_PIN 14
#define LOAD_CONTROL_PIN 33
#define ONBOARD_LED_PIN 2
#define SOLAR_UPDATE_INTERVAL 5000

WiFiClient wifiClient;

WiFiHelper wifi;
INA3221 ina3221;

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

// MQTT stuff
const char* mqtt_server = "192.168.1.5";
PubSubClient mqttClient(wifiClient);
void callback(char* topic, byte* message, unsigned int length);
void reconnect();

void setup(void) 
{
    
  Serial.begin(115200);
  Serial.println("Solar Panel Monitor");

  pinMode(SOLAR_SENSOR_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  
  Serial.println("Setting up ina3221 ...");
  ina3221.begin();

  Serial.print("Manufactures ID=0x");
  int MID;
  MID = ina3221.getManufID();
  Serial.println(MID,HEX);
  wifi.connect(ssid, password); //SSID and password should be in secrets.h

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(LOAD_CONTROL_PIN, ledChannel);

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
  mqttClient.subscribe("solar/loadcontrol");
}

unsigned int lastPIRread = 0;
unsigned int motionCount = 0;

void measureAndSendMotion(){
  bool motion = digitalRead(PIR_PIN);
  if(motion){
    motionCount++;
  }
  if(millis() - lastPIRread > SOLAR_UPDATE_INTERVAL){
    lastPIRread = millis();
    char motString[8];
    dtostrf(motionCount, 1, 2, motString);
    mqttClient.publish("solar/movement", motString);
  }
}

unsigned int lastsolarread = 0;

void measureAndSendSolar(){
  if(millis() - lastsolarread > SOLAR_UPDATE_INTERVAL){
    lastsolarread = millis();
    float shuntvoltage1 = 0;
    float busvoltage1 = 0;
    float current_mA1 = 0;
    float loadvoltage1 = 0;
    int solarSensor = 0;
   


    busvoltage1 = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
    shuntvoltage1 = ina3221.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL);
    current_mA1 = ina3221.getCurrent_mA(LIPO_BATTERY_CHANNEL); 
    loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
    solarSensor = analogRead(SOLAR_SENSOR_PIN);

    Serial.print("LIPO_Battery Bus Voltage:   "); Serial.print(busvoltage1); Serial.println(" V");
    Serial.print("LIPO_Battery Shunt Voltage: "); Serial.print(shuntvoltage1); Serial.println(" mV");
    Serial.print("LIPO_Battery Load Voltage:  "); Serial.print(loadvoltage1); Serial.println(" V");
    Serial.print("LIPO_Battery Current 1:       "); Serial.print(current_mA1); Serial.println(" mA");
    Serial.println("");

    float shuntvoltage2 = 0;
    float busvoltage2 = 0;
    float current_mA2 = 0;
    float loadvoltage2 = 0;

    busvoltage2 = ina3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
    shuntvoltage2 = ina3221.getShuntVoltage_mV(SOLAR_CELL_CHANNEL);
    current_mA2 = ina3221.getCurrent_mA(SOLAR_CELL_CHANNEL);
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

    Serial.print("Solar Cell Bus Voltage 2:   "); Serial.print(busvoltage2); Serial.println(" V");
    Serial.print("Solar Cell Shunt Voltage 2: "); Serial.print(shuntvoltage2); Serial.println(" mV");
    Serial.print("Solar Cell Load Voltage 2:  "); Serial.print(loadvoltage2); Serial.println(" V");
    Serial.print("Solar Cell Current 2:       "); Serial.print(current_mA2); Serial.println(" mA");
    Serial.println("");

    float shuntvoltage3 = 0;
    float busvoltage3 = 0;
    float current_mA3 = 0;
    float loadvoltage3 = 0;

    busvoltage3 = ina3221.getBusVoltage_V(OUTPUT_CHANNEL);
    shuntvoltage3 = ina3221.getShuntVoltage_mV(OUTPUT_CHANNEL);
    current_mA3 = ina3221.getCurrent_mA(OUTPUT_CHANNEL);
    loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000);

    Serial.print("Output Bus Voltage 3:   "); Serial.print(busvoltage3); Serial.println(" V");
    Serial.print("Output Shunt Voltage 3: "); Serial.print(shuntvoltage3); Serial.println(" mV");
    Serial.print("Output Load Voltage 3:  "); Serial.print(loadvoltage3); Serial.println(" V");
    Serial.print("Output Current 3:       "); Serial.print(current_mA3); Serial.println(" mA");
    Serial.println("");

    char ma1String[8];
    dtostrf(current_mA1, 1, 2, ma1String);
    mqttClient.publish("solar/chargingcurrent", ma1String);

    char bv1String[8];
    dtostrf(busvoltage1, 1, 2, bv1String);
    mqttClient.publish("solar/chargingvoltage", bv1String);

    char ma2String[8];
    dtostrf(current_mA2, 1, 2, ma2String);
    mqttClient.publish("solar/inputcurrent", ma2String);

    char bv2String[8];
    dtostrf(busvoltage2, 1, 2, bv2String);
    mqttClient.publish("solar/inputvoltage", bv2String);

    char ma3String[8];
    dtostrf(current_mA3, 1, 2, ma3String);
    mqttClient.publish("solar/loadcurrent", ma3String);

    char bv3String[8];
    dtostrf(busvoltage3, 1, 2, bv3String);
    mqttClient.publish("solar/loadvoltage", bv3String);

    char ssString[8];
    dtostrf(solarSensor, 1, 2, ssString);
    mqttClient.publish("solar/irradiance", ssString);
  }
}

void loop(void) 
{
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  measureAndSendSolar();

  delay(100);
}


void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("Solar")) {
      Serial.println("connected");
      // Subscribe
      mqttClient.subscribe("solar/loadcontrol");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "solar/loadcontrol") {
    Serial.print("Changing output to ");
    ledcWrite(ledChannel, messageTemp.toInt());
    /*
    if(messageTemp == "on"){
      Serial.println("on");
      //digitalWrite(ONBOARD_LED_PIN, HIGH);
      ledcWrite(ledChannel, 100);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      //digitalWrite(ONBOARD_LED_PIN, LOW);
      ledcWrite(ledChannel, 50);
    }*/
  }
}
