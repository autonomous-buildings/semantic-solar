#include "WiFiHelper.h"
#include <Arduino.h>
#include <WiFi.h>

void WiFiHelper::connect(const char* ssid, const char* password){
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

bool WiFiHelper::isConnected(){
    return WiFi.status() == WL_CONNECTED;
}

void WiFiHelper::disconnect(){
    
}
