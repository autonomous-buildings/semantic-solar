#include <WiFi.h>

class WiFiHelper{
    public:
    void connect(const char* ssid, const char* password);
    bool isConnected();
    void disconnect();
};