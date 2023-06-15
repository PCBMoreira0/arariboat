#pragma once
#include <WiFi.h>
#include <HTTPClient.h>
//#include <ArduinoJson.h>

const char* targetURL = "[fc94:369e:f312:cf38:75f7:afee:58cf:3f36]:80";

// Send random voltage data via HTTP GET params

void SendGETRequest(String baseAddress, String route) { // param-value pair
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Not connected to WiFi");
        return;
    }
    HTTPClient client;
    String url = baseAddress + route;
    client.begin(url);
    client.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpCode = client.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = client.getString();
        Serial.println(payload);
    } else {
        Serial.println("Error on HTTP request");
    }
    client.end();
}

void SetGETRequest(String baseAddress, String route, String param, String value) {

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Not connected to WiFi");
        return;
    }
    HTTPClient client;
    String url = baseAddress + route + "?" + param + "=" + value;
    Serial.println("Sending GET request to " + url);
    client.begin(url);
    client.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpCode = client.GET();
    if (httpCode == HTTP_CODE_OK) {
        String payload = client.getString();
        Serial.println(payload);
    }
    else {
        // Log error information 
        Serial.printf("[HTTP] GET... failed, error: %s\n", client.errorToString(httpCode).c_str());
    }
}



