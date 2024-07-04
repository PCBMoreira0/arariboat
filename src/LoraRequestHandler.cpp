#include "LoraConfigManager.hpp"
#include "LoraRequestHandler.hpp"

#define HTTP_POST true
#define HTTP_SUCCESS 200
#define HTTP_CLIENT_ERROR 400
#define HTTP_SERVER_ERROR 500

static bool checkRequestParams(AsyncWebServerRequest* request) {
    
    if (!request->hasParam("frequency", HTTP_POST)       || 
        !request->hasParam("bandwidth", HTTP_POST)       || 
        !request->hasParam("spreadingFactor", HTTP_POST) || 
        !request->hasParam("codingRate", HTTP_POST)      || 
        !request->hasParam("syncWord", HTTP_POST)        || 
        !request->hasParam("power", HTTP_POST)           || 
        !request->hasParam("crcEnabled", HTTP_POST)) 
        {
            request->send(HTTP_CLIENT_ERROR, "text/plain", "Missing parameters.");
            return false;
        }

    return HTTP_POST;
}

static LoraConfiguration parseRequestParams(AsyncWebServerRequest* request) {
    
    LoraConfiguration config;
    config.frequency = request->getParam("frequency", HTTP_POST)->value().toInt();
    config.bandwidth = request->getParam("bandwidth", HTTP_POST)->value().toInt();
    config.spreadingFactor = request->getParam("spreadingFactor", HTTP_POST)->value().toInt();
    config.codingRate = request->getParam("codingRate", HTTP_POST)->value().toInt();
    config.syncWord = request->getParam("syncWord", HTTP_POST)->value().toInt();
    config.power = request->getParam("power", HTTP_POST)->value().toInt();
    config.crcEnabled = request->getParam("crcEnabled", HTTP_POST)->value().toInt();

    return config;
}

void handleLoraConfigRequest(AsyncWebServerRequest* request) {
    
    if (!checkRequestParams(request)) {
        return;
    }

    LoraConfiguration config = parseRequestParams(request);
    if (!TrySaveLoraConfiguration(config)) {
        request->send(HTTP_SERVER_ERROR, "text/plain", "Failed to save configuration.");
        return;
    }   

    request->send(HTTP_SUCCESS, "text/plain", "Configuration saved.");
}