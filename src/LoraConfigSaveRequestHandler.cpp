#include "LoraConfigManager.hpp"
#include "LoraRequestHandlers.hpp"

static bool checkRequestParams(AsyncWebServerRequest* request) {
    
    if (!request->hasParam("frequency", HTTP_POST_BOOL)       || 
        !request->hasParam("bandwidth", HTTP_POST_BOOL)       || 
        !request->hasParam("spreadingFactor", HTTP_POST_BOOL) || 
        !request->hasParam("codingRate", HTTP_POST_BOOL)      || 
        !request->hasParam("syncWord", HTTP_POST_BOOL)        || 
        !request->hasParam("power", HTTP_POST_BOOL)           || 
        !request->hasParam("crcEnabled", HTTP_POST_BOOL)) 
        {
            request->send(HTTP_CLIENT_ERROR, "text/plain", "Missing parameters.");
            return false;
        }

    return HTTP_POST_BOOL;
}

static LoraConfiguration parseRequestParams(AsyncWebServerRequest* request) {
    
    LoraConfiguration config;
    config.frequency = request->getParam("frequency", HTTP_POST_BOOL)->value().toInt();
    config.bandwidth = request->getParam("bandwidth", HTTP_POST_BOOL)->value().toInt();
    config.spreadingFactor = request->getParam("spreadingFactor", HTTP_POST_BOOL)->value().toInt();
    config.codingRate = request->getParam("codingRate", HTTP_POST_BOOL)->value().toInt();
    config.syncWord = request->getParam("syncWord", HTTP_POST_BOOL)->value().toInt();
    config.power = request->getParam("power", HTTP_POST_BOOL)->value().toInt();
    config.crcEnabled = request->getParam("crcEnabled", HTTP_POST_BOOL)->value().toInt();

    return config;
}

void handleLoraConfigSaveRequest(AsyncWebServerRequest* request) {
    
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