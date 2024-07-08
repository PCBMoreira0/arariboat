#pragma once
#include "ESPAsyncWebServer.h"

#define HTTP_POST_BOOL true
#define HTTP_SUCCESS 200
#define HTTP_CLIENT_ERROR 400
#define HTTP_SERVER_ERROR 500

void handleLoraConfigSaveRequest(AsyncWebServerRequest* request);
void handleLoraTransmitRequest(AsyncWebServerRequest* request);
