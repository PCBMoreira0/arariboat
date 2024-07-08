#include "LoraRequestHandlers.hpp"
#include "arariboat/mavlink.h"
#include "MavlinkUtilities.hpp"
#include "radio.hpp"

static bool checkRequestParams(AsyncWebServerRequest* request) {

    if (!request->hasParam("messageId", HTTP_POST_BOOL)) {
        request->send(400, "text/plain", "Missing message ID.");
        return false;
    }

    return true;
}

static int parseRequestParams(AsyncWebServerRequest* request) {
    return request->getParam("messageId", HTTP_POST_BOOL)->value().toInt();
}

void handleLoraTransmitRequest(AsyncWebServerRequest* request) {

    if (!checkRequestParams(request)) {
        return;
    }

    int mavlink_message_id = parseRequestParams(request);
    mavlink_message_t message;
    if (!GenerateMavlinkMessage(message, mavlink_message_id)) {
        request->send(HTTP_CLIENT_ERROR, "text/plain", "Invalid message ID.");
        return;
    }

    EnqueueMavlinkMessage(message, radioQueue);
    request->send(HTTP_SUCCESS, "text/plain", "Message sent.");
}