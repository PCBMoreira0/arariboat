#include <Arduino.h>
#include "Utilities.hpp"
#include "HTTPClient.h"

#define HTTP_RESPONSE int

HTTP_RESPONSE postToInfluxDB(String data) {

    HTTPClient client;
    client.setTimeout(3000);
    client.begin("44.221.0.169", 8086, "/api/v2/write?bucket=Arariboia&org=Innomaker&precision=s");
    client.setReuse(false);

    client.addHeader("Content-Type", "text/plain; charset=utf-8");
    client.addHeader("Accept", "application/json");
    client.setAuthorizationType("Token");
    client.setAuthorization("gK8YfMaAl54lo2sgZLiM3Y5CQStHip-7mBe5vbhh1no86k72B4Hqo8Tj1qAL4Em-zGRUxGwBWLkQd1MR9foZ-g==");

    int http_response_code = client.POST((uint8_t*)data.c_str(), data.length());

    if (http_response_code > 0) {
        String response = client.getString();
        DEBUG_PRINTF("\n[HTTP][%d]Response\n%s\n", http_response_code, response.c_str());
    } else {
        DEBUG_PRINTF("\n[HTTP]Request failed, error: %s\n", client.errorToString(http_response_code).c_str());
    }

    return http_response_code;
}


