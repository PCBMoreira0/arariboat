#include <logger.hpp>
#include <Arduino.h>

Logger::Logger(const char* message) {
    Serial.println(message);
}

Logger::Logger(const char* message, unsigned long logInterval) {
    this->message = message;
    this->logInterval = logInterval;
    this->lastLogTime = millis();
}

void Logger::log(const char* message, unsigned long logInterval) {
    if (logInterval > 100 && logInterval != this->logInterval) {
        this->logInterval = logInterval;
    }

    if (millis() - this->lastLogTime >= this->logInterval) {
        Serial.println(message);
        this->lastLogTime = millis();
    }
}

void Logger::log(const char* message) {
    Serial.println(message);
}

void Logger::stop() {
    this->logInterval = 0;
}


