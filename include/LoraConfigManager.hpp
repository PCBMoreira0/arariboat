#pragma once
class LoraConfiguration {
public:
    long frequency;
    long bandwidth;
    int spreadingFactor;
    int codingRate;
    int syncWord;
    int power;
    bool crcEnabled;
};


// Preferences, SPIFFS or other type of NVS storage aren't shown on these headers, so client modules don't need to know the underlying storage mechanism.
bool TrySaveLoraConfiguration(const LoraConfiguration& config);
bool GetLoraConfiguration(LoraConfiguration& config);