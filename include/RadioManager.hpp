#pragma once
#include <functional>
#include "arariboat/mavlink.h"

//https://lora.readthedocs.io/en/latest/#:~:text=RSSI%20minimum%20%3D%20%2D120%20dBm.
#define LORA_RSSI_MINIMUM -120

class RadioManager {
public:

    RadioManager(const RadioManager&) = delete; // Delete copy constructor
    RadioManager& operator=(const RadioManager&) = delete; // Delete copy assignment

    using GetRSSICallback = std::function<int()>;

    static RadioManager& GetInstance() {
        static RadioManager instance;
        return instance;
    }

    void SetRSSICallback(GetRSSICallback callback) {
        GetRSSICallback_ = callback;
    }

    void SetLastMessageName(const String& name) {
        lastMessageName_ = name;
    }

    void SetSequence(int sequence) {
        lastSequence_ = sequence;
    }

    String GetLastMessageName() {
        return lastMessageName_;
    }

    int GetRSSI() const {
        return GetRSSICallback_ ? GetRSSICallback_() : LORA_RSSI_MINIMUM;
    }

    int GetSequence() const {
        return lastSequence_;
    }

private:

    RadioManager() = default;
    GetRSSICallback GetRSSICallback_;

    String lastMessageName_ = "";
    int lastSequence_ = 0;
};

