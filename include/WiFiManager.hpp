#pragma once
#include <Arduino.h>
#include <functional>

//https://www.metageek.com/training/resources/understanding-rssi/
#define WIFI_RSSI_MINIMUM -100

class WifiManager {
public:

    WifiManager(const WifiManager&) = delete; // Delete copy constructor
    WifiManager& operator=(const WifiManager&) = delete; // Delete copy assignment

    using GetSSIDCallback = std::function<String()>;
    using GetIPCallback = std::function<String()>;
    using GetRSSICallback = std::function<int8_t()>;

    static WifiManager& GetInstance() {
        static WifiManager instance;
        return instance;
    }

    inline void SetSSIDCallback(GetSSIDCallback callback) {
        GetSSIDCallback_ = callback;
    }

    inline void SetIPCallback(GetIPCallback callback) {
        GetIPCallback_ = callback;
    }

    inline void SetRSSICallback(GetRSSICallback callback) {
        GetRSSICallback_ = callback;
    }

    inline String GetSSID() const {
        return GetSSIDCallback_ ? GetSSIDCallback_() : "No SSID";
    }

    inline String GetIP() const {
        return GetIPCallback_ ? GetIPCallback_() : "No IP";
    
    }
    
    inline int8_t GetRSSI() const {
       return GetRSSICallback_ ? GetRSSICallback_() : WIFI_RSSI_MINIMUM;
    }

private:

    WifiManager() = default;
    GetSSIDCallback GetSSIDCallback_;
    GetIPCallback GetIPCallback_;
    GetRSSICallback GetRSSICallback_;
};

