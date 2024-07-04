#ifndef WIFI_MANAGER_HPP
#define WIFI_MANAGER_HPP
#include <Arduino.h>
#include <functional>

class WifiManager {
public:
    using GetSSIDCallback = std::function<String()>;
    using GetIPCallback = std::function<String()>;
    using GetRSSICallback = std::function<int8_t()>;

    void SetSSIDCallback(GetSSIDCallback callback);
    void SetIPCallback(GetIPCallback callback);
    void SetRSSICallback(GetRSSICallback callback);

    String GetSSID() const;
    String GetIP() const;
    int8_t GetRSSI() const;

private:
    GetSSIDCallback GetSSIDCallback_;
    GetIPCallback GetIPCallback_;
    GetRSSICallback GetRSSICallback_;
};

extern WifiManager wifiManager;
#endif