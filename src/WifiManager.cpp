#include "WiFiManager.hpp"

void WifiManager::SetSSIDCallback(GetSSIDCallback callback) {
    GetSSIDCallback_ = callback;
}

void WifiManager::SetIPCallback(GetIPCallback callback) {
    GetIPCallback_ = callback;
}

void WifiManager::SetRSSICallback(GetRSSICallback callback) {
    GetRSSICallback_ = callback;
}

String WifiManager::GetSSID() const {
    if (GetSSIDCallback_ == nullptr) {
        return "";
    }

    return GetSSIDCallback_();
}

String WifiManager::GetIP() const {
    if (GetIPCallback_ == nullptr) {
        return "";
    }

    return GetIPCallback_();
}

int8_t WifiManager::GetRSSI() const {
    if (GetRSSICallback_ == nullptr) {
        return 0;
    }

    return GetRSSICallback_();
}

WifiManager wifiManager;