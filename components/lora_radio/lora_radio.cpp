#include "lora_radio.h"

LoRaRadio::LoRaRadio(GPIOPin *nss, GPIOPin *reset, GPIOPin *busy, GPIOPin *dio1)
: lora_module_(nss->to_pin_mode(), dio1->to_pin_mode(), reset->to_pin_mode(), busy->to_pin_mode()),
lora_(&lora_module_) {}

void LoRaRadio::setup() {
    int state = lora_.begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE("LoRa", "Failed to initialize LoRa! Error code: %d", state);
    } else {
        ESP_LOGI("LoRa", "LoRa initialized successfully!");
    }
}

void LoRaRadio::loop() {
    // Handle LoRa operations (send/receive messages)
}
