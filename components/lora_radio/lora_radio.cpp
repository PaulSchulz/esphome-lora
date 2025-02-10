#include "esphome/core/log.h"
#include "lora_radio.h"

// #include "SPI.h"
#include "RadioLib.h"

// Hardware
// Heltec Wifi LoRa 32 (V3) - SX126x pin configuration
#define PIN_LORA_RESET 12  // LORA RESET
#define PIN_LORA_DIO_1 14  // LORA DIO_1
#define PIN_LORA_BUSY  13  // LORA BUSY
#define PIN_LORA_NSS    8  // LORA CS
#define PIN_LORA_SCLK   9  // LORA SPI CLK
#define PIN_LORA_MISO  11  // LORA SPI MISO
#define PIN_LORA_MOSI  10  // LORA SPI MOSI
#define RADIO_TXEN     -1  // LORA ANTENNA TX ENABLE
#define RADIO_RXEN     -1  // LORA ANTENNA RX ENABLE

namespace esphome {
    namespace lora_radio {

        static const char *TAG = "lora_radio.component";

        // Constructor implementation
        // LoRaRadio::LoRaRadio() {
        // ESP_LOGI(TAG, "LoRaRadio object created");
        // }

        void LoRaRadio::setup() {
            ESP_LOGI(TAG, "Setting up LoRaRadio...");

            // auto nss   = id(lora_nss);
            // auto reset = id(lora_reset);
            // auto busy  = id(lora_busy);
            // auto dio1  = id(lora_dio1);

            // Initialize LoRa module
            // lora_module_ = Module(nss, dio1, reset, busy);
            // int state = lora_.begin();
            // if (state != RADIOLIB_ERR_NONE) {
            // ESP_LOGE("LoRa", "Failed to initialize LoRa! Error code: %d", state);
            // } else {
            //    ESP_LOGI("LoRa", "LoRa initialized successfully!");
            // }

            SX1262 radio = new Module(PIN_LORA_NSS,
                                      PIN_LORA_DIO_1,
                                      PIN_LORA_RESET,
                                      PIN_LORA_BUSY);

        }

        void LoRaRadio::loop() {

        }

        void LoRaRadio::dump_config(){
            ESP_LOGCONFIG(TAG, "LoRa Radio component");
        }


    }  // namespace lora_radio
}  // namespace esphome
