#include "esphome/core/log.h"
#include "lora_radio.h"

namespace esphome {
    namespace lora_radio {

        static const char *TAG = "lora_radio.component";

        void LoRaRadio::setup() {

        }

        void LoRaRadio::loop() {

        }

        void LoRaRadio::dump_config(){
            ESP_LOGCONFIG(TAG, "LoRa Radio component");
        }


    }  // namespace lora_radio
}  // namespace esphome
