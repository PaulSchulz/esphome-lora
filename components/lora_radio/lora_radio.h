#pragma once
#include "esphome.h"
#include "RadioLib.h"

#include "lora_radio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace lora_radio {

    class LoraRadio : public Component {
    public:
        // Constructor
        LoraRadio();

        void setup() override;
        void loop() override;

    private:
        std::string board_type_;
        Module lora_module_;
        SX1262 lora_;

    // Pin configurations for both boards
    //int nss_pin_, reset_pin_, busy_pin_, dio_1_pin_;
};

}  // namespace lora_radio
}  // namespace esphome
