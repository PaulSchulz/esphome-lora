#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"

#include "RadioLib.h"

namespace esphome {
    namespace lora_radio  {
        class LoRaRadio : public Component,
        public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,spi::CLOCK_POLARITY_LOW,
                              spi::CLOCK_PHASE_LEADING,spi::DATA_RATE_1KHZ>
        {
        public:
            // LoRaRadio();
            void setup() override;
            void loop() override;
            void dump_config() override;

            void set_control_pins(//uint32_t cs,
                                  uint32_t reset,
                                  uint32_t dio1,
                                  uint32_t busy);

        protected:
            int reset_pin_;
            int dio1_pin_;
            int busy_pin_;
            SX1262 *radio_;

        private:
            // Module lora_module_;
            // SX1262 lora_;
        };

    }  // namespace lora_radio
}  // namespace esphome
