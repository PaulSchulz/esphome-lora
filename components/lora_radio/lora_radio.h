#pragma once
#include "esphome.h"
#include "RadioLib.h"

class LoRaRadio : public Component {
 public:
    LoRaRadio(GPIOPin *nss, GPIOPin *reset, GPIOPin *busy, GPIOPin *dio1);
    void setup() override;
    void loop() override;

 private:
    Module lora_module_;
    SX1262 lora_;
};
