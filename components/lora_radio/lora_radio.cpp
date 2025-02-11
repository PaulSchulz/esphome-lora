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

        // flag to indicate that a packet was sent or received
        volatile bool operationDone = false;

        SX1262 radio = new Module(PIN_LORA_NSS,
                                  PIN_LORA_DIO_1,
                                  PIN_LORA_RESET,
                                  PIN_LORA_BUSY);

        void setFlag(void) {
            // we sent or received a packet, set the flag
            ESP_LOGI(TAG,"packet");
            operationDone = true;
        }

        /** hash a string into an integer
         *
         * djb2 by Dan Bernstein.
         * http://www.cse.yorku.ca/~oz/hash.html
         */
        uint32_t hash(const char *str) {
            uint32_t hash = 5381;
            int c;

            while ((c = *str++) != 0)
                hash = ((hash << 5) + hash) + (unsigned char)c; /* hash * 33 + c */

            return hash;
        }

        // Taken from mesh/RadioInterface.h
        // Slottime is the minimum time to wait, consisting of:
        // - CAD duration (maximum of SX126x and SX127x);
        // - roundtrip air propagation time (assuming max. 30km between nodes);
        // - Tx/Rx turnaround time (maximum of SX126x and SX127x);
        // - MAC processing time (measured on T-beam) */
        uint32_t computeSlotTimeMsec(float bw, float sf) {
        return 8.5 * pow(2, sf) / bw + 0.2 + 0.4 + 7;
    }

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

            // int state = radio.begin();
            // carrier frequency:           915.0 MHz
            // bandwidth:                   500.0 kHz
            // spreading factor:            6
            // coding rate:                 5
            // sync word:                   0x34 (public network/LoRaWAN)
            // output power:                2 dBm
            // preamble length:             20 symbols

            // Meshtastic LoRa parameters
            const char *region     = "ANZ";
            uint8_t modem_preset   = 0;
            float freqStart        = 915.0;
            float freqEnd          = 928.0;
            float spacing          =   0.0; // Bandwidth separation
            float bw               = 250.0; // FastLong
            float frequency_offset = 0.0;

            uint32_t numChannels
                = floor((freqEnd - freqStart) / (spacing + (bw / 1000.0)));
            const char *channelName = "LongFast"; // Default
            uint32_t    channel_num = hash(channelName) % numChannels;

            float    freq = freqStart + (bw / 2000.0) + (channel_num * (bw / 1000.0));
            //float    freq           = 919.875;
            // float    bw             = 250.0;
            uint8_t  sf             =    11;
            uint8_t  cr             =     5;
            uint8_t  syncWord       =  0x2b;
            int32_t  tx_power       =    22;
            uint32_t preambleLength =    16;

            uint32_t slotTimeMsec = computeSlotTimeMsec(bw, sf);

            // From RadioInterface.#include "lora_radio.h"
            ESP_LOGI(TAG, "Radio: freq=%.3f", freq);
            ESP_LOGI(TAG, "Radio: bw=%.1f, sf=%d, cr=%d",
                     bw, sf, cr);
            ESP_LOGI(TAG, "Radio syncWord=%#x, tx_power=%d, preambleLength=%d",
                     syncWord, tx_power, preambleLength);

            ESP_LOGI(TAG, "Radio: freq=%.3f, frequency_offset=%.3f",
                     freq, frequency_offset);
            ESP_LOGI(TAG, "Radio: region=%s, name=%s, config=%u, ch=%d, power=%d",
                     region, channelName, modem_preset,
                     channel_num, tx_power);
            ESP_LOGI(TAG, "Radio: freqStart -> freqEnd: %f -> %f (%f MHz)",
                     freqStart, freqEnd,
                     freqEnd - freqStart);
            ESP_LOGI(TAG, "Radio: numChannels: %d x %.3fkHz", numChannels, bw);
            ESP_LOGI(TAG, "Radio: channel_num: %d", channel_num + 1);
            ESP_LOGI(TAG, "Radio: frequency: %f", freq);;
            ESP_LOGI(TAG, "Radio: Slot time: %u msec", slotTimeMsec);

            // Long Fast (Meshtastic)
            int state = radio.begin(freq,
                                    bw,
                                    sf,
                                    cr,
                                    syncWord,
                                    tx_power,
                                    preambleLength);

            // Set the function that will be called
            // when new packet is received
            radio.setDio1Action(setFlag);

            ESP_LOGI(TAG,"Starting to listen ...");
            state = radio.startReceive();
        }

        void LoRaRadio::loop() {
            if (operationDone == true) {
                int state;
                operationDone = false;

                String str;
                state = radio.readData(str);
                ESP_LOGI(TAG,"Data:\t\t%#x",str);

                ESP_LOGI(TAG,"Starting to listen (again) ...");
                state = radio.startReceive();
            }
        }

        void LoRaRadio::dump_config(){
            ESP_LOGCONFIG(TAG, "LoRa Radio component");
        }


    }  // namespace lora_radio
}  // namespace esphome
