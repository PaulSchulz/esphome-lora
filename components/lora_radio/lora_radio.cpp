#include "esphome/core/log.h"
#include "lora_radio.h"

// #include "SPI.h"
#include "RadioLib.h"

// #include <base64.h>

#define MAX_LORA_PAYLOAD_LEN     255 // max length of 255 per Semtech's datasheets on SX12xx
#define MESHTASTIC_HEADER_LENGTH  16
#define MESHTASTIC_PKC_OVERHEAD   12

#define PACKET_FLAGS_HOP_LIMIT_MASK  0x07
#define PACKET_FLAGS_WANT_ACK_MASK   0x08
#define PACKET_FLAGS_VIA_MQTT_MASK   0x10
#define PACKET_FLAGS_HOP_START_MASK  0xE0
#define PACKET_FLAGS_HOP_START_SHIFT    5

#define NODENUM_BROADCAST UINT32_MAX
#define NODENUM_BROADCAST_NO_LORA 1

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

// Protobuf definitions
#define pb_size_t uint32_t
#define pb_byte_t uint8_t
#define PB_BYTES_ARRAY_T(n) struct { pb_size_t size; pb_byte_t bytes[n]; }

#define NodeNum uint32_t
#define PacketId uint32_t

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


        //////////////////////////////////////////////////////////////////////
        // Data structures from RadioLibInterface
        typedef struct {
            NodeNum to, from; // can be 1 byte or four bytes
            PacketId id; // can be 1 byte or 4 bytes
            /**
             * Usage of flags:
             *
             * The bottom three bits of flags are use to store hop_limit whe sent over the wire.
             **/
            uint8_t flags;

            /** The channel hash - used as a hint for the decoder to limit which channels we consider */
            uint8_t channel;

            // ***For future use*** Last byte of the NodeNum of the next-hop for this packet
            uint8_t next_hop;

            // ***For future use*** Last byte of the NodeNum of the node that will relay/relayed this packet
            uint8_t relay_node;
        } PacketHeader;

        /**
         * This structure represent the structured buffer : a PacketHeader then the payload. The whole is
         * MAX_LORA_PAYLOAD_LEN + 1 length
         * It makes the use of its data easier, and avoids manipulating pointers (and potential non aligned accesses)
         */
        typedef struct {
            /** The header, as defined just before */
            PacketHeader header;

            /** The payload, of maximum length minus the header, aligned just to be sure */
            uint8_t payload[MAX_LORA_PAYLOAD_LEN + 1 - sizeof(PacketHeader)] __attribute__((__aligned__));

        } RadioBuffer;

        //////////////////////////////////////////////////////////////////////
        // meshtastic_PortNum
        typedef enum _meshtastic_PortNum {
            meshtastic_PortNum_UNKNOWN_APP = 0,
            meshtastic_PortNum_TEXT_MESSAGE_APP = 1,
            meshtastic_PortNum_REMOTE_HARDWARE_APP = 2,
            meshtastic_PortNum_POSITION_APP = 3,
            meshtastic_PortNum_NODEINFO_APP = 4,
            meshtastic_PortNum_ROUTING_APP = 5,
            meshtastic_PortNum_ADMIN_APP = 6,
            meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP = 7,
            meshtastic_PortNum_WAYPOINT_APP = 8,
            meshtastic_PortNum_AUDIO_APP = 9,
            meshtastic_PortNum_DETECTION_SENSOR_APP = 10,
            meshtastic_PortNum_ALERT_APP = 11,
            meshtastic_PortNum_REPLY_APP = 32,
            meshtastic_PortNum_IP_TUNNEL_APP = 33,
            meshtastic_PortNum_PAXCOUNTER_APP = 34,
            meshtastic_PortNum_SERIAL_APP = 64,
            meshtastic_PortNum_STORE_FORWARD_APP = 65,
            meshtastic_PortNum_RANGE_TEST_APP = 66,
            meshtastic_PortNum_TELEMETRY_APP = 67,
            meshtastic_PortNum_ZPS_APP = 68,
            meshtastic_PortNum_SIMULATOR_APP = 69,
            meshtastic_PortNum_TRACEROUTE_APP = 70,
            meshtastic_PortNum_NEIGHBORINFO_APP = 71,
            meshtastic_PortNum_ATAK_PLUGIN = 72,
            meshtastic_PortNum_MAP_REPORT_APP = 73,
            meshtastic_PortNum_POWERSTRESS_APP = 74,
            meshtastic_PortNum_PRIVATE_APP = 256,
            meshtastic_PortNum_ATAK_FORWARDER = 257,
            meshtastic_PortNum_MAX = 511
        } meshtastic_PortNum;

        typedef PB_BYTES_ARRAY_T(233) meshtastic_Data_payload_t;
        // (Formerly called SubPacket) The payload portion fo a packet, this is
        //  the actual bytes that are sent inside a radio packet (because
        //  from/to are broken out by the comms library)

        // meshtastic_Data
        typedef struct _meshtastic_Data {
            // Formerly named typ and of type Type
            meshtastic_PortNum portnum;
            // TODO: REPLACE
            meshtastic_Data_payload_t payload;
            // Not normally used, but for testing a sender can request that
            // recipient responds in kind (i.e. if it received a position, it
            // should unicast back it's position). Note: that if you set this on
            // a broadcast you will receive many replies.
            bool want_response;
            // The address of the destination node. This field is is filled in
            // by the mesh radio device software, application layer software should never
            // need it. RouteDiscovery messages _must_ populate this. Other message types
            // might need to if they are doing multihop routing.
            uint32_t dest;
            // The address of the original sender for this message. This field
            // should _only_ be populated for reliable multihop packets (to keep
            // packets small).
            uint32_t source;
            // Only used in routing or response messages. Indicates the original
            // message ID that this message is reporting failure on. (formerly
            // called original_id)
            uint32_t request_id;
            // If set, this message is intened to be a reply to a previously
            // sent message with the defined id.
            uint32_t reply_id;
            // Defaults to false. If true, then what is in the payload should be
            // treated as an emoji like giving a message a heart or poop
            // emoji.
            uint32_t emoji;
            // Bitfield for extra flags. First use is to indicate that user
            // approves the packet being uploaded to MQTT.
            bool has_bitfield;
            uint8_t bitfield;
        } meshtastic_Data;

        // meshtastic_MeshPacket_encrypted_t
        // meshtastic_MeshPacket_public_key_t
        typedef PB_BYTES_ARRAY_T(256) meshtastic_MeshPacket_encrypted_t;
        typedef PB_BYTES_ARRAY_T(32) meshtastic_MeshPacket_public_key_t;

        // meshtastic_MeshPacket_Priority
        typedef enum _meshtastic_MeshPacket_Priority {
            meshtastic_MeshPacket_Priority_UNSET = 0,
            meshtastic_MeshPacket_Priority_MIN = 1,
            meshtastic_MeshPacket_Priority_BACKGROUND = 10,
            meshtastic_MeshPacket_Priority_DEFAULT = 64,
            meshtastic_MeshPacket_Priority_RELIABLE = 70,
            meshtastic_MeshPacket_Priority_RESPONSE = 80,
            meshtastic_MeshPacket_Priority_HIGH = 100,
            meshtastic_MeshPacket_Priority_ALERT = 110,
            meshtastic_MeshPacket_Priority_ACK = 120,
            meshtastic_MeshPacket_Priority_MAX = 127
        } meshtastic_MeshPacket_Priority;

        // meshtastic_MeshPacket_Delayed
        /* Identify if this is a delayed packet */
        typedef enum _meshtastic_MeshPacket_Delayed {
            meshtastic_MeshPacket_Delayed_NO_DELAY = 0,
            meshtastic_MeshPacket_Delayed_DELAYED_BROADCAST = 1,
            meshtastic_MeshPacket_Delayed_DELAYED_DIRECT = 2
        } meshtastic_MeshPacket_Delayed;

        // meshtastic_MeshPacket
        // See: mesh/generated/meshtastic/mesh.pb.h:699
        // Formatted for readability
        typedef struct _meshtastic_MeshPacket {
            // The sending node number.
            //   Note: Our crypto implementation uses this field as well.
            //   See [crypto](/docs/overview/encryption) for details.
            uint32_t from;
            // The (immediate) destination for this packet
            uint32_t to;
            // (Usually) If set, this indicates the index in the
            // secondary_channels table that this packet was sent/received on.
            // If unset, packet was on the primary channel. A particular node
            // might know only a subset of channels in use on the mesh.
            // Therefore channel_index is inherently a local concept and
            // meaningless to send between nodes. Very briefly, while sending
            // and receiving deep inside the device Router code, this field
            // instead contains the 'channel hash' instead of the index. This
            // 'trick' is only used while the payload_variant is an 'encrypted'.
            //
            uint8_t channel;
            pb_size_t which_payload_variant;
            union {
                // TODO: REPLACE
                meshtastic_Data decoded;
                // TODO: REPLACE
                meshtastic_MeshPacket_encrypted_t encrypted;
            };
            // A unique ID for this packet. Always 0 for no-ack packets or non
            // broadcast packets (and therefore take zero bytes of space).
            // Otherwise a unique ID for this packet, useful for flooding
            // algorithms. ID only needs to be unique on a _per sender_ basis,
            // and it only needs to be unique for a few minutes (long enough to
            // last for the length of any ACK or the completion of a mesh
            // broadcast flood). Note: Our crypto implementation uses this id as
            // well. See [crypto](/docs/overview/encryption) for details.
            uint32_t id;
            // The time this message was received by the esp32 (secs since
            // 1970). Note: this field is _never_ sent on the radio link itself
            // (to save space) Times are typically not sent over the mesh, but
            // they will be added to any Packet (chain of SubPacket) sent to the
            // phone (so the phone can know exact time of reception)
            uint32_t rx_time;
            // *Never* sent over the radio links.
            // Set during reception to indicate the SNR of this packet.
            // Used to collect statistics on current link quality.
            float rx_snr;
            // If unset treated as zero (no forwarding, send to direct neighbor
            // nodes only) if 1, allow hopping through one node, etc... For our
            // usecase real world topologies probably have a max of about 3.
            // This field is normally placed into a few of bits in the header.
            uint8_t hop_limit;
            // This packet is being sent as a reliable message, we would prefer
            // it to arrive at the destination. We would like to receive a ack
            // packet in response. Broadcasts messages treat this flag
            // specially: Since acks for broadcasts would rapidly flood the
            // channel, the normal ack behavior is suppressed. Instead, the
            // original sender listens to see if at least one node is
            // rebroadcasting this packet (because naive flooding algorithm). If
            // it hears that the odds (given typical LoRa topologies) the odds
            // are very high that every node should eventually receive the
            // message. So FloodingRouter.cpp generates an implicit ack which is
            // delivered to the original sender. If after some time we don't
            // hear anyone rebroadcast our packet, we will timeout and
            // retransmit, using the regular resend logic. Note: This flag is
            // normally sent in a flag bit in the header when sent over the
            // wire.
            bool want_ack;
            // The priority of this message for sending. See MeshPacket.Priority
            // description for more details.
            meshtastic_MeshPacket_Priority priority;
            //  rssi of received packet. Only sent to phone for dispay purposes.
            int32_t rx_rssi;
            // Describe if this message is delayed.
            meshtastic_MeshPacket_Delayed delayed;
            // Describes whether this packet passed via MQTT somewhere along the
            // path it currently took.
            bool via_mqtt;
            // Hop limit with which the original packet started. Sent via LoRa
            // using three bits in the unencrypted header. When receiving a
            // packet, the difference between hop_start and hop_limit gives how
            // many hops it traveled.
            uint8_t hop_start;
            // Records the public key the packet was encrypted with, if
            // applicable.
            meshtastic_MeshPacket_public_key_t public_key;
            // Indicates whether the packet was en/decrypted using PKI
            bool pki_encrypted;
            // Last byte of the node number of the node that should be used as
            // the next hop in routing. Set by the firmware internally,
            // clients are not supposed to set this. */
            uint8_t next_hop;
            // Last byte of the node number of the node that will relay/relayed
            // this packet. Set by the firmware internally, clients are not
            // supposed to set this.
            uint8_t relay_node;
            // *Never* sent over the radio links. Timestamp after which this
            // packet may be sent. Set by the firmware internally, clients are
            // not supposed to set this.
            uint32_t tx_after;
        } meshtastic_MeshPacket;

        //////////////////////////////////////////////////////////////////////
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


        void printPacket(const char *prefix, const meshtastic_MeshPacket *p) {
            std::string out = "";

            ESP_LOGI(TAG, "%s (id=0x%08x fr=0x%08x to=0x%08x, WantAck=%d, HopLim=%d Ch=0x%x",
                     prefix, p->id,
                     p->from, p->to,
                     p->want_ack, p->hop_limit, p->channel);

            /*
              if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
                auto &s = p->decoded;

                out += DEBUG_PORT.mt_sprintf(" Portnum=%d", s.portnum);

                if (s.want_response)
                    out += DEBUG_PORT.mt_sprintf(" WANTRESP");

                if (p->pki_encrypted)
                    out += DEBUG_PORT.mt_sprintf(" PKI");

                if (s.source != 0)
                    out += DEBUG_PORT.mt_sprintf(" source=%08x", s.source);

                if (s.dest != 0)
                    out += DEBUG_PORT.mt_sprintf(" dest=%08x", s.dest);

                if (s.request_id)
                    out += DEBUG_PORT.mt_sprintf(" requestId=%0x", s.request_id);
             */

            /* now inside Data and therefore kinda opaque
               if (s.which_ackVariant == SubPacket_success_id_tag)
            out += DEBUG_PORT.mt_sprintf(" successId=%08x", s.ackVariant.success_id);
        else if (s.which_ackVariant == SubPacket_fail_id_tag)
            out += DEBUG_PORT.mt_sprintf(" failId=%08x", s.ackVariant.fail_id); */

            //} else {
            //    out += " encrypted";
            //out += DEBUG_PORT.mt_sprintf(" len=%d", p->encrypted.size + sizeof(PacketHeader));

            /*
              if (p->rx_time != 0)
                out += DEBUG_PORT.mt_sprintf(" rxtime=%u", p->rx_time);
            if (p->rx_snr != 0.0)
                out += DEBUG_PORT.mt_sprintf(" rxSNR=%g", p->rx_snr);
            if (p->rx_rssi != 0)
                out += DEBUG_PORT.mt_sprintf(" rxRSSI=%i", p->rx_rssi);
            if (p->via_mqtt != 0)
                out += DEBUG_PORT.mt_sprintf(" via MQTT");
            if (p->hop_start != 0)
                out += DEBUG_PORT.mt_sprintf(" hopStart=%d", p->hop_start);
            if (p->priority != 0)
                out += DEBUG_PORT.mt_sprintf(" priority=%d", p->priority);

            out += ")";
             */
            ESP_LOGI(TAG, "%s", out.c_str());
        }

        //////////////////////////////////////////////////////////////////////
        void setFlag(void) {
            // we sent or received a packet, set the flag
            ESP_LOGI(TAG,"packet");

            operationDone = true;
        }

        //////////////////////////////////////////////////////////////////////
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
            float frequency_offset =   0.0;

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
            ESP_LOGI(TAG, "Radio: syncWord=%#x, tx_power=%d, preambleLength=%d",
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
            RadioBuffer radioBuffer __attribute__((__aligned__));
            meshtastic_MeshPacket mp;

            if (operationDone == true) {
                int state;
                operationDone = false;

                // String str;
                // state = radio.readData(str);
                // ESP_LOGI(TAG,"Data:\t\t%#x",str);

                // state = radio.readData(mp);
                // printPacket("PKT", mp);

                // byte byteArr[256];
                int numBytes = radio.getPacketLength();
                state = radio.readData((uint8_t *)&radioBuffer, numBytes);
                ESP_LOGI(TAG, "Size: %d", numBytes);

                mp.from    = radioBuffer.header.from;
                mp.to      = radioBuffer.header.to;
                mp.id      = radioBuffer.header.id;
                mp.channel = radioBuffer.header.channel;
                mp.hop_limit =  radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
                mp.hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK)
                               >> PACKET_FLAGS_HOP_START_SHIFT;
                mp.want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
                mp.via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);

                ESP_LOGD(TAG, "size=%d",      numBytes);
                ESP_LOGD(TAG, "from=%#x",     mp.from);
                ESP_LOGD(TAG, "to=%#x",       mp.to);
                ESP_LOGD(TAG, "id=%#x",       mp.id);
                ESP_LOGD(TAG, "channel=%#x",  mp.channel);
                ESP_LOGD(TAG, "hop_limit=%d", mp.hop_limit);
                ESP_LOGD(TAG, "hop_start=%d", mp.hop_start);
                ESP_LOGD(TAG, "want_ack=%d",  mp.want_ack);
                ESP_LOGD(TAG, "via_mqtt=%d",  mp.via_mqtt);


                // pki_key="AQ==";
                uint8_t key[32] = {
                    0x38, 0x4b, 0xbc, 0xc0, 0x1d, 0xc0, 0x22, 0xd1,
                    0x81, 0xbf, 0x36, 0xb8, 0x61, 0x21, 0xe1, 0xfb,
                    0x96, 0xb7, 0x2e, 0x55, 0xbf, 0x74, 0x22, 0x7e,
                    0x9d, 0x6a, 0xfb, 0x48, 0xd6, 0x4c, 0xb1, 0xa1
                };

                // uint8_t other_public_key[32] = 0x00;
                // uint8_t my_public_key[32] 0x00;

                // Calculate the length of the encoded Base64 string
                // int encodedLength = base64_enc_len(sizeof(key));
                // char encoded[encodedLength];

                std::string encoded;
                // Encode the key
                // base64_encode(encoded, key, sizeof(key));
                encoded = base64_encode(key, sizeof(key));

                // Try an decrypt
                if (mp.channel == 0
                    && mp.to == 0xfa80683c
                        // && nodeDB->getMeshNode(p->from) != nullptr
                        // && nodeDB->getMeshNode(p->from)->user.public_key.size > 0
                        // && nodeDB->getMeshNode(p->to)->user.public_key.size > 0 &&
                    )
                    ESP_LOGD(TAG, "*** Packet to me! ***");

                // printPacket("PKT", (meshtastic_MeshPacket)byteArr);

                ESP_LOGI(TAG,"Starting to listen (again) ...");
                state = radio.startReceive();
            }
    }

    void LoRaRadio::dump_config(){
        ESP_LOGCONFIG(TAG, "LoRa Radio component");
    }


    }  // namespace lora_radio
}  // namespace esphome
