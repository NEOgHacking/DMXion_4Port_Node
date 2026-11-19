#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ST7789.h>
#include <Charger.h>
#include <DmxOutput.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>
#include <atomic>
#include <cstring>
#include <pico/critical_section.h>

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
// Rotary encoder: quadrature A/B inputs and the push switch.
constexpr uint8_t ENCODER_A = 8;
constexpr uint8_t ENCODER_B = 9;
constexpr uint8_t ENCODER_SWITCH = 7;

// Ethernet controller W5500 pins and the two status input pins.
constexpr uint8_t ETH_MOSI = 31;
constexpr uint8_t ETH_MISO = 28;
constexpr uint8_t ETH_SCK = 30;
constexpr uint8_t ETH_CS = 29;
constexpr uint8_t LINKLED = 25;
constexpr uint8_t ACTLED = 26;

// NeoPixel status strip.
constexpr uint8_t LED_PIN = 21;
constexpr uint8_t LED_COUNT = 10;

// Four DMX transmitter data and output-enable pins, in output 1-4 order.
constexpr uint8_t UNIVERSETX[4] = {39, 35, 43, 47};
constexpr uint8_t UNIVERSEOE[4] = {38, 34, 42, 46};

// ST7789 display pins.
constexpr uint8_t TFT_CS = 5;
constexpr uint8_t TFT_MOSI = 3;
constexpr uint8_t TFT_SCK = 2;
constexpr uint8_t TFT_RST = 1;
constexpr uint8_t TFT_DC = 6;
constexpr uint8_t TFT_BACKLIGHT = 0;

// -----------------------------------------------------------------------------
// Network and DMX configuration
// -----------------------------------------------------------------------------
constexpr uint8_t DMX_SPEED = 30;                 // DMX transmitter rate in Hz.
constexpr uint16_t UNIVERSE_LENGTH = 512;        // Number of channels in each universe.
constexpr uint16_t MAX_ARTNET_UNIVERSE = 0x7FFF; // Highest supported Art-Net universe.
constexpr uint16_t ARTNET_PORT = 6454;           // Standard Art-Net UDP port.
constexpr size_t ARTNET_PACKET_MAX_SIZE = 530;   // Largest supported Art-Net packet.
constexpr uint16_t SACN_PORT = 5568;
constexpr size_t SACN_PACKET_MAX_SIZE = 638;     // Largest supported sACN packet.

// -----------------------------------------------------------------------------
// Persistent settings and defaults
// -----------------------------------------------------------------------------
constexpr uint32_t CONFIG_MAGIC = 0x444D5831;    // Identifies a valid DMXion config block.
constexpr uint32_t BACKLIGHT_TIMEOUT_MS = 60000; // Default LCD inactivity timeout.
constexpr uint32_t ENCODER_HOLD_MS = 650;        // Time needed for a hold action.
constexpr uint32_t ENCODER_FAST_ROTATION_MS = 100; // Gap below which rotary acceleration starts.
constexpr uint32_t ENCODER_FAST_ROTATION_TIMEOUT_MS = 100;
constexpr uint8_t DEFAULT_UNIVERSE_TIMEOUT_SECONDS = 10;
constexpr uint8_t DEFAULT_LED_BRIGHTNESS_PERCENT = 39;
constexpr uint8_t DEFAULT_LCD_BRIGHTNESS_PERCENT = 100;
constexpr uint8_t DEFAULT_LCD_TIMEOUT_SECONDS = BACKLIGHT_TIMEOUT_MS / 1000;
constexpr uint8_t DEFAULT_DISABLED_OUTPUT_BRIGHTNESS_PERCENT = 10;
constexpr uint8_t SETTINGS_VERSION = 3;          // Increases when settings layout changes.
constexpr uint8_t SETTINGS_MAIN_OUTPUT = 5;      // Virtual menu item in the main UI.
constexpr uint8_t DMX_LOCK_BYTES = (UNIVERSE_LENGTH + 7) / 8;

// -----------------------------------------------------------------------------
// Runtime objects and shared state
// -----------------------------------------------------------------------------
// The RP2040/RP2350 runs the network task and UI task concurrently. A Pico critical
// section protects values shared between the two cores and the repeating timers.
unsigned long last_dmx_packet[4] = {0, 0, 0, 0};
std::atomic<bool> keep_sending[4] = {true, true, true, true};
std::atomic<bool> network_ready{false};
std::atomic<bool> network_pause_requested{false};
std::atomic<bool> network_paused{false};
std::atomic<bool> network_reconfigure_requested{false};
std::atomic<bool> network_reconfiguration_pending{false};
std::atomic<bool> sacn_reconfigure_requested{false};
std::atomic<bool> dhcp_enabled{false};
std::atomic<bool> dhcp_lease_acquired{false};
std::atomic<bool> link_led_on{false};
std::atomic<bool> act_led_on{false};
std::atomic<bool> ethernet_link_up{false};
std::atomic<bool> battery_measurement_requested{true};
std::atomic<int32_t> encoder_transition_count{0};
uint8_t encoder_previous_ab = 0;
constexpr int8_t ENCODER_TRANSITIONS[16] = {
    0, 1, -1, 0, -1, 0, 0, 1,
    1, 0, 0, -1, 0, -1, 1, 0
};

repeating_timer_t interval_timer;
repeating_timer_t charger_timer;
repeating_timer_t startup_rainbow_timer;

// pending_universedata contains the start-code byte plus all 512 DMX channels.
// output_universedata is the copy actually written to the PIO-based DMX output.
uint8_t pending_universedata[4][UNIVERSE_LENGTH + 1] = {};
uint8_t output_universedata[4][UNIVERSE_LENGTH + 1] = {};
uint32_t activity_deadline[4] = {};
uint32_t received_packets[4] = {};
bool activity_active[4] = {};
bool received_any[4] = {};
uint16_t connector_universe[4] = {0, 1, 2, 3};
uint8_t enabled_outputs = 0x0F;
IPAddress configured_ip(192, 168, 2, 212);
critical_section_t frame_lock;

// EEPROM layout. Keep this structure compact and bump SETTINGS_VERSION when
// persisted setting fields or meanings change.
struct StoredConfiguration {
    uint32_t magic;
    uint16_t universes[4];
    uint8_t enabled_mask;
    uint8_t ip_address[4];
    uint8_t dhcp_enabled;
    uint8_t settings_version;
    uint8_t universe_timeout_seconds;
    uint8_t led_brightness_percent;
    uint8_t lcd_brightness_percent;
    uint8_t lcd_timeout_seconds;
    uint8_t led_color_indices[5];
    uint8_t charging_led;
    uint8_t disabled_output_dim_percent;
    uint8_t dmx_address_lock_bits[4][DMX_LOCK_BYTES];
    uint8_t dmx_address_value[4][UNIVERSE_LENGTH];
};

// The main UI and settings views share one screen state machine.
enum class InterfaceMode : uint8_t {
    SelectOutput,
    EditUniverse,
    EditIp,
    SettingsMenu,
    EditSetting,
    SourcesMenu,
    DmxOverrideMenu,
    DmxViewMenu
};

InterfaceMode interface_mode = InterfaceMode::SelectOutput;
uint8_t selected_output = 0;
uint16_t edited_universe = 0;
uint8_t edited_ip[4] = {};
uint8_t edited_ip_octet = 0;
uint8_t universe_timeout_seconds = DEFAULT_UNIVERSE_TIMEOUT_SECONDS;
uint8_t led_brightness_percent = DEFAULT_LED_BRIGHTNESS_PERCENT;
uint8_t lcd_brightness_percent = DEFAULT_LCD_BRIGHTNESS_PERCENT;
uint8_t lcd_timeout_seconds = DEFAULT_LCD_TIMEOUT_SECONDS;
uint8_t led_color_indices[5] = {0, 3, 5, 7, 7};
uint8_t charging_led = 0;
uint8_t disabled_output_dim_percent =
    DEFAULT_DISABLED_OUTPUT_BRIGHTNESS_PERCENT;
// One lock bit and one fixed value are kept for every channel of every universe.
uint8_t dmx_address_locked[4][UNIVERSE_LENGTH] = {};
uint8_t dmx_address_value[4][UNIVERSE_LENGTH] = {};
uint8_t dmx_override_universe = 0;
uint16_t dmx_override_address = 0;
uint8_t dmx_override_menu_index = 0;
bool dmx_override_editing = false;
uint16_t dmx_override_view_scroll = 0;
uint8_t settings_category = 0;
uint8_t settings_menu_index = 0;
uint8_t settings_menu_scroll = 0;
uint8_t edited_setting_index = 0;
uint8_t original_setting_value = 0;
bool settings_root_menu = true;
bool edit_ip_returns_to_settings = false;
bool configuration_saved = true;
uint32_t last_user_activity = 0;
uint32_t last_interface_stats_update = 0;
bool backlight_is_on = true;
bool battery_voltage_valid = false;
float battery_voltage = 0.0f;
uint8_t battery_percentage = 0;
bool battery_charge_status_valid = false;
bool battery_is_charging = false;
bool startup_rainbow_running = false;
uint32_t startup_rainbow_last_frame = 0;
uint8_t startup_rainbow_frame = 0;

DmxOutput universes[4];

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

EthernetUDP artnet_receiver;
EthernetUDP sacn_receivers[4];
uint16_t sacn_receiver_universes[4] = {};
bool sacn_receiver_active[4] = {};

constexpr uint8_t SOURCE_ENTRY_COUNT = 12;
struct SourceEntry {
    uint32_t ip;
    uint16_t universe;
    uint32_t last_seen;
    uint8_t protocol;
};
SourceEntry source_entries[SOURCE_ENTRY_COUNT] = {};
uint8_t source_menu_index = 0;
uint8_t source_menu_scroll = 0;
SourceEntry active_source_entries[SOURCE_ENTRY_COUNT] = {};
uint8_t active_source_count = 0;

void recordSource(uint8_t protocol, const IPAddress &source_ip,
                  uint16_t universe) {
    const uint32_t ip_value = source_ip;
    if(ip_value == 0) {
        return;
    }
    const uint32_t now = millis();
    critical_section_enter_blocking(&frame_lock);

    int8_t free_index = -1;
    int8_t matching_index = -1;
    for(uint8_t i = 0; i < SOURCE_ENTRY_COUNT; i++) {
        if(source_entries[i].ip == ip_value &&
           source_entries[i].protocol == protocol &&
           source_entries[i].universe == universe) {
            matching_index = i;
            break;
        }
        if(free_index < 0 && source_entries[i].ip == 0) {
            free_index = i;
        }
    }

    if(matching_index < 0) {
        matching_index = free_index;
        if(matching_index < 0) {
            matching_index = 0;
            uint32_t oldest_age = 0;
            for(uint8_t i = 0; i < SOURCE_ENTRY_COUNT; i++) {
                const uint32_t age = now - source_entries[i].last_seen;
                if(age >= oldest_age) {
                    oldest_age = age;
                    matching_index = i;
                }
            }
        }
    }

    source_entries[matching_index] = {
        ip_value, universe, now, protocol
    };
    critical_section_exit(&frame_lock);
}

void refreshActiveSources() {
    active_source_count = 0;
    const uint32_t now = millis();
    critical_section_enter_blocking(&frame_lock);
    for(uint8_t i = 0; i < SOURCE_ENTRY_COUNT; i++) {
        const SourceEntry &entry = source_entries[i];
        if(entry.ip != 0 &&
           static_cast<uint32_t>(now - entry.last_seen) <= 30000) {
            active_source_entries[active_source_count++] = entry;
        }
    }
    critical_section_exit(&frame_lock);
}

byte mac[] = { 0x02, 0x44, 0x4D, 0x58, 0x00, 0x01 };
IPAddress ip(192, 168, 2, 212);  // Set to an unused address on your Art-Net network

Adafruit_ST7789 tftHardware = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 tftCanvas(320, 240);
Adafruit_GFX &tft = tftCanvas;

void drawFrameToDisplay() {
    tftHardware.startWrite();
    tftHardware.drawRGBBitmap(0, 0, tftCanvas.getBuffer(), 320, 240);
    tftHardware.endWrite();
}

void drawSettingsButton() {
    const bool selected = selected_output == SETTINGS_MAIN_OUTPUT;
    tft.fillRoundRect(250, 28, 65, 20, 3,
                      selected ? ST77XX_YELLOW : 0x2104);
    tft.drawRoundRect(250, 28, 65, 20, 3,
                      selected ? ST77XX_YELLOW : 0x7BEF);
    tft.setTextSize(1);
    tft.setTextColor(selected ? ST77XX_BLACK : ST77XX_WHITE);
    tft.setCursor(258, 35);
    tft.print(F("SETTINGS"));
}

// Status LED palette used by the menu and NeoPixel status roles.
struct StatusColor {
    const char *name;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

constexpr StatusColor STATUS_COLORS[] = {
    {"RED", 255, 0, 0},
    {"ORANGE", 255, 96, 0},
    {"YELLOW", 255, 180, 0},
    {"GREEN", 0, 255, 0},
    {"CYAN", 0, 180, 255},
    {"BLUE", 0, 0, 255},
    {"MAGENTA", 255, 0, 180},
    {"WHITE", 255, 255, 255}
};
constexpr uint8_t STATUS_COLOR_COUNT =
    sizeof(STATUS_COLORS) / sizeof(STATUS_COLORS[0]);

void applyLedBrightness() {
    strip.setBrightness(static_cast<uint8_t>(
        (static_cast<uint16_t>(led_brightness_percent) * 255 + 50) / 100));
}

void applyLcdBrightness() {
    const uint8_t pwm_value = backlight_is_on ?
        static_cast<uint8_t>((static_cast<uint16_t>(lcd_brightness_percent) * 255 + 50) / 100) : 0;
    analogWrite(TFT_BACKLIGHT, pwm_value);
}

void setStatusColor(uint8_t pixel, uint8_t role) {
    const StatusColor &color = STATUS_COLORS[led_color_indices[role]];
    strip.setPixelColor(pixel, color.red, color.green, color.blue);
}

void setStatusColorDimmed(uint8_t pixel, uint8_t role) {
    const StatusColor &color = STATUS_COLORS[led_color_indices[role]];
    const uint8_t red = static_cast<uint8_t>((static_cast<uint16_t>(color.red) *
                                              disabled_output_dim_percent + 50) / 100);
    const uint8_t green = static_cast<uint8_t>((static_cast<uint16_t>(color.green) *
                                                disabled_output_dim_percent + 50) / 100);
    const uint8_t blue = static_cast<uint8_t>((static_cast<uint16_t>(color.blue) *
                                               disabled_output_dim_percent + 50) / 100);
    strip.setPixelColor(pixel, red, green, blue);
}

// Keeps transmitting enabled universes at DMX_SPEED, even after network traffic
// stops. Locked channels replace the corresponding received channel value.
bool sendDMX(__unused repeating_timer *rt) {
    for(int i = 0; i < 4; i++) {
        if(keep_sending[i].load(std::memory_order_relaxed) && !universes[i].busy()) {
            critical_section_enter_blocking(&frame_lock);
            const bool enabled = (enabled_outputs & (1u << i)) != 0;
            if(enabled) {
                std::memcpy(output_universedata[i], pending_universedata[i], sizeof(output_universedata[i]));
                for(uint16_t address = 0; address < UNIVERSE_LENGTH; address++) {
                    if(dmx_address_locked[i][address]) {
                        output_universedata[i][address + 1] = dmx_address_value[i][address];
                    }
                }
            }
            critical_section_exit(&frame_lock);
            if(enabled) {
                universes[i].write(output_universedata[i], sizeof(output_universedata[i]));
            }
        }
    }
    return true;
}

bool sendChargerData(__unused repeating_timer *rt) {
    battery_measurement_requested.store(true, std::memory_order_relaxed);
    return true;
}

uint8_t estimateBatteryPercentage(float voltage) {
    constexpr float voltage_points[] = {3.3f, 3.5f, 3.6f, 3.7f, 3.8f, 3.9f, 4.0f, 4.1f, 4.2f};
    constexpr uint8_t percentage_points[] = {0, 10, 20, 40, 60, 75, 85, 95, 100};
    constexpr size_t point_count = sizeof(voltage_points) / sizeof(voltage_points[0]);

    if(voltage <= voltage_points[0]) {
        return percentage_points[0];
    }
    if(voltage >= voltage_points[point_count - 1]) {
        return percentage_points[point_count - 1];
    }

    for(size_t i = 1; i < point_count; i++) {
        if(voltage <= voltage_points[i]) {
            const float fraction = (voltage - voltage_points[i - 1]) /
                                   (voltage_points[i] - voltage_points[i - 1]);
            const float percentage = percentage_points[i - 1] +
                fraction * (percentage_points[i] - percentage_points[i - 1]);
            return static_cast<uint8_t>(percentage + 0.5f);
        }
    }
    return 100;
}

void updateBatteryMeasurement() {
    if(!battery_measurement_requested.exchange(false, std::memory_order_relaxed)) {
        return;
    }

    float measured_voltage = 0.0f;
    battery_voltage_valid = getVbat(measured_voltage);
    if(battery_voltage_valid) {
        battery_voltage = measured_voltage;
        battery_percentage = estimateBatteryPercentage(battery_voltage);
    }

    uint8_t charge_status = 0;
    battery_charge_status_valid = getChargeStatus(charge_status);
    if(battery_charge_status_valid) {
        constexpr uint8_t charging_status_mask = (1u << 2) | (1u << 3) | (1u << 4);
        battery_is_charging = (charge_status & charging_status_mask) != 0;
    }
}

// Copies received data into the matching output buffers and records packet timing
// for status LEDs and timeout handling.
void applyDmxData(uint16_t received_universe, const uint8_t *data, uint16_t size) {
    const uint16_t data_size = size < UNIVERSE_LENGTH ? size : UNIVERSE_LENGTH;
    const uint32_t received_at = millis();

    for(int i = 0; i < 4; i++) {
        critical_section_enter_blocking(&frame_lock);
        if(received_universe == connector_universe[i]) {
            pending_universedata[i][0] = 0;
            std::memset(&pending_universedata[i][1], 0, UNIVERSE_LENGTH);
            std::memcpy(&pending_universedata[i][1], data, data_size);
            last_dmx_packet[i] = received_at;
            received_packets[i]++;
            received_any[i] = true;
            activity_deadline[i] = received_at + 20;
            activity_active[i] = true;
        }
        critical_section_exit(&frame_lock);
    }
}

uint16_t readBigEndian16(const uint8_t *data) {
    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

void beginArtNetReceiver() {
    if(!artnet_receiver.begin(ARTNET_PORT)) {
        Serial.println(F("Failed to bind Art-Net UDP port 6454"));
    }
}

void receiveArtNetPackets() {
    uint8_t packet[ARTNET_PACKET_MAX_SIZE];
    for(uint8_t processed = 0; processed < 4; processed++) {
        const int packet_size = artnet_receiver.parsePacket();
        if(packet_size <= 0) {
            return;
        }
        if(packet_size > static_cast<int>(sizeof(packet))) {
            artnet_receiver.read(static_cast<uint8_t *>(nullptr), packet_size);
            continue;
        }

        const int bytes_read = artnet_receiver.read(packet, packet_size);
        if(bytes_read != packet_size ||
           packet_size < 18 ||
           std::memcmp(packet, "Art-Net\0", 8) != 0 ||
           packet[8] != 0x00 || packet[9] != 0x50 ||
           readBigEndian16(packet + 10) < 14) {
            continue;
        }

        const uint16_t data_size = readBigEndian16(packet + 16);
        if(data_size == 0 || data_size > UNIVERSE_LENGTH ||
           (data_size & 1) != 0 ||
           packet_size < 18 + data_size) {
            continue;
        }

        const uint16_t received_universe =
            (static_cast<uint16_t>(packet[15] & 0x7F) << 8) | packet[14];
        recordSource(0, artnet_receiver.remoteIP(), received_universe);
        applyDmxData(received_universe, packet + 18, data_size);
    }
}

uint32_t readBigEndian32(const uint8_t *data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           data[3];
}

bool validSacnPduLength(const uint8_t *packet, size_t packet_size, size_t offset) {
    if(packet_size < offset + 2) {
        return false;
    }
    const uint16_t pdu_length = readBigEndian16(packet + offset);
    return (pdu_length & 0xF000) == 0x7000 &&
           (pdu_length & 0x0FFF) == packet_size - offset;
}

void onSacnPacket(const uint8_t *packet, size_t packet_size,
                  uint16_t subscribed_universe,
                  const IPAddress &source_ip) {
    constexpr size_t root_vector_offset = 18;
    constexpr size_t framing_vector_offset = 40;
    constexpr size_t universe_candidates[][2] = {{112, 113}, {113, 114}};
    constexpr size_t start_code_offset = 125;

    if(packet_size < start_code_offset + 1 ||
       packet[0] != 0x00 || packet[1] != 0x10 ||
       packet[2] != 0x00 || packet[3] != 0x00 ||
       std::memcmp(packet + 4, "ASC-E1.17\0\0\0", 12) != 0 ||
       readBigEndian32(packet + root_vector_offset) != 0x00000004 ||
       readBigEndian32(packet + framing_vector_offset) != 0x00000002 ||
       packet[start_code_offset] != 0) {
        return;
    }

    uint16_t sacn_universe = 0;
    for(uint8_t i = 0; i < 2; i++) {
        const uint16_t candidate =
            (static_cast<uint16_t>(packet[universe_candidates[i][0]]) << 8) |
            packet[universe_candidates[i][1]];
        if(candidate == subscribed_universe) {
            sacn_universe = candidate;
            break;
        }
    }
    if(sacn_universe == 0 || sacn_universe > 63999) {
        return;
    }

    const uint16_t property_value_count = readBigEndian16(packet + 123);
    if(property_value_count == 0 ||
       property_value_count > UNIVERSE_LENGTH + 1 ||
       packet_size != start_code_offset + property_value_count) {
        return;
    }

    // DMP address/type and vector checks avoid accepting unrelated ACN traffic.
    if(packet_size >= 119 &&
       (packet[117] != 0x02 || packet[118] != 0xA1 ||
        readBigEndian16(packet + 119) != 0 ||
        readBigEndian16(packet + 121) != 1)) {
        return;
    }

    // The device's displayed universe is Art-Net's zero-based universe number.
    recordSource(1, source_ip, sacn_universe - 1);
    applyDmxData(sacn_universe - 1, packet + start_code_offset + 1,
                 property_value_count - 1);
}

void stopSacnReceivers() {
    for(uint8_t i = 0; i < 4; i++) {
        sacn_receivers[i].stop();
        sacn_receiver_active[i] = false;
    }
}

void beginSacnReceivers() {
    stopSacnReceivers();
    for(uint8_t output = 0; output < 4; output++) {
        uint16_t artnet_universe;
        critical_section_enter_blocking(&frame_lock);
        artnet_universe = connector_universe[output];
        critical_section_exit(&frame_lock);
        const uint16_t sacn_universe = artnet_universe + 1;

        bool already_subscribed = false;
        for(uint8_t receiver = 0; receiver < output; receiver++) {
            if(sacn_receiver_active[receiver] &&
               sacn_receiver_universes[receiver] == sacn_universe) {
                already_subscribed = true;
                break;
            }
        }
        if(already_subscribed) {
            continue;
        }

        const IPAddress multicast_group(239, 255,
                                        static_cast<uint8_t>(sacn_universe >> 8),
                                        static_cast<uint8_t>(sacn_universe));
        if(sacn_receivers[output].beginMulticast(multicast_group, SACN_PORT)) {
            sacn_receiver_universes[output] = sacn_universe;
            sacn_receiver_active[output] = true;
        } else {
            Serial.print(F("Failed to join sACN universe "));
            Serial.println(sacn_universe);
        }
    }
}

void receiveSacnPackets() {
    uint8_t packet[SACN_PACKET_MAX_SIZE];
    for(uint8_t i = 0; i < 4; i++) {
        if(!sacn_receiver_active[i]) {
            continue;
        }

        const int packet_size = sacn_receivers[i].parsePacket();
        if(packet_size <= 0) {
            continue;
        }
        const size_t bytes_to_read = packet_size < static_cast<int>(sizeof(packet)) ?
                                     packet_size : sizeof(packet);
        const int bytes_read = sacn_receivers[i].read(packet, bytes_to_read);
        if(bytes_read > 0) {
            onSacnPacket(packet, static_cast<size_t>(bytes_read),
                         sacn_receiver_universes[i],
                         sacn_receivers[i].remoteIP());
        }
    }
}

void onLED(uint gpio, uint32_t event_mask) {
    if(gpio == ENCODER_A || gpio == ENCODER_B) {
        const uint32_t gpio_state = gpio_get_all();
        const uint8_t current_ab = static_cast<uint8_t>(
            (((gpio_state >> ENCODER_A) & 1u) << 1) |
            ((gpio_state >> ENCODER_B) & 1u));
        const uint8_t transition =
            static_cast<uint8_t>((encoder_previous_ab << 2) | current_ab);
        encoder_transition_count.fetch_add(ENCODER_TRANSITIONS[transition],
                                           std::memory_order_relaxed);
        encoder_previous_ab = current_ab;
    } else if(gpio == LINKLED) {
        link_led_on.store(event_mask == GPIO_IRQ_EDGE_FALL, std::memory_order_relaxed);
    } else if(gpio == ACTLED) {
        act_led_on.store(event_mask == GPIO_IRQ_EDGE_FALL, std::memory_order_relaxed);
    }
}

// Updates NeoPixel status colors and the LINK/ACT indicators. Red status colors
// indicate no recent packet data; disabled output LEDs are turned completely off.
void updateStatusLeds() {
    const uint32_t now = millis();
    uint32_t packet_times[4];
    bool packet_activity[4];
    bool has_received[4];
    bool output_enabled[4];

    critical_section_enter_blocking(&frame_lock);
    for(int i = 0; i < 4; i++) {
        packet_times[i] = last_dmx_packet[i];
        has_received[i] = received_any[i];
        output_enabled[i] = (enabled_outputs & (1u << i)) != 0;
        if(activity_active[i] && static_cast<int32_t>(now - activity_deadline[i]) >= 0) {
            activity_active[i] = false;
        }
        packet_activity[i] = activity_active[i];
    }
    critical_section_exit(&frame_lock);

    for(int i = 0; i < 4; i++) {
        const bool has_recent_packet = has_received[i] &&
            static_cast<uint32_t>(now - packet_times[i]) <=
                static_cast<uint32_t>(universe_timeout_seconds) * 1000;
        keep_sending[i].store(output_enabled[i], std::memory_order_relaxed);
        if(output_enabled[i]) {
            setStatusColor(7 - 2 * i, has_recent_packet ? 1 : 0);
        } else {
            strip.setPixelColor(7 - 2 * i, 0);
        }

        if(packet_activity[i]) {
            if(output_enabled[i]) {
                setStatusColor(6 - 2 * i, 2);
            } else {
                setStatusColorDimmed(6 - 2 * i, 2);
            }
        } else {
            strip.setPixelColor(6 - 2 * i, 0);
        }
    }
    link_led_on.store(gpio_get(LINKLED) == 0, std::memory_order_relaxed);
    act_led_on.store(gpio_get(ACTLED) == 0, std::memory_order_relaxed);
    const bool link_on = link_led_on.load(std::memory_order_relaxed);
    const bool act_on = act_led_on.load(std::memory_order_relaxed);
    if(link_on) {
        setStatusColor(8, 3);
    } else {
        strip.setPixelColor(8, 0);
    }
    if(act_on) {
        if(enabled_outputs != 0x0F) {
            setStatusColorDimmed(9, 4);
        } else {
            setStatusColor(9, 4);
        }
    } else {
        strip.setPixelColor(9, 0);
    }
}

bool validStoredConfiguration(const StoredConfiguration &config) {
    if(config.magic != CONFIG_MAGIC || (config.enabled_mask & 0xF0) != 0) {
        return false;
    }

    for(uint8_t i = 0; i < 4; i++) {
        if(config.universes[i] > MAX_ARTNET_UNIVERSE) {
            return false;
        }
    }
    return true;
}

bool validStoredSettings(const StoredConfiguration &config) {
    if(config.settings_version != SETTINGS_VERSION ||
       config.universe_timeout_seconds == 0 ||
       config.universe_timeout_seconds > 60 ||
       config.led_brightness_percent > 100 ||
       config.lcd_brightness_percent < 10 ||
       config.lcd_brightness_percent > 100 ||
       config.lcd_timeout_seconds == 0 ||
       config.lcd_timeout_seconds > 60 ||
       config.charging_led > 2 ||
       config.disabled_output_dim_percent > 100) {
       return false;
    }

    for(uint8_t i = 0; i < 5; i++) {
        if(config.led_color_indices[i] >= STATUS_COLOR_COUNT) {
            return false;
        }
    }
    return true;
}

// Loads all persisted configuration. Invalid settings leave firmware defaults in place.
void loadConfiguration() {
    EEPROM.begin(sizeof(StoredConfiguration));
    StoredConfiguration stored{};
    EEPROM.get(0, stored);
    if(!validStoredConfiguration(stored)) {
        return;
    }

    critical_section_enter_blocking(&frame_lock);
    std::memcpy(connector_universe, stored.universes, sizeof(connector_universe));
    enabled_outputs = stored.enabled_mask;
    critical_section_exit(&frame_lock);

    const bool stored_ip_is_valid =
        stored.ip_address[0] != 0 && stored.ip_address[0] != 255 &&
        stored.ip_address[3] != 0 && stored.ip_address[3] != 255;
    if(stored_ip_is_valid) {
        configured_ip = IPAddress(stored.ip_address[0], stored.ip_address[1],
                                  stored.ip_address[2], stored.ip_address[3]);
        ip = configured_ip;
    }
    dhcp_enabled.store(stored.dhcp_enabled == 1, std::memory_order_relaxed);
    if(validStoredSettings(stored)) {
        universe_timeout_seconds = stored.universe_timeout_seconds;
        led_brightness_percent = stored.led_brightness_percent;
        lcd_brightness_percent = stored.lcd_brightness_percent;
        lcd_timeout_seconds = stored.lcd_timeout_seconds;
        std::memcpy(led_color_indices, stored.led_color_indices,
                    sizeof(led_color_indices));
        charging_led = stored.charging_led;
        disabled_output_dim_percent =
            stored.disabled_output_dim_percent;
        for(uint8_t universe = 0; universe < 4; universe++) {
            for(uint16_t address = 0; address < UNIVERSE_LENGTH; address++) {
                dmx_address_locked[universe][address] =
                    (stored.dmx_address_lock_bits[universe][address >> 3] &
                     (1u << (address & 7))) != 0;
            }
        }
        std::memcpy(dmx_address_value, stored.dmx_address_value,
                    sizeof(dmx_address_value));
    }
}

void saveConfiguration() {
    StoredConfiguration stored{};
    stored.magic = CONFIG_MAGIC;
    critical_section_enter_blocking(&frame_lock);
    std::memcpy(stored.universes, connector_universe, sizeof(connector_universe));
    stored.enabled_mask = enabled_outputs;
    critical_section_exit(&frame_lock);
    for(uint8_t i = 0; i < 4; i++) {
        stored.ip_address[i] = configured_ip[i];
    }
    stored.dhcp_enabled = dhcp_enabled.load(std::memory_order_relaxed) ? 1 : 0;
    stored.settings_version = SETTINGS_VERSION;
    stored.universe_timeout_seconds = universe_timeout_seconds;
    stored.led_brightness_percent = led_brightness_percent;
    stored.lcd_brightness_percent = lcd_brightness_percent;
    stored.lcd_timeout_seconds = lcd_timeout_seconds;
    std::memcpy(stored.led_color_indices, led_color_indices,
                sizeof(led_color_indices));
    stored.charging_led = charging_led;
    stored.disabled_output_dim_percent = disabled_output_dim_percent;
    std::memset(stored.dmx_address_lock_bits, 0,
                sizeof(stored.dmx_address_lock_bits));
    for(uint8_t universe = 0; universe < 4; universe++) {
        for(uint16_t address = 0; address < UNIVERSE_LENGTH; address++) {
            if(dmx_address_locked[universe][address]) {
                stored.dmx_address_lock_bits[universe][address >> 3] |=
                    static_cast<uint8_t>(1u << (address & 7));
            }
        }
    }
    std::memcpy(stored.dmx_address_value, dmx_address_value,
                sizeof(dmx_address_value));

    EEPROM.put(0, stored);
    configuration_saved = EEPROM.commit();
}

void drawNetworkStatus() {
    tft.fillRect(5, 25, 310, 38, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(8, 28);
    tft.print(F("ETHERNET: "));
    tft.setTextColor(ethernet_link_up.load(std::memory_order_relaxed) ? ST77XX_GREEN : ST77XX_RED);
    tft.print(ethernet_link_up.load(std::memory_order_relaxed) ? F("LINK UP") : F("NO LINK"));

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 44);
    tft.print(F("IP: "));
    IPAddress current_ip;
    critical_section_enter_blocking(&frame_lock);
    current_ip = ip;
    critical_section_exit(&frame_lock);
    tft.print(current_ip);

    uint32_t total_packets = 0;
    critical_section_enter_blocking(&frame_lock);
    for(uint8_t i = 0; i < 4; i++) {
        total_packets += received_packets[i];
    }
    critical_section_exit(&frame_lock);

    tft.setCursor(170, 44);
    tft.print(F("RX: "));
    tft.print(total_packets);
    tft.setCursor(8, 56);
    if(battery_voltage_valid) {
        tft.print(F("BAT "));
        tft.print(battery_voltage, 2);
        tft.print(F("V ~"));
        tft.print(battery_percentage);
        tft.print(F("% "));
        if(battery_charge_status_valid) {
            tft.setTextColor(battery_is_charging ? ST77XX_GREEN : ST77XX_CYAN);
            tft.print(battery_is_charging ? F("CHARGING") : F("DISCHARGING"));
        } else {
            tft.setTextColor(ST77XX_RED);
            tft.print(F("CHARGE STATUS ERR"));
        }
    } else {
        tft.setTextColor(ST77XX_RED);
        tft.print(F("BATTERY READ ERROR"));
    }
    drawSettingsButton();
}

void drawOutputStats(uint8_t output);

void drawOutputRow(uint8_t output) {
    const int16_t y = 66 + output * 37;
    uint16_t universe;
    uint32_t packet_count;
    uint32_t last_packet;
    bool has_received;
    bool enabled;

    critical_section_enter_blocking(&frame_lock);
    universe = connector_universe[output];
    packet_count = received_packets[output];
    last_packet = last_dmx_packet[output];
    has_received = received_any[output];
    enabled = (enabled_outputs & (1u << output)) != 0;
    critical_section_exit(&frame_lock);

    const bool selected = interface_mode != InterfaceMode::EditIp && output == selected_output;
    constexpr uint16_t disabled_row_color = 0x4208;
    const uint16_t row_color = enabled ? (selected ? 0x2104 : ST77XX_BLACK) :
                                         disabled_row_color;
    tft.fillRect(5, y, 310, 35, row_color);
    tft.drawRect(5, y, 310, 35, selected ? ST77XX_YELLOW :
                                           (enabled ? 0x7BEF : 0x8410));

    tft.setTextSize(1);
    tft.setCursor(10, y + 5);
    tft.setTextColor(!enabled ? 0xC618 : (selected ? ST77XX_YELLOW : ST77XX_WHITE));
    tft.print(F("OUT "));
    tft.print(output + 1);
    tft.print(F("  U: "));
    if(interface_mode == InterfaceMode::EditUniverse && selected) {
        tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
        tft.print(edited_universe);
        tft.setTextColor(ST77XX_YELLOW);
    } else {
        tft.print(universe);
    }
    tft.setCursor(176, y + 5);
    tft.setTextColor(enabled ? ST77XX_GREEN : 0xC618);
    tft.print(enabled ? F("ON") : F("OFF"));

    drawOutputStats(output);
}

void drawIpRow() {
    const bool selected = selected_output == 4;
    const uint16_t background = selected ? 0x2104 : ST77XX_BLACK;
    tft.fillRect(5, 218, 310, 20, background);
    tft.drawRect(5, 218, 310, 20, selected ? ST77XX_YELLOW : 0x7BEF);
    tft.setTextSize(1);
    tft.setTextColor(selected ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.setCursor(10, 224);
    tft.print(F("IP: "));
    if(interface_mode == InterfaceMode::EditIp) {
        for(uint8_t i = 0; i < 4; i++) {
            if(i == edited_ip_octet) {
                tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
            }
            tft.print(edited_ip[i]);
            tft.setTextColor(selected ? ST77XX_YELLOW : ST77XX_WHITE);
            if(i < 3) {
                tft.print('.');
            }
        }
    } else {
        IPAddress current_ip;
        critical_section_enter_blocking(&frame_lock);
        current_ip = ip;
        critical_section_exit(&frame_lock);
        tft.print(current_ip);
    }

    const bool using_dhcp = dhcp_enabled.load(std::memory_order_relaxed);
    const bool network_pending = network_reconfiguration_pending.load(std::memory_order_relaxed);
    tft.setCursor(172, 224);
    tft.setTextColor(using_dhcp ? ST77XX_CYAN : ST77XX_WHITE);
    tft.print(using_dhcp ? F("DHCP ON") : F("DHCP OFF"));
    if(network_pending) {
        tft.setCursor(242, 224);
        tft.setTextColor(ST77XX_YELLOW);
        tft.print(F("WAIT"));
    } else if(using_dhcp) {
        tft.setCursor(242, 224);
        tft.setTextColor(dhcp_lease_acquired.load(std::memory_order_relaxed) ?
                         ST77XX_GREEN : ST77XX_RED);
        tft.print(dhcp_lease_acquired.load(std::memory_order_relaxed) ? F("OK") : F("FAIL"));
    } else {
        tft.setCursor(242, 224);
        tft.setTextColor(configuration_saved ? ST77XX_GREEN : ST77XX_RED);
        tft.print(configuration_saved ? F("SAVED") : F("ERR"));
    }
}

void drawOutputStats(uint8_t output) {
    const int16_t y = 66 + output * 37;
    uint32_t packet_count;
    uint32_t last_packet;
    bool has_received;
    bool enabled;

    critical_section_enter_blocking(&frame_lock);
    packet_count = received_packets[output];
    last_packet = last_dmx_packet[output];
    has_received = received_any[output];
    enabled = (enabled_outputs & (1u << output)) != 0;
    critical_section_exit(&frame_lock);

    constexpr uint16_t disabled_row_color = 0x4208;
    const uint16_t background = enabled ?
        (output == selected_output ? 0x2104 : ST77XX_BLACK) : disabled_row_color;
    tft.fillRect(10, y + 19, 294, 13, background);
    tft.setCursor(10, y + 21);
    const uint32_t age = has_received ? millis() - last_packet : 0;
    const bool packet_stale = !has_received ||
        age > static_cast<uint32_t>(universe_timeout_seconds) * 1000;
    tft.setTextColor(packet_stale ? ST77XX_RED :
                     (enabled ? ST77XX_WHITE : 0xC618));
    tft.print(F("RX "));
    tft.print(packet_count);
    if(!has_received) {
        tft.print(F("  --"));
    } else if(!packet_stale) {
        tft.print(F("  "));
        tft.print(age);
        tft.print(F("ms"));
    }
}

const char *settingsCategoryName(uint8_t category) {
    switch(category) {
        case 0: return "DISPLAY";
        case 1: return "DMX OUTPUTS";
        case 2: return "STATUS LEDS";
        default: return "NETWORK";
    }
}

uint8_t settingsCategoryItemCount(uint8_t category) {
    switch(category) {
        case 0: return 2;
        case 1: return 3;
        case 2: return 8;
        default: return 2;
    }
}

const char *settingsItemName(uint8_t category, uint8_t item) {
    switch(category) {
        case 0:
            return item == 0 ? "LCD brightness" : "LCD sleep timeout";
        case 1:
            if(item == 0) return "Universe timeout";
            if(item == 1) return "View DMX channels";
            return "DMX address overrides";
        case 2:
            switch(item) {
                case 0: return "LED brightness";
                case 1: return "No signal color";
                case 2: return "Signal color";
                case 3: return "Packet activity";
                case 4: return "Ethernet link";
                case 5: return "Ethernet activity";
                case 6: return "Charging LED";
                default: return "Disabled output brightness";
            }
        default:
            return item == 0 ? "Static IP address" : "DHCP";
    }
}

uint8_t getSettingValue() {
    switch(settings_category) {
        case 0:
            return edited_setting_index == 0 ? lcd_brightness_percent :
                                                lcd_timeout_seconds;
        case 1:
            return universe_timeout_seconds;
        case 2:
            if(edited_setting_index == 0) {
                return led_brightness_percent;
            }
            if(edited_setting_index == 6) {
                return charging_led;
            }
            if(edited_setting_index == 7) {
                return disabled_output_dim_percent;
            }
            return led_color_indices[edited_setting_index - 1];
        default:
            return 0;
    }
}

void setSettingValue(uint8_t value) {
    switch(settings_category) {
        case 0:
            if(edited_setting_index == 0) {
                lcd_brightness_percent = value;
                applyLcdBrightness();
            } else {
                lcd_timeout_seconds = value;
            }
            break;
        case 1:
            universe_timeout_seconds = value;
            break;
        case 2:
            if(edited_setting_index == 0) {
                led_brightness_percent = value;
                applyLedBrightness();
            } else if(edited_setting_index == 6) {
                charging_led = value;
                setChargingLed(charging_led);
            } else if(edited_setting_index == 7) {
                disabled_output_dim_percent = value;
            } else {
                led_color_indices[edited_setting_index - 1] = value;
            }
            break;
        default:
            break;
    }
}

void drawSettingsInterface() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 7);
    if(settings_root_menu) {
        tft.print(F("SETTINGS"));
    } else {
        tft.print(settingsCategoryName(settings_category));
    }
    tft.drawFastHLine(5, 29, 310, ST77XX_CYAN);
    tft.setTextSize(1);

    if(interface_mode == InterfaceMode::EditSetting) {
        const char *label = settingsItemName(settings_category, edited_setting_index);
        tft.setTextColor(ST77XX_CYAN);
        tft.setCursor(10, 50);
        tft.print(label);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(10, 84);
        if((settings_category == 0 && edited_setting_index == 0) ||
           (settings_category == 2 && edited_setting_index == 0)) {
            tft.print(getSettingValue());
            tft.print(F("%"));
        } else if(settings_category == 2 && edited_setting_index == 6) {
            tft.print(charging_led == 0 ? F("LED0 RED") :
                      charging_led == 1 ? F("LED1 GREEN") : F("LED2 BLUE"));
        } else if(settings_category == 2 && edited_setting_index == 7) {
            tft.print(disabled_output_dim_percent);
            tft.print(F("%"));
        } else if(settings_category == 2) {
            const uint8_t color_index = getSettingValue();
            tft.print(STATUS_COLORS[color_index].name);
            tft.fillRect(10, 126, 48, 28,
                         tftHardware.color565(STATUS_COLORS[color_index].red,
                                              STATUS_COLORS[color_index].green,
                                              STATUS_COLORS[color_index].blue));
        } else {
            tft.print(getSettingValue());
            tft.print(F(" sec"));
        }
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(10, 190);
        tft.print(F("ROTATE TO CHANGE"));
        tft.setCursor(10, 208);
        tft.print(F("CLICK SAVE / HOLD CANCEL"));
    } else {
        const uint8_t item_count = settings_root_menu ? 6 :
                                   settingsCategoryItemCount(settings_category) + 1;
        constexpr uint8_t visible_items = 6;
        if(settings_menu_index < settings_menu_scroll) {
            settings_menu_scroll = settings_menu_index;
        } else if(settings_menu_index >= settings_menu_scroll + visible_items) {
            settings_menu_scroll = settings_menu_index - visible_items + 1;
        }
        if(settings_menu_scroll + visible_items > item_count && item_count > visible_items) {
            settings_menu_scroll = item_count - visible_items;
        } else if(settings_menu_scroll >= item_count) {
            settings_menu_scroll = 0;
        }

        const uint8_t last_item = item_count < settings_menu_scroll + visible_items ?
                                  item_count : settings_menu_scroll + visible_items;
        for(uint8_t item = settings_menu_scroll; item < last_item; item++) {
            const int16_t y = 37 + (item - settings_menu_scroll) * 27;
            const bool selected = item == settings_menu_index;
            if(selected) {
                tft.fillRect(6, y - 2, 308, 23, 0x2104);
                tft.drawRect(6, y - 2, 308, 23, ST77XX_YELLOW);
            }
            tft.setTextColor(selected ? ST77XX_YELLOW : ST77XX_WHITE);
            tft.setCursor(12, y + 4);
            if(settings_root_menu) {
                switch(item) {
                    case 0: tft.print(F("Display")); break;
                    case 1: tft.print(F("DMX Outputs")); break;
                    case 2: tft.print(F("Status LEDs")); break;
                    case 3: tft.print(F("Network")); break;
                    case 4: tft.print(F("Art-Net / sACN Sources")); break;
                    default: tft.print(F("Exit settings")); break;
                }
            } else if(item == settingsCategoryItemCount(settings_category)) {
                tft.print(F("< Back"));
            } else {
                tft.print(settingsItemName(settings_category, item));
                tft.setCursor(208, y + 4);
                if(settings_category == 0 && item == 0) {
                    tft.print(lcd_brightness_percent);
                    tft.print(F("%"));
                } else if(settings_category == 0) {
                    tft.print(lcd_timeout_seconds);
                    tft.print(F(" sec"));
                } else if(settings_category == 1 && item == 0) {
                    tft.print(universe_timeout_seconds);
                    tft.print(F(" sec"));
                } else if(settings_category == 1) {
                    tft.print(F("OPEN"));
                } else if(settings_category == 2 && item == 0) {
                    tft.print(led_brightness_percent);
                    tft.print(F("%"));
                } else if(settings_category == 2 && item == 6) {
                    tft.print(charging_led == 0 ? F("LED0 RED") :
                              charging_led == 1 ? F("LED1 GREEN") : F("LED2 BLUE"));
                } else if(settings_category == 2 && item == 7) {
                    tft.print(disabled_output_dim_percent);
                    tft.print(F("%"));
                } else if(settings_category == 2) {
                    const uint8_t color_index = led_color_indices[item - 1];
                    tft.setTextColor(tftHardware.color565(STATUS_COLORS[color_index].red,
                                                          STATUS_COLORS[color_index].green,
                                                          STATUS_COLORS[color_index].blue));
                    tft.print(STATUS_COLORS[color_index].name);
                    tft.setTextColor(selected ? ST77XX_YELLOW : ST77XX_WHITE);
                } else if(item == 0) {
                    IPAddress current_ip;
                    critical_section_enter_blocking(&frame_lock);
                    current_ip = ip;
                    critical_section_exit(&frame_lock);
                    tft.print(current_ip);
                } else {
                    tft.print(dhcp_enabled.load(std::memory_order_relaxed) ?
                              F("ON") : F("OFF"));
                }
            }
        }
        tft.setTextColor(ST77XX_CYAN);
        tft.setCursor(8, 225);
        tft.print(F("ROTATE SELECT / CLICK OPEN / HOLD BACK"));
    }

    if(!configuration_saved) {
        tft.setTextColor(ST77XX_RED);
        tft.setCursor(270, 8);
        tft.print(F("SAVE ERR"));
    }
}

void drawSourcesInterface() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 7);
    tft.print(F("ART-NET / sACN SOURCES"));
    tft.drawFastHLine(5, 29, 310, ST77XX_CYAN);
    tft.setTextSize(1);

    refreshActiveSources();
    if(active_source_count == 0) {
        source_menu_index = 0;
        source_menu_scroll = 0;
    } else {
        if(source_menu_index >= active_source_count) {
            source_menu_index = active_source_count - 1;
        }
        constexpr uint8_t visible_sources = 8;
        if(source_menu_index < source_menu_scroll) {
            source_menu_scroll = source_menu_index;
        } else if(source_menu_index >= source_menu_scroll + visible_sources) {
            source_menu_scroll = source_menu_index - visible_sources + 1;
        }
        if(source_menu_scroll + visible_sources > active_source_count &&
           active_source_count > visible_sources) {
            source_menu_scroll = active_source_count - visible_sources;
        } else if(source_menu_scroll >= active_source_count) {
            source_menu_scroll = 0;
        }

        const uint8_t last_source = active_source_count < source_menu_scroll + visible_sources ?
                                    active_source_count : source_menu_scroll + visible_sources;
        for(uint8_t i = source_menu_scroll; i < last_source; i++) {
            const SourceEntry &entry = active_source_entries[i];
            const int16_t y = 38 + (i - source_menu_scroll) * 22;
            const bool selected = i == source_menu_index;
            if(selected) {
                tft.fillRect(6, y - 2, 308, 20, 0x2104);
                tft.drawRect(6, y - 2, 308, 20, ST77XX_YELLOW);
            }
            tft.setTextColor(selected ? ST77XX_YELLOW : ST77XX_WHITE);
            tft.setCursor(12, y + 4);
            tft.print(entry.protocol == 0 ? F("Art-Net  ") : F("sACN     "));
            tft.print(IPAddress(entry.ip));
            tft.setCursor(220, y + 4);
            tft.print(F("U:"));
            tft.print(entry.universe);
        }
    }

    if(active_source_count == 0) {
        tft.setTextColor(ST77XX_RED);
        tft.setCursor(12, 44);
        tft.print(F("No active sources"));
    }

    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(8, 225);
    tft.print(F("ROTATE VIEW / HOLD BACK"));
}

// Channel view: 50 channels per page, with live values and lock state.
void drawDmxViewInterface() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 7);
    tft.print(F("DMX UNIVERSE "));
    tft.print(dmx_override_universe + 1);
    tft.drawFastHLine(5, 29, 310, ST77XX_CYAN);
    tft.setTextSize(1);

    const uint32_t now = millis();
    uint32_t view_last_packet;
    bool view_received;
    critical_section_enter_blocking(&frame_lock);
    view_last_packet = last_dmx_packet[dmx_override_universe];
    view_received = received_any[dmx_override_universe];
    uint8_t view_pending[UNIVERSE_LENGTH];
    for(uint16_t address = 0; address < UNIVERSE_LENGTH; address++) {
        view_pending[address] = pending_universedata[dmx_override_universe][address + 1];
    }
    critical_section_exit(&frame_lock);

    for(uint8_t row = 0; row < 10; row++) {
        const int16_t y = 36 + row * 18;
        for(uint8_t column = 0; column < 5; column++) {
            const uint16_t address = dmx_override_view_scroll + row * 5 + column;
            if(address >= UNIVERSE_LENGTH) break;

            const int16_t x = 5 + column * 62;
            const bool locked = dmx_address_locked[dmx_override_universe][address] != 0;
            const uint32_t age = view_last_packet == 0 ?
                                 0xFFFFFFFFu : now - view_last_packet;
            const bool receiving = view_received &&
                                   age <= static_cast<uint32_t>(universe_timeout_seconds) * 1000;
            const uint16_t border = locked ? ST77XX_RED :
                                    (receiving ? ST77XX_GREEN : 0x7BEF);

            tft.fillRect(x, y - 2, 58, 16, ST77XX_BLACK);
            tft.drawRect(x, y - 2, 58, 16, border);
            const uint8_t value = locked ?
                dmx_address_value[dmx_override_universe][address] :
                view_pending[address];
            tft.setCursor(x + 3, y + 3);
            tft.setTextColor(locked ? ST77XX_RED :
                             (receiving ? ST77XX_GREEN : ST77XX_WHITE));
            tft.print(address + 1);
            tft.print(F(" "));
            tft.print(value);
        }
    }

    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(8, 225);
    tft.print(F("ROTATE 50 CH / CLICK UNIVERSE / HOLD BACK"));
}

// Channel override editor: first select a field, then rotate to edit it.
void drawDmxOverrideInterface() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 7);
    tft.print(F("DMX ADDRESS OVERRIDE"));
    tft.drawFastHLine(5, 29, 310, ST77XX_CYAN);
    tft.setTextSize(1);

    const char *labels[] = {"Universe", "Address", "Lock", "Value"};
    for(uint8_t row = 0; row < 4; row++) {
        const int16_t y = 38 + row * 36;
        const bool selected = dmx_override_menu_index == row;
        const bool editing = selected && dmx_override_editing;
        if(selected) {
            tft.fillRect(6, y - 4, 308, 34,
                         editing ? ST77XX_YELLOW : 0x2104);
            tft.drawRect(6, y - 4, 308, 34,
                         editing ? ST77XX_WHITE : ST77XX_YELLOW);
        }
        tft.setTextColor(editing ? ST77XX_BLACK :
                         (selected ? ST77XX_YELLOW : ST77XX_CYAN));
        tft.setCursor(12, y);
        tft.print(labels[row]);
        tft.setTextSize(2);
        tft.setCursor(12, y + 14);
        tft.setTextColor(editing ? ST77XX_BLACK :
                         (selected ? ST77XX_YELLOW : ST77XX_WHITE));
        if(row == 0) {
            tft.print(dmx_override_universe + 1);
        } else if(row == 1) {
            tft.print(dmx_override_address + 1);
        } else if(row == 2) {
            tft.print(dmx_address_locked[dmx_override_universe][dmx_override_address] ?
                      F("ON") : F("OFF"));
        } else {
            tft.print(dmx_address_value[dmx_override_universe][dmx_override_address]);
        }
        tft.setTextSize(1);
    }

    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(8, 225);
    tft.print(dmx_override_editing ? F("ROTATE CHANGE / CLICK APPLY / HOLD BACK") :
                                      F("ROTATE SELECT / CLICK EDIT / HOLD BACK"));
}

// drawInterface redraws the logical 320x240 canvas, then copies the completed
// frame to the physical ST7789. This avoids visible partial or flickering updates.
void drawInterface(bool redraw_all) {
    if(interface_mode == InterfaceMode::DmxViewMenu) {
        drawDmxViewInterface();
        drawFrameToDisplay();
        return;
    }
    if(interface_mode == InterfaceMode::DmxOverrideMenu) {
        drawDmxOverrideInterface();
        drawFrameToDisplay();
        return;
    }
    if(interface_mode == InterfaceMode::SourcesMenu) {
        drawSourcesInterface();
        drawFrameToDisplay();
        return;
    }
    if(interface_mode == InterfaceMode::SettingsMenu ||
       interface_mode == InterfaceMode::EditSetting) {
        drawSettingsInterface();
        drawFrameToDisplay();
        return;
    }

    if(redraw_all) {
        tft.fillScreen(ST77XX_BLACK);
    } else {
        tft.fillRect(0, 0, 320, 24, ST77XX_BLACK);
    }

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(8, 7);
    tft.print(F("DMXion Node"));
    tft.setTextSize(1);
    tft.setCursor(162, 12);
    if(interface_mode == InterfaceMode::EditIp) {
        tft.print(F("CLICK NEXT / HOLD SAVE"));
    } else if(interface_mode == InterfaceMode::EditUniverse) {
        tft.print(F("CLICK SAVE DMX U"));
    } else if(selected_output == SETTINGS_MAIN_OUTPUT) {
        tft.print(F("CLICK OPEN SETTINGS"));
    } else if(selected_output == 4) {
        tft.print(F("CLICK DHCP / HOLD IP"));
    } else {
        tft.print(F("HOLD: EDIT DMX U"));
    }

    tft.drawFastHLine(5, 23, 310, ST77XX_CYAN);

    drawNetworkStatus();
    for(uint8_t i = 0; i < 4; i++) {
        drawOutputRow(i);
    }
    drawIpRow();
    drawFrameToDisplay();
}

void recordUserActivity() {
    last_user_activity = millis();
    if(!backlight_is_on) {
        backlight_is_on = true;
        applyLcdBrightness();
        drawInterface(true);
    }
}

void adjustSettingValue(int32_t delta);

void handleEncoderRotation(int32_t delta) {
    recordUserActivity();
    if(interface_mode == InterfaceMode::EditUniverse) {
        const int32_t universe_count =
            static_cast<int32_t>(MAX_ARTNET_UNIVERSE) + 1;
        int32_t universe =
            (static_cast<int32_t>(edited_universe) + delta) % universe_count;
        if(universe < 0) {
            universe += universe_count;
        }
        edited_universe = static_cast<uint16_t>(universe);
        drawOutputRow(selected_output);
        return;
    }
    if(interface_mode == InterfaceMode::EditIp) {
        edited_ip[edited_ip_octet] = static_cast<uint8_t>(
            edited_ip[edited_ip_octet] + delta);
        drawIpRow();
        return;
    }
    if(interface_mode == InterfaceMode::EditSetting) {
        adjustSettingValue(delta);
        drawInterface(false);
        return;
    }
    if(interface_mode == InterfaceMode::DmxViewMenu) {
        int32_t view = static_cast<int32_t>(dmx_override_view_scroll) + delta * 50;
        if(view < 0) view = 0;
        if(view > UNIVERSE_LENGTH - 50) view = UNIVERSE_LENGTH - 50;
        dmx_override_view_scroll = static_cast<uint16_t>(view);
        drawInterface(true);
        return;
    }
    if(interface_mode == InterfaceMode::DmxOverrideMenu) {
        if(!dmx_override_editing) {
            int32_t row = (static_cast<int32_t>(dmx_override_menu_index) + delta) % 4;
            if(row < 0) row += 4;
            dmx_override_menu_index = static_cast<uint8_t>(row);
        } else {
            int32_t value;
            switch(dmx_override_menu_index) {
                case 0:
                    value = (static_cast<int32_t>(dmx_override_universe) + delta) % 4;
                    if(value < 0) value += 4;
                    dmx_override_universe = static_cast<uint8_t>(value);
                    break;
                case 1:
                    value = (static_cast<int32_t>(dmx_override_address) + delta) % UNIVERSE_LENGTH;
                    if(value < 0) value += UNIVERSE_LENGTH;
                    dmx_override_address = static_cast<uint16_t>(value);
                    break;
                case 3:
                    value = static_cast<int32_t>(dmx_address_value[dmx_override_universe][dmx_override_address]) + delta;
                    if(value < 0) value = 0;
                    if(value > 255) value = 255;
                    dmx_address_value[dmx_override_universe][dmx_override_address] = static_cast<uint8_t>(value);
                    break;
                case 2:
                    break;
            }
        }
        drawInterface(true);
        return;
    }
    if(interface_mode == InterfaceMode::SourcesMenu) {
        refreshActiveSources();
        if(active_source_count == 0) {
            source_menu_index = 0;
            source_menu_scroll = 0;
        } else {
            int32_t index = (static_cast<int32_t>(source_menu_index) + delta) %
                            active_source_count;
            if(index < 0) {
                index += active_source_count;
            }
            source_menu_index = static_cast<uint8_t>(index);
        }
        drawInterface(true);
        return;
    }
    if(interface_mode == InterfaceMode::SettingsMenu) {
        const uint8_t item_count = settings_root_menu ? 6 :
                                   settingsCategoryItemCount(settings_category) + 1;
        int32_t index = (static_cast<int32_t>(settings_menu_index) + delta) %
                        item_count;
        if(index < 0) {
            index += item_count;
        }
        settings_menu_index = static_cast<uint8_t>(index);
        drawInterface(false);
        return;
    }

    int32_t output = (static_cast<int32_t>(selected_output) + delta) % 6;
    if(output < 0) {
        output += 6;
    }
    selected_output = static_cast<uint8_t>(output);
    drawInterface(false);
}

void toggleSelectedOutput() {
    if(selected_output >= 4) {
        return;
    }
    critical_section_enter_blocking(&frame_lock);
    enabled_outputs ^= static_cast<uint8_t>(1u << selected_output);
    critical_section_exit(&frame_lock);
    saveConfiguration();
    drawInterface(false);
}

void saveEditedUniverse() {
    critical_section_enter_blocking(&frame_lock);
    connector_universe[selected_output] = edited_universe;
    last_dmx_packet[selected_output] = 0;
    received_packets[selected_output] = 0;
    received_any[selected_output] = false;
    keep_sending[selected_output].store((enabled_outputs & (1u << selected_output)) != 0,
                                       std::memory_order_relaxed);
    critical_section_exit(&frame_lock);
    interface_mode = InterfaceMode::SelectOutput;
    saveConfiguration();
    if(network_ready.load(std::memory_order_acquire)) {
        sacn_reconfigure_requested.store(true, std::memory_order_release);
    }
    drawInterface(false);
}

bool applyNetworkSettings(const IPAddress &previous_ip, bool previous_dhcp) {
    if(network_ready.load(std::memory_order_acquire)) {
        network_pause_requested.store(true, std::memory_order_release);
        const uint32_t pause_started = millis();
        while(!network_paused.load(std::memory_order_acquire)) {
            if(static_cast<uint32_t>(millis() - pause_started) >= 1000) {
                network_pause_requested.store(false, std::memory_order_release);
                configuration_saved = false;
                return false;
            }
            delay(1);
        }
    }

    saveConfiguration();
    if(!configuration_saved) {
        configured_ip = previous_ip;
        dhcp_enabled.store(previous_dhcp, std::memory_order_relaxed);
        network_pause_requested.store(false, std::memory_order_release);
        return false;
    }

    if(network_ready.load(std::memory_order_acquire)) {
        dhcp_lease_acquired.store(false, std::memory_order_relaxed);
        network_reconfiguration_pending.store(true, std::memory_order_release);
        network_reconfigure_requested.store(true, std::memory_order_release);
    }
    return true;
}

bool applyEditedIp() {
    const bool invalid_ip =
        (edited_ip[0] == 0 && edited_ip[1] == 0 && edited_ip[2] == 0 && edited_ip[3] == 0) ||
        (edited_ip[0] >= 224) ||
        (edited_ip[0] == 255 && edited_ip[1] == 255 && edited_ip[2] == 255 && edited_ip[3] == 255);
    if(invalid_ip) {
        configuration_saved = false;
        return false;
    }

    const IPAddress previous_ip = configured_ip;
    const bool previous_dhcp = dhcp_enabled.load(std::memory_order_relaxed);
    configured_ip = IPAddress(edited_ip[0], edited_ip[1], edited_ip[2], edited_ip[3]);
    dhcp_enabled.store(false, std::memory_order_relaxed);
    return applyNetworkSettings(previous_ip, previous_dhcp);
}

void toggleDhcp() {
    const IPAddress previous_ip = configured_ip;
    const bool previous_dhcp = dhcp_enabled.load(std::memory_order_relaxed);
    dhcp_enabled.store(!previous_dhcp, std::memory_order_relaxed);
    configuration_saved = applyNetworkSettings(previous_ip, previous_dhcp);
    drawInterface(false);
}

void saveEditedIp() {
    const bool saved = applyEditedIp();
    interface_mode = edit_ip_returns_to_settings ? InterfaceMode::SettingsMenu :
                                                   InterfaceMode::SelectOutput;
    edit_ip_returns_to_settings = false;
    configuration_saved = saved;
    drawInterface(false);
}

void enterSettings() {
    settings_root_menu = true;
    settings_menu_index = 0;
    settings_menu_scroll = 0;
    interface_mode = InterfaceMode::SettingsMenu;
    drawInterface(true);
}

void exitSettings() {
    interface_mode = InterfaceMode::SelectOutput;
    selected_output = SETTINGS_MAIN_OUTPUT;
    drawInterface(true);
}

void openSettingsCategory(uint8_t category) {
    settings_category = category;
    settings_menu_index = 0;
    settings_menu_scroll = 0;
    settings_root_menu = false;
    drawInterface(true);
}

void selectSettingsMenuItem() {
    if(settings_root_menu) {
        if(settings_menu_index < 4) {
            openSettingsCategory(settings_menu_index);
        } else if(settings_menu_index == 4) {
            source_menu_index = 0;
            source_menu_scroll = 0;
            interface_mode = InterfaceMode::SourcesMenu;
            drawInterface(true);
        } else {
            exitSettings();
        }
        return;
    }

    const uint8_t item_count = settingsCategoryItemCount(settings_category);
    if(settings_menu_index == item_count) {
        settings_root_menu = true;
        settings_menu_index = settings_category;
        settings_menu_scroll = 0;
        drawInterface(true);
        return;
    }

    if(settings_category == 1 && settings_menu_index == 1) {
        dmx_override_universe = 0;
        dmx_override_view_scroll = 0;
        interface_mode = InterfaceMode::DmxViewMenu;
        drawInterface(true);
        return;
    }
    if(settings_category == 1 && settings_menu_index == 2) {
        dmx_override_universe = 0;
        dmx_override_address = 0;
        dmx_override_menu_index = 0;
        dmx_override_editing = false;
        interface_mode = InterfaceMode::DmxOverrideMenu;
        drawInterface(true);
        return;
    }
    if(settings_category == 3 && settings_menu_index == 1) {
        toggleDhcp();
        return;
    }
    if(settings_category == 3) {
        selected_output = 4;
        edit_ip_returns_to_settings = true;
        edited_ip[0] = configured_ip[0];
        edited_ip[1] = configured_ip[1];
        edited_ip[2] = configured_ip[2];
        edited_ip[3] = configured_ip[3];
        edited_ip_octet = 0;
        interface_mode = InterfaceMode::EditIp;
        drawInterface(true);
        return;
    }

    edited_setting_index = settings_menu_index;
    original_setting_value = getSettingValue();
    interface_mode = InterfaceMode::EditSetting;
    drawInterface(true);
}

void adjustSettingValue(int32_t delta) {
    uint8_t value = getSettingValue();
    if(settings_category == 2 && edited_setting_index > 0 &&
       edited_setting_index < 6) {
        int32_t color = (static_cast<int32_t>(value) + delta) %
                        STATUS_COLOR_COUNT;
        if(color < 0) {
            color += STATUS_COLOR_COUNT;
        }
        value = static_cast<uint8_t>(color);
    } else {
        const uint8_t minimum =
            settings_category == 2 &&
            (edited_setting_index == 6 || edited_setting_index == 7) ? 0 :
            (settings_category == 0 && edited_setting_index == 0 ? 10 :
            (settings_category == 2 && edited_setting_index == 0 ? 0 : 1));
        const uint8_t maximum =
            settings_category == 2 && edited_setting_index == 6 ? 2 :
            (settings_category == 2 && edited_setting_index == 7 ? 100 :
            (settings_category == 0 && edited_setting_index == 0 ? 100 :
            (settings_category == 2 ? 100 : 60)));
        const int32_t adjusted_value = static_cast<int32_t>(value) + delta;
        if(adjusted_value < minimum) {
            value = minimum;
        } else if(adjusted_value > maximum) {
            value = maximum;
        } else {
            value = static_cast<uint8_t>(adjusted_value);
        }
    }
    setSettingValue(value);
}

void finishSettingEdit(bool save) {
    if(!save) {
        setSettingValue(original_setting_value);
    } else {
        saveConfiguration();
        if(!configuration_saved) {
            setSettingValue(original_setting_value);
        }
    }
    interface_mode = InterfaceMode::SettingsMenu;
    drawInterface(true);
}

// Polls the rotary encoder and button. Short clicks navigate/select, while a
// 650 ms hold performs mode-specific back or cancel actions.
void handleEncoder() {
    static int32_t encoder_steps = 0;
    static bool raw_button_state = HIGH;
    static bool stable_button_state = HIGH;
    static InterfaceMode button_mode_at_press = InterfaceMode::SelectOutput;
    static bool long_press_entered_edit = false;
    static uint32_t raw_state_changed_at = 0;
    static uint32_t button_pressed_at = 0;
    static uint32_t last_rotation_ms = 0;
    static uint8_t fast_rotation_count = 0;

    encoder_steps += encoder_transition_count.exchange(0, std::memory_order_relaxed);
    const int32_t completed_steps = encoder_steps / 2;
    encoder_steps %= 2;
    if(completed_steps != 0) {
        int32_t accelerated_steps = completed_steps;

        if(interface_mode == InterfaceMode::DmxOverrideMenu && dmx_override_editing) {
            const uint32_t rotation_ms = millis();
            const uint32_t gap = rotation_ms - last_rotation_ms;
            if(last_rotation_ms == 0 || gap >= ENCODER_FAST_ROTATION_TIMEOUT_MS) {
                fast_rotation_count = 0;
            }

            const int32_t step_magnitude = completed_steps < 0 ?
                                           -completed_steps : completed_steps;
            if(gap < ENCODER_FAST_ROTATION_MS) {
                fast_rotation_count++;
            } else {
                fast_rotation_count = 0;
            }

            int32_t multiplier = 1;
            if(step_magnitude >= 8) {
                multiplier = 20;
            } else if(step_magnitude >= 4) {
                multiplier = 10;
            } else if(step_magnitude >= 2) {
                multiplier = 5;
            } else if(fast_rotation_count >= 2 && gap < ENCODER_FAST_ROTATION_MS) {
                multiplier = 3;
            }

            accelerated_steps = completed_steps * multiplier;
            last_rotation_ms = rotation_ms;
        } else {
            last_rotation_ms = 0;
            fast_rotation_count = 0;
        }

        // Preserve the encoder's existing direction correction.
        handleEncoderRotation(-accelerated_steps);
    }

    const uint32_t now = millis();
    const bool button_state = digitalRead(ENCODER_SWITCH);
    if(button_state != raw_button_state) {
        raw_button_state = button_state;
        raw_state_changed_at = now;
    }

    if(button_state != stable_button_state &&
       static_cast<uint32_t>(now - raw_state_changed_at) >= 25) {
        stable_button_state = button_state;
        if(stable_button_state == LOW) {
            button_pressed_at = now;
            button_mode_at_press = interface_mode;
            long_press_entered_edit = false;
            recordUserActivity();
        } else {
            recordUserActivity();
            if(button_mode_at_press == InterfaceMode::EditUniverse) {
                saveEditedUniverse();
            } else if(button_mode_at_press == InterfaceMode::EditIp &&
                      !long_press_entered_edit) {
                edited_ip_octet = static_cast<uint8_t>((edited_ip_octet + 1) % 4);
                drawIpRow();
            } else if(button_mode_at_press == InterfaceMode::EditSetting &&
                      !long_press_entered_edit) {
                finishSettingEdit(true);
            } else if(button_mode_at_press == InterfaceMode::DmxViewMenu &&
                      !long_press_entered_edit) {
                dmx_override_universe = (dmx_override_universe + 1) % 4;
                dmx_override_view_scroll = 0;
                drawInterface(true);
            } else if(button_mode_at_press == InterfaceMode::DmxOverrideMenu &&
                      !long_press_entered_edit) {
                if(dmx_override_menu_index == 2) {
                    dmx_address_locked[dmx_override_universe][dmx_override_address] =
                        dmx_address_locked[dmx_override_universe][dmx_override_address] ? 0 : 1;
                    dmx_override_editing = false;
                    saveConfiguration();
                } else if(!dmx_override_editing) {
                    dmx_override_editing = true;
                } else {
                    dmx_override_editing = false;
                    saveConfiguration();
                }
                drawInterface(true);
            } else if(button_mode_at_press == InterfaceMode::SettingsMenu &&
                      !long_press_entered_edit) {
                selectSettingsMenuItem();
            } else if(!long_press_entered_edit) {
                if(selected_output == 4) {
                    toggleDhcp();
                } else if(selected_output == SETTINGS_MAIN_OUTPUT) {
                    enterSettings();
                } else {
                    toggleSelectedOutput();
                }
            }
        }
    }

    if(stable_button_state == LOW && !long_press_entered_edit &&
       static_cast<uint32_t>(now - button_pressed_at) >= ENCODER_HOLD_MS) {
        long_press_entered_edit = true;
        recordUserActivity();
        if(button_mode_at_press == InterfaceMode::EditIp) {
            saveEditedIp();
            return;
        }
        if(button_mode_at_press == InterfaceMode::EditSetting) {
            finishSettingEdit(false);
            return;
        }
        if(button_mode_at_press == InterfaceMode::DmxViewMenu) {
            interface_mode = InterfaceMode::SettingsMenu;
            settings_root_menu = false;
            settings_category = 1;
            settings_menu_index = 1;
            settings_menu_scroll = 0;
            drawInterface(true);
            return;
        }
        if(button_mode_at_press == InterfaceMode::DmxOverrideMenu) {
            saveConfiguration();
            dmx_override_editing = false;
            interface_mode = InterfaceMode::SettingsMenu;
            settings_root_menu = false;
            settings_category = 1;
            settings_menu_index = 2;
            settings_menu_scroll = 0;
            drawInterface(true);
            return;
        }
        if(button_mode_at_press == InterfaceMode::SourcesMenu) {
            interface_mode = InterfaceMode::SettingsMenu;
            settings_root_menu = true;
            settings_menu_index = 4;
            settings_menu_scroll = 0;
            drawInterface(true);
            return;
        }
        if(button_mode_at_press == InterfaceMode::SettingsMenu) {
            if(settings_root_menu) {
                exitSettings();
            } else {
                settings_root_menu = true;
                settings_menu_index = settings_category;
                drawInterface(true);
            }
            return;
        }
        if(button_mode_at_press != InterfaceMode::SelectOutput) {
            return;
        }

        if(selected_output == 4) {
            edit_ip_returns_to_settings = false;
            interface_mode = InterfaceMode::EditIp;
            edited_ip[0] = configured_ip[0];
            edited_ip[1] = configured_ip[1];
            edited_ip[2] = configured_ip[2];
            edited_ip[3] = configured_ip[3];
            edited_ip_octet = 0;
        } else if(selected_output == SETTINGS_MAIN_OUTPUT) {
            enterSettings();
        } else {
            interface_mode = InterfaceMode::EditUniverse;
            critical_section_enter_blocking(&frame_lock);
            edited_universe = connector_universe[selected_output];
            critical_section_exit(&frame_lock);
        }
        drawInterface(false);
    }
}

void updateBacklight() {
    if(backlight_is_on &&
       static_cast<uint32_t>(millis() - last_user_activity) >=
           static_cast<uint32_t>(lcd_timeout_seconds) * 1000) {
        backlight_is_on = false;
        applyLcdBrightness();
    }
}

void updateStartupRainbow() {
    if(!startup_rainbow_running) {
        return;
    }

    const uint32_t now = millis();
    if(static_cast<uint32_t>(now - startup_rainbow_last_frame) < 20) {
        return;
    }

    for(uint8_t pixel = 0; pixel < LED_COUNT; pixel++) {
        const uint16_t hue = static_cast<uint16_t>(
            (static_cast<uint32_t>(startup_rainbow_frame) * 65536 / 120) +
            (static_cast<uint32_t>(pixel) * 65536 / LED_COUNT));
        strip.setPixelColor(pixel, strip.ColorHSV(hue, 255, 100));
    }
    strip.show();
    startup_rainbow_frame++;
    startup_rainbow_last_frame = now;
}

bool updateStartupRainbowTimer(repeating_timer_t *) {
    updateStartupRainbow();
    return true;
}

void startStartupRainbow() {
    startup_rainbow_running = true;
    startup_rainbow_frame = 0;
    startup_rainbow_last_frame = millis();
    add_repeating_timer_ms(-20, updateStartupRainbowTimer, nullptr,
                           &startup_rainbow_timer);
}

void stopStartupRainbow() {
    startup_rainbow_running = false;
    cancel_repeating_timer(&startup_rainbow_timer);
    strip.clear();
    strip.show();
}

// Arduino setup runs once. It initializes hardware, network services, DMX output,
// the display, and starts background timers.
void setup() {
    Serial.begin(115200);
    critical_section_init(&frame_lock);
    loadConfiguration();
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_SWITCH, INPUT_PULLUP);
    encoder_previous_ab = static_cast<uint8_t>(
        (gpio_get(ENCODER_A) << 1) | gpio_get(ENCODER_B));
    last_user_activity = millis();

    strip.begin();
    applyLedBrightness();
    startStartupRainbow();

    // Give time to open the serial monitor
    delay(1000);

    // Start the charger
    if(!chargerStart(McciCatenaNpm1300::cNPM1300::Ntc::NotFitted, McciCatenaNpm1300::cNPM1300::Vterm::V4_20, McciCatenaNpm1300::cNPM1300::Current::MA_1500, 800, charging_led)) {
        while(1) {
            Serial.println("Charger init failed");
            delay(20);
        }
    } else {
        Serial.println("Charger initialized succesfully");
    }
    add_repeating_timer_ms(-5000, sendChargerData, NULL, &charger_timer);

    // Configure the LEDs (started above; keep the rainbow running throughout setup)
    // Configure the display
    delay(500);
    SPI.setCS(TFT_CS);
    SPI.setTX(TFT_MOSI);
    SPI.setSCK(TFT_SCK);
    pinMode(TFT_BACKLIGHT, OUTPUT);
    analogWrite(TFT_BACKLIGHT, 0);
    tftHardware.init(240, 320);
    tftHardware.setRotation(3);
    tftCanvas.fillScreen(ST77XX_BLACK);
    drawFrameToDisplay();
    backlight_is_on = true;
    applyLcdBrightness();

    // Configure the DMX
    for(int i = 0; i < 4; i++) {
        pinMode(UNIVERSEOE[i], OUTPUT);
        digitalWrite(UNIVERSEOE[i], HIGH);
        if(i < 2) {
            universes[i].begin(UNIVERSETX[i], pio0);
        } else {
            universes[i].begin(UNIVERSETX[i], pio1);
        }
    }
    add_repeating_timer_ms(-(1000/DMX_SPEED), sendDMX, NULL, &interval_timer);

    // Configure the network
    pinMode(LINKLED, INPUT);
    pinMode(ACTLED, INPUT);
    gpio_set_irq_enabled_with_callback(LINKLED, GPIO_IRQ_EDGE_RISE, true, onLED);
    gpio_set_irq_enabled_with_callback(LINKLED, GPIO_IRQ_EDGE_FALL, true, onLED);
    gpio_set_irq_enabled_with_callback(ACTLED, GPIO_IRQ_EDGE_RISE, true, onLED);
    gpio_set_irq_enabled_with_callback(ACTLED, GPIO_IRQ_EDGE_FALL, true, onLED);
    gpio_set_irq_enabled(ENCODER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    link_led_on.store(gpio_get(LINKLED) == 0, std::memory_order_relaxed);
    act_led_on.store(gpio_get(ACTLED) == 0, std::memory_order_relaxed);
    SPI1.setCS(ETH_CS);
    SPI1.setTX(ETH_MOSI);
    SPI1.setRX(ETH_MISO);
    SPI1.setSCK(ETH_SCK);

    Ethernet.init(29, SPI1);

    const bool request_dhcp = dhcp_enabled.load(std::memory_order_relaxed);
    const bool lease_acquired = request_dhcp &&
                                Ethernet.begin(mac, 8000, 3000) == 1;
    if(lease_acquired) {
        ip = Ethernet.localIP();
        Serial.print(F("DHCP lease acquired: "));
        Serial.println(ip);
    } else {
        if(request_dhcp) {
            Serial.println(F("DHCP lease failed; using saved static IP"));
        }
        Ethernet.begin(mac, configured_ip);
        ip = configured_ip;
    }

    dhcp_lease_acquired.store(lease_acquired, std::memory_order_relaxed);
    if(Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println(F("W5500 not detected"));
    } else {
        ethernet_link_up.store(Ethernet.linkStatus() == LinkON,
                               std::memory_order_relaxed);
        beginArtNetReceiver();
        beginSacnReceivers();
        network_ready.store(true, std::memory_order_release);
    }

    updateStartupRainbow();
    drawInterface(true);
    stopStartupRainbow();
}

// Arduino core loop: UI input, backlight, battery, NeoPixels, and screen updates.
void loop() {
    handleEncoder();

    updateBacklight();
    updateBatteryMeasurement();
    if(!startup_rainbow_running) {
        updateStatusLeds();
        strip.show();
    }

    const uint32_t now = millis();
    if(backlight_is_on && interface_mode == InterfaceMode::SourcesMenu &&
       static_cast<uint32_t>(now - last_interface_stats_update) >= 500) {
        drawInterface(true);
        last_interface_stats_update = now;
    }
    if(backlight_is_on && interface_mode == InterfaceMode::DmxViewMenu &&
       static_cast<uint32_t>(now - last_interface_stats_update) >= 500) {
        drawInterface(true);
        last_interface_stats_update = now;
    }
    if(backlight_is_on &&
       interface_mode != InterfaceMode::SettingsMenu &&
       interface_mode != InterfaceMode::EditSetting &&
       interface_mode != InterfaceMode::SourcesMenu &&
       interface_mode != InterfaceMode::DmxOverrideMenu &&
       interface_mode != InterfaceMode::DmxViewMenu &&
       static_cast<uint32_t>(now - last_interface_stats_update) >= 500) {
        drawNetworkStatus();
        drawIpRow();
        for(uint8_t i = 0; i < 4; i++) {
            drawOutputStats(i);
        }
        drawFrameToDisplay();
        last_interface_stats_update = now;
    }
}

// Second-core loop: Ethernet maintenance and Art-Net/sACN packet reception.
void loop1() {
    static uint32_t last_dhcp_maintain = 0;
    static uint32_t last_dhcp_attempt = 0;
    if(network_reconfigure_requested.load(std::memory_order_acquire)) {
        stopSacnReceivers();
        const bool request_dhcp = dhcp_enabled.load(std::memory_order_acquire);
        const bool lease_acquired = request_dhcp &&
                                    Ethernet.begin(mac, 8000, 3000) == 1;
        if(lease_acquired) {
            critical_section_enter_blocking(&frame_lock);
            ip = Ethernet.localIP();
            critical_section_exit(&frame_lock);
            Serial.print(F("DHCP lease acquired: "));
            Serial.println(Ethernet.localIP());
        } else {
            if(request_dhcp) {
                Serial.println(F("DHCP lease failed; using saved static IP"));
            }
            Ethernet.begin(mac, configured_ip);
            critical_section_enter_blocking(&frame_lock);
            ip = configured_ip;
            critical_section_exit(&frame_lock);
        }
        dhcp_lease_acquired.store(lease_acquired, std::memory_order_relaxed);
        beginArtNetReceiver();
        beginSacnReceivers();
        sacn_reconfigure_requested.store(false, std::memory_order_release);
        ethernet_link_up.store(Ethernet.linkStatus() == LinkON, std::memory_order_relaxed);
        network_reconfigure_requested.store(false, std::memory_order_release);
        network_reconfiguration_pending.store(false, std::memory_order_release);
        network_pause_requested.store(false, std::memory_order_release);
        network_paused.store(false, std::memory_order_release);
    } else if(network_pause_requested.load(std::memory_order_acquire)) {
        network_paused.store(true, std::memory_order_release);
        delay(1);
    } else if(network_ready.load(std::memory_order_acquire)) {
        network_paused.store(false, std::memory_order_release);
        if(sacn_reconfigure_requested.exchange(false, std::memory_order_acq_rel)) {
            beginSacnReceivers();
        }
        receiveArtNetPackets();
        receiveSacnPackets();
        static uint32_t last_link_check = 0;
        const uint32_t now = millis();
        if(static_cast<uint32_t>(now - last_link_check) >= 500) {
            const bool link_up = Ethernet.linkStatus() == LinkON;
            ethernet_link_up.store(link_up, std::memory_order_relaxed);
            last_link_check = now;
            if(link_up &&
               dhcp_enabled.load(std::memory_order_relaxed) &&
               !dhcp_lease_acquired.load(std::memory_order_relaxed) &&
               static_cast<uint32_t>(now - last_dhcp_attempt) >= 5000) {
                last_dhcp_attempt = now;
                network_reconfiguration_pending.store(true, std::memory_order_release);
                const bool lease_acquired = Ethernet.begin(mac, 8000, 3000) == 1;
                if(lease_acquired) {
                    critical_section_enter_blocking(&frame_lock);
                    ip = Ethernet.localIP();
                    critical_section_exit(&frame_lock);
                    Serial.print(F("DHCP lease acquired: "));
                    Serial.println(Ethernet.localIP());
                } else {
                    Ethernet.begin(mac, configured_ip);
                    critical_section_enter_blocking(&frame_lock);
                    ip = configured_ip;
                    critical_section_exit(&frame_lock);
                    Serial.println(F("DHCP retry failed; keeping saved static IP"));
                }
                dhcp_lease_acquired.store(lease_acquired, std::memory_order_relaxed);
                network_reconfiguration_pending.store(false, std::memory_order_release);
            }
        }
        if(static_cast<uint32_t>(now - last_dhcp_maintain) >= 5000 &&
           dhcp_enabled.load(std::memory_order_relaxed) &&
           dhcp_lease_acquired.load(std::memory_order_relaxed)) {
            Ethernet.maintain();
            critical_section_enter_blocking(&frame_lock);
            ip = Ethernet.localIP();
            critical_section_exit(&frame_lock);
            last_dhcp_maintain = now;
        }
    } else {
        delay(1);
    }
}