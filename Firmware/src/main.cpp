#include <Arduino.h>
#include <Charger.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <DmxOutput.h>
#include <pico/critical_section.h>
#include <atomic>
#include <cstring>
#include <EEPROM.h>

// Configurables
#define DMX_SPEED 30 // In hertz
constexpr uint16_t MAX_ARTNET_UNIVERSE = 0x7FFF;
constexpr uint16_t ARTNET_PORT = 6454;
constexpr size_t ARTNET_PACKET_MAX_SIZE = 530;
constexpr uint16_t SACN_PORT = 5568;
constexpr size_t SACN_PACKET_MAX_SIZE = 638;
constexpr uint32_t CONFIG_MAGIC = 0x444D5831;
constexpr uint32_t BACKLIGHT_TIMEOUT_MS = 60000;
constexpr uint32_t ENCODER_HOLD_MS = 650;
constexpr uint8_t ENCODER_A = 8;
constexpr uint8_t ENCODER_B = 9;
constexpr uint8_t ENCODER_SWITCH = 7;
constexpr uint8_t DEFAULT_UNIVERSE_TIMEOUT_SECONDS = 10;
constexpr uint8_t DEFAULT_LED_BRIGHTNESS_PERCENT = 39;
constexpr uint8_t DEFAULT_LCD_BRIGHTNESS_PERCENT = 100;
constexpr uint8_t DEFAULT_LCD_TIMEOUT_SECONDS = BACKLIGHT_TIMEOUT_MS / 1000;
constexpr uint8_t SETTINGS_VERSION = 1;
constexpr uint8_t SETTINGS_MAIN_OUTPUT = 5;


// Standard definitions
// Ethernet
#define ETH_MOSI        31
#define ETH_MISO        28
#define ETH_SCK         30
#define ETH_CS          29
#define LINKLED         25
#define ACTLED          26
// LEDs
#define LED_PIN         21
#define LED_COUNT       10
// DMX
const int UNIVERSETX[4]   {39, 35, 43, 47};
const int UNIVERSEOE[4]   {38, 34, 42, 46};
// Display
#define TFT_CS          5
#define TFT_MOSI        3
#define TFT_SCK         2
#define TFT_RST         1
#define TFT_DC          6
#define TFT_BACKLIGHT   0

// define objects
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

repeating_timer_t interval_timer;
repeating_timer_t charger_timer;

#define UNIVERSE_LENGTH 512
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
};

enum class InterfaceMode : uint8_t {
    SelectOutput,
    EditUniverse,
    EditIp,
    SettingsMenu,
    EditSetting
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
uint8_t settings_category = 0;
uint8_t settings_menu_index = 0;
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

DmxOutput universes[4];

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

EthernetUDP artnet_receiver;
EthernetUDP sacn_receivers[4];
uint16_t sacn_receiver_universes[4] = {};
bool sacn_receiver_active[4] = {};

byte mac[] = { 0x02, 0x44, 0x4D, 0x58, 0x00, 0x01 };
IPAddress ip(192, 168, 2, 212);  // Set to an unused address on your Art-Net network

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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

bool sendDMX(__unused repeating_timer *rt) {
    for(int i = 0; i < 4; i++) {
        if(keep_sending[i].load(std::memory_order_relaxed) && !universes[i].busy()) {
            critical_section_enter_blocking(&frame_lock);
            const bool enabled = (enabled_outputs & (1u << i)) != 0;
            if(enabled) {
                std::memcpy(output_universedata[i], pending_universedata[i], sizeof(output_universedata[i]));
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
                  uint16_t subscribed_universe) {
    constexpr size_t root_layer_offset = 16;
    constexpr size_t framing_layer_offset = 38;
    constexpr size_t dmp_layer_offset = 115;
    constexpr size_t start_code_offset = 125;

    if(packet_size < start_code_offset + 1 ||
       packet[0] != 0x00 || packet[1] != 0x10 ||
       packet[2] != 0x00 || packet[3] != 0x00 ||
       std::memcmp(packet + 4, "ASC-E1.17\0\0\0", 12) != 0 ||
       !validSacnPduLength(packet, packet_size, root_layer_offset) ||
       readBigEndian32(packet + 18) != 0x00000004 ||
       !validSacnPduLength(packet, packet_size, framing_layer_offset) ||
       readBigEndian32(packet + 40) != 0x00000002 ||
       !validSacnPduLength(packet, packet_size, dmp_layer_offset) ||
       packet[117] != 0x02 || packet[118] != 0xA1 ||
       readBigEndian16(packet + 119) != 0 ||
       readBigEndian16(packet + 121) != 1 ||
       (packet[112] & 0xC0) != 0 ||
       packet[start_code_offset] != 0) {
        return;
    }

    const uint16_t property_value_count = readBigEndian16(packet + 123);
    if(property_value_count == 0 || property_value_count > UNIVERSE_LENGTH + 1 ||
       packet_size != start_code_offset + property_value_count) {
        return;
    }

    const uint16_t sacn_universe = readBigEndian16(packet + 113);
    if(sacn_universe == 0 || sacn_universe > 63999 ||
       sacn_universe != subscribed_universe) {
        return;
    }

    // The device's displayed universe is Art-Net's zero-based universe number.
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
                         sacn_receiver_universes[i]);
        }
    }
}

void onLED(uint gpio, uint32_t event_mask) {
    if(gpio == LINKLED) {
        link_led_on.store(event_mask == GPIO_IRQ_EDGE_FALL, std::memory_order_relaxed);
    } else if(gpio == ACTLED) {
        act_led_on.store(event_mask == GPIO_IRQ_EDGE_FALL, std::memory_order_relaxed);
    }
}

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
        keep_sending[i].store(output_enabled[i] && has_recent_packet, std::memory_order_relaxed);
        if(output_enabled[i]) {
            setStatusColor(7 - 2 * i, has_recent_packet ? 1 : 0);
        } else {
            strip.setPixelColor(7 - 2 * i, 0);
        }
        if(packet_activity[i]) {
            setStatusColor(6 - 2 * i, 2);
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
        setStatusColor(9, 4);
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
       config.lcd_timeout_seconds > 60) {
        return false;
    }

    for(uint8_t i = 0; i < 5; i++) {
        if(config.led_color_indices[i] >= STATUS_COLOR_COUNT) {
            return false;
        }
    }
    return true;
}

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
    if(selected_output == SETTINGS_MAIN_OUTPUT) {
        tft.fillRect(5, 218, 310, 20, 0x2104);
        tft.drawRect(5, 218, 310, 20, ST77XX_YELLOW);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(10, 224);
        tft.print(F("DEVICE SETTINGS"));
        tft.setCursor(190, 224);
        tft.print(F("HOLD TO OPEN"));
        return;
    }

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
        case 1: return 1;
        case 2: return 6;
        default: return 2;
    }
}

const char *settingsItemName(uint8_t category, uint8_t item) {
    switch(category) {
        case 0:
            return item == 0 ? "LCD brightness" : "LCD sleep timeout";
        case 1:
            return "Universe timeout";
        case 2:
            switch(item) {
                case 0: return "LED brightness";
                case 1: return "No signal color";
                case 2: return "Signal color";
                case 3: return "Packet activity";
                case 4: return "Ethernet link";
                default: return "Ethernet activity";
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
        default:
            return edited_setting_index == 0 ? led_brightness_percent :
                led_color_indices[edited_setting_index - 1];
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
        default:
            if(edited_setting_index == 0) {
                led_brightness_percent = value;
                applyLedBrightness();
            } else {
                led_color_indices[edited_setting_index - 1] = value;
            }
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
        } else if(settings_category == 2) {
            const uint8_t color_index = getSettingValue();
            tft.print(STATUS_COLORS[color_index].name);
            tft.fillRect(10, 126, 48, 28,
                         tft.color565(STATUS_COLORS[color_index].red,
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
        const uint8_t item_count = settings_root_menu ? 5 :
                                   settingsCategoryItemCount(settings_category) + 1;
        for(uint8_t item = 0; item < item_count; item++) {
            const int16_t y = 37 + item * 27;
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
                } else if(settings_category == 1) {
                    tft.print(universe_timeout_seconds);
                    tft.print(F(" sec"));
                } else if(settings_category == 2 && item == 0) {
                    tft.print(led_brightness_percent);
                    tft.print(F("%"));
                } else if(settings_category == 2) {
                    const uint8_t color_index = led_color_indices[item - 1];
                    tft.setTextColor(tft.color565(STATUS_COLORS[color_index].red,
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

void drawInterface(bool redraw_all) {
    if(interface_mode == InterfaceMode::SettingsMenu ||
       interface_mode == InterfaceMode::EditSetting) {
        drawSettingsInterface();
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
    tft.setCursor(190, 12);
    if(interface_mode == InterfaceMode::EditIp) {
        tft.print(F("CLICK NEXT / HOLD SAVE"));
    } else if(interface_mode == InterfaceMode::EditUniverse) {
        tft.print(F("CLICK: SAVE DMX U"));
    } else if(selected_output == SETTINGS_MAIN_OUTPUT) {
        tft.print(F("HOLD: DEVICE SETTINGS"));
    } else {
        tft.print(selected_output == 4 ? F("CLICK DHCP / HOLD IP") : F("HOLD: EDIT DMX U"));
    }
    tft.drawFastHLine(5, 23, 310, ST77XX_CYAN);

    drawNetworkStatus();
    for(uint8_t i = 0; i < 4; i++) {
        drawOutputRow(i);
    }
    drawIpRow();
}

void recordUserActivity() {
    last_user_activity = millis();
    if(!backlight_is_on) {
        backlight_is_on = true;
        applyLcdBrightness();
        drawInterface(true);
    }
}

void adjustSettingValue(int8_t direction);

void handleEncoderRotation(int8_t direction) {
    recordUserActivity();
    if(interface_mode == InterfaceMode::EditUniverse) {
        if(direction > 0) {
            edited_universe = edited_universe == MAX_ARTNET_UNIVERSE ? 0 : edited_universe + 1;
        } else {
            edited_universe = edited_universe == 0 ? MAX_ARTNET_UNIVERSE : edited_universe - 1;
        }
        drawOutputRow(selected_output);
        return;
    }
    if(interface_mode == InterfaceMode::EditIp) {
        edited_ip[edited_ip_octet] = static_cast<uint8_t>(edited_ip[edited_ip_octet] + direction);
        drawIpRow();
        return;
    }
    if(interface_mode == InterfaceMode::EditSetting) {
        adjustSettingValue(direction);
        drawInterface(false);
        return;
    }
    if(interface_mode == InterfaceMode::SettingsMenu) {
        const uint8_t item_count = settings_root_menu ? 5 :
                                   settingsCategoryItemCount(settings_category) + 1;
        settings_menu_index = static_cast<uint8_t>(
            (settings_menu_index + item_count + direction) % item_count);
        drawInterface(false);
        return;
    }

    selected_output = static_cast<uint8_t>(
        (selected_output + 6 + direction) % 6);
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
    keep_sending[selected_output].store(false, std::memory_order_relaxed);
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
    settings_root_menu = false;
    drawInterface(true);
}

void selectSettingsMenuItem() {
    if(settings_root_menu) {
        if(settings_menu_index < 4) {
            openSettingsCategory(settings_menu_index);
        } else {
            exitSettings();
        }
        return;
    }

    const uint8_t item_count = settingsCategoryItemCount(settings_category);
    if(settings_menu_index == item_count) {
        settings_root_menu = true;
        settings_menu_index = settings_category;
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

void adjustSettingValue(int8_t direction) {
    uint8_t value = getSettingValue();
    if(settings_category == 2 && edited_setting_index > 0) {
        value = static_cast<uint8_t>((value + STATUS_COLOR_COUNT + direction) %
                                     STATUS_COLOR_COUNT);
    } else {
        const uint8_t minimum =
            settings_category == 0 && edited_setting_index == 0 ? 10 :
            (settings_category == 2 && edited_setting_index == 0 ? 0 : 1);
        const uint8_t maximum =
            settings_category == 0 && edited_setting_index == 0 ? 100 :
            (settings_category == 2 ? 100 : 60);
        if(direction > 0 && value < maximum) {
            value++;
        } else if(direction < 0 && value > minimum) {
            value--;
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

void handleEncoder() {
    static constexpr int8_t transitions[16] = {
        0, 1, -1, 0, -1, 0, 0, 1,
        1, 0, 0, -1, 0, -1, 1, 0
    };
    static uint8_t previous_ab = 0;
    static int8_t encoder_steps = 0;
    static bool encoder_initialized = false;
    static bool raw_button_state = HIGH;
    static bool stable_button_state = HIGH;
    static InterfaceMode button_mode_at_press = InterfaceMode::SelectOutput;
    static bool long_press_entered_edit = false;
    static uint32_t raw_state_changed_at = 0;
    static uint32_t button_pressed_at = 0;

    const uint8_t current_ab = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
    if(!encoder_initialized) {
        previous_ab = current_ab;
        encoder_initialized = true;
    } else {
        encoder_steps += transitions[(previous_ab << 2) | current_ab];
        previous_ab = current_ab;
        if(encoder_steps >= 2) {
            handleEncoderRotation(-1);
            encoder_steps = 0;
        } else if(encoder_steps <= -2) {
            handleEncoderRotation(1);
            encoder_steps = 0;
        }
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
            } else if(button_mode_at_press == InterfaceMode::SettingsMenu &&
                      !long_press_entered_edit) {
                selectSettingsMenuItem();
            } else if(!long_press_entered_edit) {
                if(selected_output == 4) {
                    toggleDhcp();
                } else if(selected_output == SETTINGS_MAIN_OUTPUT) {
                    // Settings are opened by a long press.
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

void playStartupRainbow() {
    constexpr uint8_t frames = 120;
    constexpr uint16_t frame_delay_ms = 20;

    for(uint8_t frame = 0; frame < frames; frame++) {
        for(uint8_t pixel = 0; pixel < LED_COUNT; pixel++) {
            const uint16_t hue = static_cast<uint16_t>(
                (static_cast<uint32_t>(frame) * 65536 / frames) +
                (static_cast<uint32_t>(pixel) * 65536 / LED_COUNT));
            strip.setPixelColor(pixel, strip.ColorHSV(hue, 255, 100));
        }
        strip.show();
        delay(frame_delay_ms);
    }
    strip.clear();
    strip.show();
}

void setup() {
    Serial.begin(115200);
    critical_section_init(&frame_lock);
    loadConfiguration();
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_SWITCH, INPUT_PULLUP);
    last_user_activity = millis();

    // Give time to open the serial monitor
    delay(1000);

    // Start the charger
    if(!chargerStart(McciCatenaNpm1300::cNPM1300::Ntc::NotFitted, McciCatenaNpm1300::cNPM1300::Vterm::V4_20, McciCatenaNpm1300::cNPM1300::Current::MA_1500, 800)) {
        while(1) {
            Serial.println("Charger init failed");
            delay(20);
        }
    } else {
        Serial.println("Charger initialized succesfully");
    }
    add_repeating_timer_ms(-5000, sendChargerData, NULL, &charger_timer);

    // Configure the LEDs
    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    applyLedBrightness();
    // Configure the display
    delay(500);
    SPI.setCS(TFT_CS);
    SPI.setTX(TFT_MOSI);
    SPI.setSCK(TFT_SCK);
    pinMode(TFT_BACKLIGHT, OUTPUT);
    analogWrite(TFT_BACKLIGHT, 0);
    tft.init(240, 320);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
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

    playStartupRainbow();
    drawInterface(true);
}

void loop() {
    handleEncoder();
    updateBacklight();
    updateBatteryMeasurement();
    updateStatusLeds();
    strip.show();

    const uint32_t now = millis();
    if(backlight_is_on &&
       interface_mode != InterfaceMode::SettingsMenu &&
       interface_mode != InterfaceMode::EditSetting &&
       static_cast<uint32_t>(now - last_interface_stats_update) >= 500) {
        drawNetworkStatus();
        drawIpRow();
        for(uint8_t i = 0; i < 4; i++) {
            drawOutputStats(i);
        }
        last_interface_stats_update = now;
    }
}

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