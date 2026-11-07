#include <Arduino.h>
#include <Charger.h>
#include <ArtnetEther.h>
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
constexpr uint32_t CONFIG_MAGIC = 0x444D5831;
constexpr uint32_t BACKLIGHT_TIMEOUT_MS = 60000;
constexpr uint32_t ENCODER_HOLD_MS = 650;
constexpr uint8_t ENCODER_A = 8;
constexpr uint8_t ENCODER_B = 9;
constexpr uint8_t ENCODER_SWITCH = 7;


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
};

enum class InterfaceMode : uint8_t {
    SelectOutput,
    EditUniverse,
    EditIp
};

InterfaceMode interface_mode = InterfaceMode::SelectOutput;
uint8_t selected_output = 0;
uint16_t edited_universe = 0;
uint8_t edited_ip[4] = {};
uint8_t edited_ip_octet = 0;
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

ArtnetEtherReceiver artnet;

byte mac[] = { 0x02, 0x44, 0x4D, 0x58, 0x00, 0x01 };
IPAddress ip(192, 168, 2, 212);  // Set to an unused address on your Art-Net network

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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

void onArtNET(const uint8_t *data, uint16_t size, const art_net::art_dmx::Metadata &metadata, __unused const art_net::RemoteInfo &remote) {
    const uint16_t data_size = size < UNIVERSE_LENGTH ? size : UNIVERSE_LENGTH;
    const uint32_t received_at = millis();
    const uint16_t received_universe =
        (static_cast<uint16_t>(metadata.net) << 8) |
        (static_cast<uint16_t>(metadata.subnet) << 4) |
        metadata.universe;

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
        const bool has_recent_packet =
            has_received[i] && static_cast<uint32_t>(now - packet_times[i]) <= 10000;
        keep_sending[i].store(output_enabled[i] && has_recent_packet, std::memory_order_relaxed);
        strip.setPixelColor((7 - 2 * i),
                            output_enabled[i] && !has_recent_packet ? 255 : 0,
                            output_enabled[i] && has_recent_packet ? 255 : 0,
                            0);
        strip.setPixelColor((6 - 2 * i), 0, 0, packet_activity[i] ? 255 : 0);
    }
    const bool link_on = link_led_on.load(std::memory_order_relaxed);
    const bool act_on = act_led_on.load(std::memory_order_relaxed);
    strip.setPixelColor(8, link_on ? 255 : 0, link_on ? 255 : 0, link_on ? 255 : 0);
    strip.setPixelColor(9, act_on ? 255 : 0, act_on ? 255 : 0, act_on ? 255 : 0);
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
        tft.print(edited_universe);
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
    const bool packet_stale = !has_received || age >= 10000;
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

void drawInterface(bool redraw_all) {
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
        digitalWrite(TFT_BACKLIGHT, HIGH);
        backlight_is_on = true;
        drawInterface(true);
    }
}

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

    selected_output = static_cast<uint8_t>((selected_output + 5 + direction) % 5);
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
    interface_mode = InterfaceMode::SelectOutput;
    configuration_saved = saved;
    drawInterface(false);
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
            } else if(button_mode_at_press == InterfaceMode::EditIp) {
                edited_ip_octet = static_cast<uint8_t>((edited_ip_octet + 1) % 4);
                drawIpRow();
            } else if(!long_press_entered_edit) {
                if(selected_output == 4) {
                    toggleDhcp();
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
        if(button_mode_at_press != InterfaceMode::SelectOutput) {
            return;
        }

        if(selected_output == 4) {
            interface_mode = InterfaceMode::EditIp;
            edited_ip[0] = configured_ip[0];
            edited_ip[1] = configured_ip[1];
            edited_ip[2] = configured_ip[2];
            edited_ip[3] = configured_ip[3];
            edited_ip_octet = 0;
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
       static_cast<uint32_t>(millis() - last_user_activity) >= BACKLIGHT_TIMEOUT_MS) {
        digitalWrite(TFT_BACKLIGHT, LOW);
        backlight_is_on = false;
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
    strip.setBrightness(100);
    // Configure the display
    delay(500);
    SPI.setCS(TFT_CS);
    SPI.setTX(TFT_MOSI);
    SPI.setSCK(TFT_SCK);
    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, LOW);
    tft.init(240, 320);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    digitalWrite(TFT_BACKLIGHT, HIGH);
    backlight_is_on = true;

    // Configure the DMX
    for(int i = 0; i < 4; i++) {
        pinMode(UNIVERSEOE[i], OUTPUT);
        digitalWrite(UNIVERSEOE[i], HIGH);
        if(i < 2) {
            universes[i].begin(UNIVERSETX[i], pio1);
        } else {
            universes[i].begin(UNIVERSETX[i], pio2);
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
        // Start the artnet receiver on core 1.
        artnet.begin();
        artnet.subscribeArtDmx(onArtNET);
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
        artnet.begin();
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
        artnet.parse();
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