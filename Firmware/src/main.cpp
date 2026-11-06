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

// Configurables
#define DMX_SPEED 30 // In hertz
int     UNIVERSE_RECIEVING[4] = {0, 1, 2, 3};


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
bool keep_sending[4] = {true, true, true, true};

repeating_timer_t interval_timer;
repeating_timer_t charger_timer;
repeating_timer_t timeout_timer;

#define UNIVERSE_LENGTH 512
uint8_t universedata[4][UNIVERSE_LENGTH + 1];

DmxOutput universes[4];

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

ArtnetEtherReceiver artnet;

byte mac[] = { 0x02, 0x44, 0x4D, 0x58, 0x00, 0x01 };
IPAddress ip(192, 168, 2, 212);  // Set to an unused address on your Art-Net network

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool sendDMX(__unused repeating_timer *rt) {
    for(int i = 0; i < 4; i++) {
        if(keep_sending[i] && !universes[i].busy()) {
            universes[i].write(universedata[i], 513);
        }
    }
    return true;
}

bool checkDMX(__unused repeating_timer *rt) {
    for(int i = 0; i < 4; i++) {
        if(millis() > last_dmx_packet[i] + 10000) {
            keep_sending[i] = false;
            strip.setPixelColor((7 -2 * i), 255, 0, 0);
        } else {
            keep_sending[i] = true;
            strip.setPixelColor((7 -2 * i), 0, 255, 0);
        }
    }
    return true;
}

bool sendChargerData(__unused repeating_timer *rt) {
    Serial.println(getVbat());
    return true;
}

long long actoff(__unused alarm_id_t id, void *user_data) {
    strip.setPixelColor((int)user_data, 0, 0, 0);
    return 0;
}

void onArtNET(const uint8_t *data, uint16_t size, const art_net::art_dmx::Metadata &metadata, __unused const art_net::RemoteInfo &remote) {
    for(int i = 0; i < 4; i++) {
        if(metadata.universe == UNIVERSE_RECIEVING[i]) {
            universedata[i][0] = 0;
            for(int j = 0; j < size; j++) {
                universedata[i][j + 1] = data[j];
            }
            last_dmx_packet[i] = millis();
            strip.setPixelColor((6 - 2 * i), 0, 0, 255);
            add_alarm_in_ms(20, actoff, (void*)(6 - 2 * i), true);
        }
    }
}

void onLED(uint gpio, uint32_t event_mask) {
    if(gpio == LINKLED && event_mask == GPIO_IRQ_EDGE_FALL) {
        strip.setPixelColor(8, 255, 255, 255);
    } else if(gpio == LINKLED) {
        strip.setPixelColor(8, 0, 0, 0);
    }
    if(gpio == ACTLED && event_mask == GPIO_IRQ_EDGE_FALL) {
        strip.setPixelColor(9, 255, 255, 255);
    } else if(gpio == ACTLED) {
        strip.setPixelColor(9, 0, 0, 0);
    }
}

void setup() {
    Serial.begin(115200);

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
    add_repeating_timer_ms(-1000, sendChargerData, NULL, &charger_timer);

    // Configure the LEDs
    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.setBrightness(100);

    // Configure the display
    delay(500);
    SPI.setCS(TFT_CS);
    SPI.setTX(TFT_MOSI);
    SPI.setSCK(TFT_SCK);
    pinMode(TFT_BACKLIGHT, OUTPUT);
    pinMode(TFT_BACKLIGHT, LOW);
    tft.init(240, 320);
    tft.fillScreen(ST77XX_BLACK);
    delay(500);
    pinMode(TFT_BACKLIGHT, HIGH);

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
    Ethernet.begin(mac, ip);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("W5500 not detected");
        return;
    }

    // Start the artnet
    artnet.begin();
    artnet.subscribeArtDmx(onArtNET);

    add_repeating_timer_ms(-100, checkDMX, NULL, &timeout_timer);
}

void loop() {
    artnet.parse();
    strip.show();
}