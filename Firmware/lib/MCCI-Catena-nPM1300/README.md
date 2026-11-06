# MCCI-Catena-nPM1300 Library

The `MCCI_Catena_nPM1300` library provides an interface to the nPM1300 power management IC, allowing users to control and monitor various features such as voltage regulation, load switches, battery charging, GPIO configuration, and LED control. This library is designed for Arduino-compatible platforms and simplifies communication with the nPM1300 via the I2C protocol.

## Features

- Control buck regulators (enable/disable, set output voltage).
- Configure load switches and LDO regulators.
- Measure system parameters like:
  - VBAT (Battery Voltage)
  - VBUS (Input Voltage)
  - VSYS (System Voltage)
  - Temperature
- Control LED modes and states.
- Manage battery charger settings, including current limits and termination voltages.
- Configure GPIOs:
  - Set GPIO modes.
  - Configure drive strength, pull-up, pull-down, and open-drain.
  - Enable/disable debounce functionality.
  - Retrieve GPIO status.
- Comprehensive error handling and state management.

## Installation

1. Download or clone this repository.
2. Copy the `MCCI_Catena_nPM1300` folder into your Arduino `libraries` directory.
3. Restart the Arduino IDE to recognize the library.

## Getting Started

### Include the Library
Add the library to your sketch:
```cpp
#include <MCCI_Catena_nPM1300.h>
```

### Initialize the Device
```cpp
#include <Wire.h>
#include <MCCI_Catena_nPM1300.h>

McciCatenaNpm1300::cNPM1300 npm1300 {Wire};

void setup() {
    Wire.begin();
    if (npm1300.begin(cNPM1300::Ntc::NotFitted)) {
        Serial.println("nPM1300 initialized successfully.");
    } else {
        Serial.print("Failed to initialize nPM1300: ");
        Serial.println(npm1300.getLastErrorName());
    }
}

void loop() {
    // Example logic here
}
```

## API Reference

### Initialization and State
- **`bool begin(cNPM1300::Ntc ntc)`**: Initialize the nPM1300 device. `ntc` says whether the board has a battery NTC thermistor (`Ntc::Fitted` or `Ntc::NotFitted`); there is no default, because on a board without one the charger refuses to charge unless told to ignore the NTC limits. `begin()` applies the setting.
- **`void end()`**: Terminate the device operation.
- **`bool reset()`**: Perform a soft reset on the nPM1300.

### Voltage and Current Measurement
- **`bool measureVbat(float &v)`**: Measure battery voltage. Returns `false` (and leaves `v` unchanged) if the conversion times out or the read fails.
- **`bool measureVbus(float &v)`**: Measure input voltage. Returns `false` (and leaves `v` unchanged) if the conversion times out or the read fails.
- **`bool measureVsys(float &v)`**: Measure system voltage. Returns `false` (and leaves `v` unchanged) if the conversion times out or the read fails.
- **`bool measureTemperature(float &t)`**: Measure die temperature. Returns `false` (and leaves `t` unchanged) if the conversion times out or the read fails.
- **`bool getVbusStatus(uint8_t &status)`**: Get the Vbus status register. Returns `false` (and leaves `status` unchanged) if the read fails.
- **`bool getVbusComparatorStatus(uint8_t &status)`**: Get the Vbus comparator status register. Returns `false` (and leaves `status` unchanged) if the read fails.

### Voltage Regulators
- **`bool setBuck_1(bool state)`**: Enable or disable Buck 1.
- **`bool setBuck_2(bool state)`**: Enable or disable Buck 2.
- **`bool setBuckVoltage_1(Voltage voltage)`**: Set output voltage for Buck 1.
- **`bool setBuckVoltage_2(Voltage voltage)`**: Set output voltage for Buck 2.
- **`bool getBuckStatus(uint8_t &status)`**: Get the Buck 1/Buck 2 enable status register. Returns `false` (and leaves `status` unchanged) if the read fails.
- **`bool getLoadSwitchStatus(uint8_t &status)`**: Get the load switch status register (LDSWSTATUS). Returns `false` (and leaves `status` unchanged) if the read fails.
- **`bool enterShipMode()`**: Enter ship mode: all rails off, so on success the host loses power. With VBUS present, the nPM1300 waits for VBUS to go. VBUS above VBUSPOR, or SHPHLD held low, wakes it, and the host starts from a power-on reset.

### Load Switch and LDO
- **`bool setLoadSwitch_1(bool state)`**: Control Load Switch 1.
- **`bool setLoadSwitch_2(bool state)`**: Control Load Switch 2.
- **`bool selectLdoLoadSwitchMode_1(bool mode)`**: Select mode between LDO and Load Switch for LDO 1.
- **`bool selectLdoLoadSwitchMode_2(bool mode)`**: Select mode between LDO and Load Switch for LDO 2.
- **`bool setLdoVoltage_1(Voltage voltage)`**: Set voltage for LDO 1.
- **`bool setLdoVoltage_2(Voltage voltage)`**: Set voltage for LDO 2.

### LEDs
- **`bool ledSetMode(Led led, LedMode mode)`**: Set LED mode.
- **`bool ledOn(Led led)`**: Turn on an LED.
- **`bool ledOff(Led led)`**: Turn off an LED.
- **`bool ledToggle(Led led)`**: Toggle the LED state.

### GPIO Configuration
- **`bool setGpioMode(GpioMode mode, uint8_t gpioNumber)`**: Configure GPIO mode.
- **`bool setGpioDrive(GpioDrive current, uint8_t gpioNumber)`**: Set GPIO drive strength.
- **`bool setGpioPullUp(bool enable, uint8_t gpioNumber)`**: Enable or disable pull-up resistor for GPIO.
- **`bool setGpioPullDown(bool enable, uint8_t gpioNumber)`**: Enable or disable pull-down resistor for GPIO.
- **`bool setGpioOpenDrain(bool enable, uint8_t gpioNumber)`**: Enable or disable open-drain mode for GPIO.
- **`bool setGpioDebounce(bool enable, uint8_t gpioNumber)`**: Enable or disable debounce for GPIO.
- **`bool getGpioStatus(uint8_t &status)`**: Retrieve the current status of GPIOs. Returns `false` (and leaves `status` unchanged) if the read fails.

### Buck Regulator GPIO Control
- **`bool setBuckGpioControl(uint8_t buckNumber, uint8_t gpioNumber, bool invert)`**: Configure GPIO for Buck regulator control.

### Load Switch GPIO Control
- **`bool setLoadSwitchGpioControl_1(uint8_t gpioNumber, bool invert)`**: Configure GPIO control for Load Switch 1.
- **`bool setLoadSwitchGpioControl_2(uint8_t gpioNumber, bool invert)`**: Configure GPIO control for Load Switch 2.

### Battery Charger
- **`bool chargerOn()`**: Apply the NTC setting given to `begin()`, then enable the charger.
- **`bool chargerOff()`**: Disable the charger.
- **`cNPM1300::Ntc getNtc() const`**: Return the NTC setting given to `begin()`.
- **`bool enableRecharge(bool enable)`**: Enable or disable recharging of the battery once charged.
- **`bool setChargeCurrent(uint16_t limit)`**: Set charge current (32-800 mA, in 2 mA steps). An enabled charger is disabled while the registers change and then enabled again, so the new value applies at once.
- **`bool getVterm(uint8_t &vTerm)`**: Get the battery charger's normal termination voltage. Returns `false` (and leaves `vTerm` unchanged) if the read fails.
- **`bool setVterm(Vterm vTerm)`**: Set the battery charger's normal termination voltage.
- **`bool getChargeStatus(uint8_t &status)`**: Get the charging status. Returns `false` (and leaves `status` unchanged) if the read fails.
- **`bool getErrorReason(uint8_t &reason)`**: Retrieve the last charger error reason. Returns `false` (and leaves `reason` unchanged) if the read fails.
- **`bool getErrorSensor(uint8_t &sensor)`**: Retrieve the last charger error related to an external sensor. Returns `false` (and leaves `sensor` unchanged) if the read fails.
- **`bool suspendVbus(bool mode)`**: Suspend or resume drawing current from Vbus.
- **`bool setInCurrentLimitVbus(Current limit)`**: Set the Vbus input current limit.
- **`bool readInCurrentLimitVbus(uint8_t &limit)`**: Read back the Vbus input current limit. Returns `false` (and leaves `limit` unchanged) if the read fails.
- **`bool setStartupInCurrentLimitVbus(Current limit)`**: Set the Vbus startup input current limit.
- **`bool readStartupInCurrentLimitVbus(uint8_t &limit)`**: Read back the Vbus startup input current limit. Returns `false` (and leaves `limit` unchanged) if the read fails.

### Error Handling
- **`Error getLastError()`**: Retrieve the last error code.
- **`const char* getLastErrorName()`**: Get a human-readable error message.

## Example

### Measure VBAT and Toggle an LED
```cpp
#include <Wire.h>
#include <MCCI_Catena_nPM1300.h>

McciCatenaNpm1300::cNPM1300 npm1300 {Wire};

void setup() {
    Wire.begin();
    Serial.begin(9600);

    if (!npm1300.begin(cNPM1300::Ntc::NotFitted)) {
        Serial.print("Failed to initialize nPM1300: ");
        Serial.println(npm1300.getLastErrorName());
        while (true);
    }

    float vbat;
    if (npm1300.measureVbat(vbat)) {
        Serial.print("Battery Voltage: ");
        Serial.println(vbat, 2);
    } else {
        Serial.print("measureVbat() failed: ");
        Serial.println(npm1300.getLastErrorName());
    }

    if (npm1300.ledToggle(McciCatenaNpm1300::cNPM1300::Led::LED_0)) {
        Serial.println("LED toggled successfully.");
    } else {
        Serial.println("Failed to toggle LED.");
    }
}

void loop() {
    // Add your logic
}
```

## Supported Devices
- nPM1300 Power Management IC

## Meta

### License

This repository is released under the [MIT](./LICENSE.md) license. Commercial licenses are also available from MCCI Corporation.

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [store.mcci.com](https://store.mcci.com/).

### Trademarks

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. All other marks are the property of their respective owners.