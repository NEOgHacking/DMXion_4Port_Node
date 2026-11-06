# nPM1300 Simple Example

This is a simple example sketch demonstrating the use of the `MCCI_Catena_nPM1300` library to interface with the nPM1300 Power Management IC. It initializes the PMIC, configures basic settings, and periodically prints system and battery metrics to the Serial Monitor. It requires no additional libraries.

## Features

- Configures and initializes the nPM1300 PMIC.
- Measures and displays:
  - Battery Voltage (VBAT)
  - System Voltage (VSYS)
  - Bus Voltage (VBUS)
  - Die Temperature
  - Bus Voltage Status
- Configures the battery termination voltage and charging current.

## Prerequisites

- Arduino IDE installed.
- `MCCI_Catena_nPM1300` library added to your Arduino libraries folder.
- An nPM1300 module connected to your Arduino-compatible board.
- Ensure a pull-up resistor is present on the I2C lines if your hardware does not provide them.

## Code Description

### Setup
1. **I2C Initialization**: Initializes the `Wire` library for I2C communication.
2. **Serial Monitor**: Initializes the serial communication at 9600 baud and waits for USB attachment.
3. **nPM1300 Initialization**:
   - Configures the battery termination voltage to 4.15V.
   - Ignores NTC monitoring if no NTC is connected.
   - Sets the charge current to 400 mA.

### Loop
The sketch continuously measures and prints the following metrics to the Serial Monitor every second:
- Battery Voltage (`measureVbat()`)
- Die Temperature (`measureTemperature()`)
- System Voltage (`measureVsys()`)
- Bus Voltage (`measureVbus()`)
- Bus Voltage Status (`getVbusStatus()`)

### Example Output
```
nPM1300 Simple Test
Battery Voltage: 4.020
Temperature: 25.5
System Voltage: 3.300
Bus Voltage: 5.000
Bus Voltage Status: 1

##################################
```

## Sketch Code

The sketch in this directory (found [here](https://github.com/mcci-catena/MCCI-Catena-nPM1300/examples/npm1300_simple) on GitHub)

## Notes

- Ensure the nPM1300 module is correctly connected and powered.
- Pass `cNPM1300::Ntc::NotFitted` to `begin()` if no NTC thermistor is connected, `Ntc::Fitted` if one is.

## Meta

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [store.mcci.com](https://store.mcci.com/).

### Trademarks

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. All other marks are the property of their respective owners.