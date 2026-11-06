/*

Module: npm1300_simple.ino

Function:
        Simple example nPM1300 PMIC module.

Copyright and License:
        See accompanying LICENSE file.

Author:
        Pranau Ravikumar, MCCI Corporation   June 2025

*/

#include <Arduino.h>
#include <Wire.h>
#include <MCCI_Catena_nPM1300.h>

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
\****************************************************************************/

using namespace McciCatenaNpm1300;

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/

McciCatenaNpm1300::cNPM1300 gNpm1300 {Wire};

/****************************************************************************\
|
|   Code.
|
\****************************************************************************/

void setup()
    {
    Wire.begin();
    Serial.begin(9600);

    // wait for USB to be attached.
    while (! Serial)
        yield();

    Serial.println("nPM1300 Simple Test");

    // this example assumes a board with no NTC thermistor.
    if (!gNpm1300.begin(cNPM1300::Ntc::NotFitted))
        {
        Serial.println("Failed to initialize nPM1300!");
        delay(1000);
        while (true);
        }

    gNpm1300.setVterm(cNPM1300::Vterm::V4_15);
    gNpm1300.setChargeCurrent(400);
    }

void loop()
    {
    gNpm1300.setVterm(cNPM1300::Vterm::V4_15);

    float vBat;
    if (gNpm1300.measureVbat(vBat))
        {
        Serial.print("Battery Voltage: ");
        Serial.println(vBat, 3);
        }
    else
        {
        Serial.print("measureVbat() failed: ");
        Serial.println(gNpm1300.getLastErrorName());
        }

    float temperature;
    if (gNpm1300.measureTemperature(temperature))
        {
        Serial.print("Temperature: ");
        Serial.println(temperature);
        }
    else
        {
        Serial.print("measureTemperature() failed: ");
        Serial.println(gNpm1300.getLastErrorName());
        }

    float vSys;
    if (gNpm1300.measureVsys(vSys))
        {
        Serial.print("System Voltage: ");
        Serial.println(vSys);
        }
    else
        {
        Serial.print("measureVsys() failed: ");
        Serial.println(gNpm1300.getLastErrorName());
        }

    float vBus;
    if (gNpm1300.measureVbus(vBus))
        {
        Serial.print("Bus Voltage: ");
        Serial.println(vBus);
        }
    else
        {
        Serial.print("measureVbus() failed: ");
        Serial.println(gNpm1300.getLastErrorName());
        }

    uint8_t vBusStatus;
    if (gNpm1300.getVbusStatus(vBusStatus))
        {
        Serial.print("Bus Status: ");
        Serial.println(vBusStatus);
        }
    else
        {
        Serial.print("getVbusStatus() failed: ");
        Serial.println(gNpm1300.getLastErrorName());
        }

    Serial.println();
    Serial.println("##################################");
    Serial.println();
    delay (1000);
    }