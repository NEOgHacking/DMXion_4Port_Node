#include <Charger.h>

#include <Wire.h>
#include <MCCI_Catena_nPM1300.h>


using namespace McciCatenaNpm1300;


McciCatenaNpm1300::cNPM1300 gNpm1300{Wire1};

// Initializes the charger and sets different settings as specified
bool chargerStart(McciCatenaNpm1300::cNPM1300::Ntc ntc_fitted, McciCatenaNpm1300::cNPM1300::Vterm charge_voltage, McciCatenaNpm1300::cNPM1300::Current max_current, int charge_current) {

    // Setup the I2C line
    Wire1.setSCL(23);
    Wire1.setSDA(22);
    Wire1.begin();

    // Start and configure the charger
    if(!gNpm1300.begin(ntc_fitted) ||
        !gNpm1300.setVterm(charge_voltage) ||
        !gNpm1300.chargerOff() ||
        !gNpm1300.suspendVbus(false) ||
        !gNpm1300.setInCurrentLimitVbus(max_current) ||
        !gNpm1300.setTaskUpdateILimit(true) ||
        !gNpm1300.setBuckVoltage_2(McciCatenaNpm1300::cNPM1300::Voltage::V3_30) ||
        !gNpm1300.setBuck_2(true) ||
        !gNpm1300.setBuck_1(false) ||
        !gNpm1300.ledSetMode(McciCatenaNpm1300::cNPM1300::Led::LED_0, McciCatenaNpm1300::cNPM1300::LedMode::Charging) ||
        !gNpm1300.ledSetMode(McciCatenaNpm1300::cNPM1300::Led::LED_1, McciCatenaNpm1300::cNPM1300::LedMode::Error) ||
        !gNpm1300.ledSetMode(McciCatenaNpm1300::cNPM1300::Led::LED_2, McciCatenaNpm1300::cNPM1300::LedMode::Host) ||
        !gNpm1300.setChargeCurrent(charge_current) ||
        !gNpm1300.chargerOn()
    ) {
        return false;
    } else {
        return true;
    }
}

bool getVbat(float &voltage) {
    return gNpm1300.measureVbat(voltage);
}

bool getChargeStatus(uint8_t &status) {
    return gNpm1300.getChargeStatus(status);
}