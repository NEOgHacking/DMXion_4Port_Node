#pragma once

#include <MCCI_Catena_nPM1300.h>

bool chargerStart(
    McciCatenaNpm1300::cNPM1300::Ntc ntc_fitted,
    McciCatenaNpm1300::cNPM1300::Vterm charge_voltage,
    McciCatenaNpm1300::cNPM1300::Current max_current,
    int charge_current
);

bool getVbat(float &voltage);
bool getChargeStatus(uint8_t &status);