/*

Module: MCCI_Catena_nPM1300.cpp

Function:
    Implementation code for MCCI Catena nPM1300 power management IC library.

Copyright and License:
    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau R, MCCI Corporation   May 2025

*/

#include "MCCI_Catena_nPM1300.h"
#include <Arduino.h>
#include <Wire.h>

using namespace McciCatenaNpm1300;

cNPM1300::cNPM1300()
    {
    //default constructor
    }

cNPM1300::~cNPM1300()
    {
    }

bool cNPM1300::begin(Ntc ntc)
    {
    this->m_ntc = ntc;

    // if no Wire is bound, fail.
    if (this->m_wire == nullptr)
        {
        this->setLastError(Error::NoWire);
        return false;
        }

    if (this->isRunning())
        {
        return true;
        }

    this->m_wire->begin();

    // probe the chip by reading the Buck 1/Buck 2 status register; only
    // declare the driver running if the chip actually answers. Do not
    // set State::Initial until we know the probe succeeded, so a failed
    // begin() leaves isRunning() false and a later begin() retries for
    // real instead of taking the early return above.
    uint8_t buckStatus;
    if (! this->getBuckStatus(buckStatus))
        return false;

    this->setState(State::Initial);

    // a board with no thermistor must have the NTC limits ignored before
    // anyone turns the charger on.
    return this->applyNtc();
    }

void cNPM1300::end()
    {
    if (this->isRunning())
        {
        this->setState(State::End);
        }
    }

bool cNPM1300::reset()
    {
    if (this->writeRegister(Register::TaskSwReset, (uint8_t)Command::Trigger))
        {
        return true;
        }
    else
        {
        this->setState(State::Uninitialized);
        return false;
        }
    }

// protected
bool cNPM1300::waitForAdcReady(std::uint8_t eventBit)
    {
    // tCONV (ADC conversion time) is 250us typical (datasheet section
    // 7.1.9, Table 29). This timeout is a generous margin over that,
    // not a tight bound: each poll is an I2C transfer in its own right,
    // so the loop below settles as soon as the bit is actually set.
    constexpr std::uint32_t kTimeoutMs = 10;

    const std::uint32_t tStart = millis();
    std::uint8_t status;

    do  {
        if (! this->readRegister(Register::EventsAdcSet, status))
            return false;

        if (status & eventBit)
            {
            // clear the event so the next conversion starts clean.
            return this->writeRegister(Register::EventsAdcClr, eventBit);
            }
        } while (std::uint32_t(millis() - tStart) < kTimeoutMs);

    return this->setLastError(Error::NotMeasuring);
    }

bool cNPM1300::measureVbat(float &voltage)
    {
    // EVENTADCVBATRDY: EVENTSADCSET/EVENTSADCCLR field A (bit 0). See
    // datasheet section 7.7.1.2.
    constexpr std::uint8_t kVbatReadyBit = 0x01;

    uint8_t vBatMsb;
    uint8_t vBatLsb;

    if (! (this->writeRegister(Register::TaskVbatMeasure, (uint8_t)Command::Trigger) &&
           this->waitForAdcReady(kVbatReadyBit) &&
           this->readRegister(Register::AdcVbatResultMsb, vBatMsb) &&
           this->readRegister(Register::AdcGpResultLsbs_0, vBatLsb)))
        return false;

    // ADCGP0RESULTLSBS field A (bits 1:0) holds the VBAT measurement's
    // LSBs. See datasheet section 7.1.10.18, ADCGP0RESULTLSBS.
    constexpr uint8_t kVbatLsbMask = 0x03;
    vBatLsb = (vBatLsb & kVbatLsbMask);

    // Combine the 8-bit MSB and 2-bit LSB into the full 10-bit ADC
    // result. See datasheet section 7.1.4, Table 28 "Measurement
    // results".
    uint16_t result = (vBatMsb << 2) | vBatLsb;

    // VBAT = (ADC result / full-scale ADC code) * VFSVBAT. The ADC is a
    // 10-bit converter, and VFSVBAT (the full-scale voltage for the
    // VBAT channel) is 5.0V. See datasheet section 7.1.4 (the "VBAT"
    // equation) and section 7.1.9, Table 29 (Electrical specification),
    // symbol VFSVBAT.
    constexpr float kAdcMaxCode10Bit = 1023.0f;
    constexpr float kVfsVbat = 5.0f;
    voltage = (result / kAdcMaxCode10Bit) * kVfsVbat;
    return true;
    }

bool cNPM1300::measureVbus(float &voltage)
    {
    // EVENTADCVBUS7V0RDY: EVENTSADCSET/EVENTSADCCLR field H (bit 7). See
    // datasheet section 7.7.1.2.
    constexpr std::uint8_t kVbusReadyBit = 0x80;

    uint8_t vBusMsb;
    uint8_t vBusLsb;

    if (! (this->writeRegister(Register::TaskVbusMeasure_7, (uint8_t)Command::Trigger) &&
           this->waitForAdcReady(kVbusReadyBit) &&
           this->readRegister(Register::AdcVbatResultMsb_3, vBusMsb) &&
           this->readRegister(Register::AdcGpResultLsbs_1, vBusLsb)))
        return false;

    // AdcGpResultLsbs_1 is shared between burst-mode VBAT0..3 and VBUS,
    // the same way AdcVbatResultMsb_3 is shared between VBAT3-burst and
    // VBUS: when TASK_VBUS7_MEASURE was triggered, the VBUS LSBs are in
    // field D (bits 7:6), the same field used for VBAT3-burst's LSBs.
    // See datasheet sections 7.1.10.22 (ADCVBAT3RESULTMSB) and
    // 7.1.10.23 (ADCGP1RESULTLSBS).
    constexpr uint8_t kVbusLsbMask = 0xC0;
    constexpr uint8_t kVbusLsbShift = 6;
    vBusLsb = (vBusLsb & kVbusLsbMask) >> kVbusLsbShift;

    // Combine the 8-bit MSB and 2-bit LSB into the full 10-bit ADC
    // result. See datasheet section 7.1.4, Table 28 "Measurement
    // results".
    uint16_t result = (vBusMsb << 2) | vBusLsb;

    // VBUS = (ADC result / full-scale ADC code) * VFSVBUS. The ADC is a
    // 10-bit converter, and VFSVBUS (the full-scale voltage for the
    // VBUS channel) is 7.5V. See datasheet section 7.1.4 (the "VBUS"
    // equation) and section 7.1.9, Table 29 (Electrical specification),
    // symbol VFSVBUS.
    constexpr float kAdcMaxCode10Bit = 1023.0f;
    constexpr float kVfsVbus = 7.5f;
    voltage = (result / kAdcMaxCode10Bit) * kVfsVbus;
    return true;
    }

bool cNPM1300::measureVsys(float &voltage)
    {
    // EVENTADCVSYSRDY: EVENTSADCSET/EVENTSADCCLR field D (bit 3). See
    // datasheet section 7.7.1.2.
    constexpr std::uint8_t kVsysReadyBit = 0x08;

    uint8_t vSysMsb;
    uint8_t vSysLsb;

    if (! (this->writeRegister(Register::TaskVsysMeasure, (uint8_t)Command::Trigger) &&
           this->waitForAdcReady(kVsysReadyBit) &&
           this->readRegister(Register::AdcVsysResultMsb, vSysMsb) &&
           this->readRegister(Register::AdcGpResultLsbs_0, vSysLsb)))
        return false;

    // ADCGP0RESULTLSBS field D (bits 7:6) holds the VSYS measurement's
    // LSBs. See datasheet section 7.1.10.18, ADCGP0RESULTLSBS.
    constexpr uint8_t kVsysLsbMask = 0xC0;
    constexpr uint8_t kVsysLsbShift = 6;
    vSysLsb = (vSysLsb & kVsysLsbMask) >> kVsysLsbShift;

    // Combine the 8-bit MSB and 2-bit LSB into the full 10-bit ADC
    // result. See datasheet section 7.1.4, Table 28 "Measurement
    // results".
    uint16_t result = (vSysMsb << 2) | vSysLsb;

    // VSYS = (ADC result / full-scale ADC code) * VFSVSYS. The ADC is a
    // 10-bit converter, and VFSVSYS (the full-scale voltage for the
    // VSYS channel) is 6.375V. See datasheet section 7.1.4 (the "VSYS"
    // equation) and section 7.1.9, Table 29 (Electrical specification),
    // symbol VFSVSYS.
    constexpr float kAdcMaxCode10Bit = 1023.0f;
    constexpr float kVfsVsys = 6.375f;
    voltage = (result / kAdcMaxCode10Bit) * kVfsVsys;
    return true;
    }

bool cNPM1300::measureTemperature(float &temperature)
    {
    // EVENTADCTEMPRDY: EVENTSADCSET/EVENTSADCCLR field C (bit 2). See
    // datasheet section 7.7.1.2.
    constexpr std::uint8_t kTempReadyBit = 0x04;

    uint8_t tempMsb;
    uint8_t tempLsb;

    if (! (this->writeRegister(Register::TaskTempMeasure, (uint8_t)Command::Trigger) &&
           this->waitForAdcReady(kTempReadyBit) &&
           this->readRegister(Register::AdcTempResultMsb, tempMsb) &&
           this->readRegister(Register::AdcGpResultLsbs_0, tempLsb)))
        return false;

    // ADCGP0RESULTLSBS field C (bits 5:4) holds the die-temperature
    // measurement's LSBs. See datasheet section 7.1.10.18,
    // ADCGP0RESULTLSBS.
    constexpr uint8_t kTempLsbMask = 0x30;
    constexpr uint8_t kTempLsbShift = 4;
    tempLsb = (tempLsb & kTempLsbMask) >> kTempLsbShift;

    // Combine the 8-bit MSB and 2-bit LSB into the full 10-bit ADC
    // result. See datasheet section 7.1.4, Table 28 "Measurement
    // results".
    uint16_t result = (tempMsb << 2) | tempLsb;

    // Die temperature (deg C) = kDieTempIntercept - kDieTempSlope * ADC
    // result. See datasheet section 7.1.4, "Die temperature in C"
    // equation, variable KDIETEMP.
    constexpr float kDieTempIntercept = 394.67f;
    constexpr float kDieTempSlope = 0.7926f;
    temperature = kDieTempIntercept - (kDieTempSlope * result);
    return true;
    }

bool cNPM1300::getVbusStatus(uint8_t &status)
    {
    return this->readRegister(Register::VbusInStatus, status);
    }

bool cNPM1300::getBuckStatus(uint8_t &status)
    {
    return this->readRegister(Register::BuckStatus, status);
    }

bool cNPM1300::getLoadSwitchStatus(uint8_t &status)
    {
    return this->readRegister(Register::LdswStatus, status);
    }

bool cNPM1300::enterShipMode()
    {
    return this->writeRegister(Register::TaskEnterShipMode, (uint8_t)Command::Trigger);
    }

bool cNPM1300::setBuck_1(bool state)
    {
    bool result;
    if (state == true)
        {
        // enable the Buck 1
        result = this->writeRegister(Register::BuckEnaSet_1, (uint8_t)Command::Trigger);
        }
    else
        {
        // disable the Buck 1
        result = this->writeRegister(Register::BuckEnaClr_1, (uint8_t)Command::Trigger);
        }

    return result;
    }

bool cNPM1300::setBuck_2(bool state)
    {
    bool result;
    if (state == true)
        {
        // enable the Buck 2
        result = this->writeRegister(Register::BuckEnaSet_2, (uint8_t)Command::Trigger);
        }
    else
        {
        // disable the Buck 2
        result = this->writeRegister(Register::BuckEnaClr_2, (uint8_t)Command::Trigger);
        }

    return result;
    }

bool cNPM1300::setBuckVoltage_1(cNPM1300::Voltage voltage)
    {
    if (! this->writeRegister(Register::BuckNormVout_1, (uint8_t)voltage))
        return false;

    // BUCKSWCTRLSEL field A (BUCK1SWCTRLSEL, bit 0): 1 = let software
    // override the VSET pin and use the value just written to
    // BuckNormVout_1. See datasheet section 6.3.8.16, BUCKSWCTRLSEL.
    // Read the register and set only this bit, so that we don't
    // clear the other buck's bit and send it back to its VSET voltage.
    constexpr uint8_t kBuck1SwCtrlSelBit = 0x01;
    uint8_t swCtrlSel;
    if (! this->readRegister(Register::BuckSwCtrlSel, swCtrlSel))
        return false;

    return this->writeRegister(Register::BuckSwCtrlSel, swCtrlSel | kBuck1SwCtrlSelBit);
    }

bool cNPM1300::setBuckVoltage_2(cNPM1300::Voltage voltage)
    {
    if (! this->writeRegister(Register::BuckNormVout_2, (uint8_t)voltage))
        return false;

    // BUCKSWCTRLSEL field B (BUCK2SWCTRLSEL, bit 1): 1 = let software
    // override the VSET pin and use the value just written to
    // BuckNormVout_2. See datasheet section 6.3.8.16, BUCKSWCTRLSEL.
    // Read the register and set only this bit, so that we don't
    // clear the other buck's bit and send it back to its VSET voltage.
    constexpr uint8_t kBuck2SwCtrlSelBit = 0x02;
    uint8_t swCtrlSel;
    if (! this->readRegister(Register::BuckSwCtrlSel, swCtrlSel))
        return false;

    return this->writeRegister(Register::BuckSwCtrlSel, swCtrlSel | kBuck2SwCtrlSelBit);
    }

bool cNPM1300::selectLdoLoadSwitchMode_1(bool mode)
    {
    bool result;
    // mode true - LDO mode, slection false - Load Switch mode
    if (mode)
        {
        result = this->writeRegister(Register::LdswLdoSel_1, (uint8_t)mode);
        }
    else
        {
        result = this->writeRegister(Register::LdswLdoSel_1, (uint8_t)mode);
        }

    return result;
    }

bool cNPM1300::selectLdoLoadSwitchMode_2(bool mode)
    {
    bool result;
    // mode true - LDO mode, slection false - Load Switch mode
    if (mode)
        {
        result = this->writeRegister(Register::LdswLdoSel_2, (uint8_t)mode);
        }
    else
        {
        result = this->writeRegister(Register::LdswLdoSel_2, (uint8_t)mode);
        }

    return result;
    }

bool cNPM1300::setLoadSwitch_1(bool state)
    {
    bool result;
    if (state == true)
        {
        // enable the load switch 1
        result = this->writeRegister(Register::TaskLdswSet_1, (uint8_t)Command::Trigger);
        }
    else
        {
        // disable the load switch 1
        result = this->writeRegister(Register::TaskLdswClr_1, (uint8_t)Command::Trigger);
        }

    return result;
    }

bool cNPM1300::setLoadSwitch_2(bool state)
    {
    bool result;
    if (state == true)
        {
        // enable the load switch 2
        result = this->writeRegister(Register::TaskLdswSet_2, (uint8_t)Command::Trigger);
        }
    else
        {
        // disable the load switch 2
        result = this->writeRegister(Register::TaskLdswClr_2, (uint8_t)Command::Trigger);
        }

    return result;
    }

bool cNPM1300::setLdoVoltage_1(cNPM1300::Voltage voltage)
    {
    bool result;
    result = this->writeRegister(Register::LdswVoutSel_1, (uint8_t)voltage);

    return result;
    }

bool cNPM1300::setLdoVoltage_2(cNPM1300::Voltage voltage)
    {
    bool result;
    result = this->writeRegister(Register::LdswVoutSel_2, (uint8_t)voltage);

    return result;
    }

bool cNPM1300::ledSetMode(cNPM1300::Led led, cNPM1300::LedMode mode)
    {
    bool result;
    static const Register reg[3] = {
        Register::LedDrvModeSel_0,
        Register::LedDrvModeSel_1,
        Register::LedDrvModeSel_2
        };

    result = this->writeRegister(reg[(uint8_t)led], (uint8_t)mode);

    return result;
    }

bool cNPM1300::ledOn(cNPM1300::Led led)
    {
    bool result;
    static const Register reg[3] = {
        Register::LedDrvSet_0,
        Register::LedDrvSet_1,
        Register::LedDrvSet_2
        };

    result = this->writeRegister(reg[(uint8_t)led], (uint8_t)Command::Trigger);
    this->ledState[(uint8_t)led] = true;

    return result;
    }

bool cNPM1300::ledOff(cNPM1300::Led led)
    {
    bool result;
    static const Register reg[3] = {
        Register::LedDrvClr_0,
        Register::LedDrvClr_1,
        Register::LedDrvClr_2
        };

    result = this->writeRegister(reg[(uint8_t)led], (uint8_t)Command::Trigger);
    this->ledState[(uint8_t)led] = false;

    return result;
    }

bool cNPM1300::ledSetState(cNPM1300::Led led, bool state)
    {
    bool result;
    if(state)
        {
        result = ledOn(led);
        }
    else
        {
        result = ledOff(led);
        }

    return result;
    }

bool cNPM1300::ledGetState(cNPM1300::Led led)
    {
    return this->ledState[(uint8_t)led];
    }

bool cNPM1300::ledToggle(cNPM1300::Led led)
    {
    bool result;
    this->ledState[(uint8_t)led] = !this->ledState[(uint8_t)led];

    result = ledSetState(led, this->ledState[(uint8_t)led]);

    return result;
    }

bool cNPM1300::setTaskUpdateILimit(bool limit)
    {
    bool result;
    result = this->writeRegister(Register::TaskUpdateILimSw, (uint8_t)limit);

    return result;
    }

bool cNPM1300::setInCurrentLimitVbus(cNPM1300::Current limit)
    {
    bool result;
    result = this->writeRegister(Register::VbusInILim_0, (uint8_t)limit);

    return result;
    }

bool cNPM1300::readInCurrentLimitVbus(uint8_t &limit)
    {
    return this->readRegister(Register::VbusInILim_0, limit);
    }

bool cNPM1300::setStartupInCurrentLimitVbus(cNPM1300::Current limit)
    {
    bool result;
    result = this->writeRegister(Register::VbusInILimStartup, (uint8_t)limit);

    return result;
    }

bool cNPM1300::readStartupInCurrentLimitVbus(uint8_t &limit)
    {
    return this->readRegister(Register::VbusInILimStartup, limit);
    }

bool cNPM1300::suspendVbus(bool mode)
    {
    bool result;
    result = this->writeRegister(Register::VbusSuspend, (uint8_t)mode);

    return result;
    }

bool cNPM1300::getVbusComparatorStatus(uint8_t &status)
    {
    return this->readRegister(Register::UsbCDetectStatus, status);
    }

bool cNPM1300::chargerOn()
    {
    if (! this->applyNtc())
        return false;

    return this->writeRegister(Register::BChgEnableSet, (uint8_t)Command::Trigger);
    }

bool cNPM1300::applyNtc()
    {
    // BCHGDISABLESET/BCHGDISABLECLR field B (DISABLENTC, bit 1): set to
    // make the charger ignore the NTC thermistor's temperature limits,
    // clear to apply them. See datasheet section 6.2.14.6.
    constexpr uint8_t kBChgDisableNtcBit = 0x02;

    return this->writeRegister(
        this->m_ntc == Ntc::NotFitted ? Register::BChgDisableSet : Register::BChgDisableClr,
        kBChgDisableNtcBit
        );
    }

bool cNPM1300::enableRecharge(bool enable)
    {
    bool result;
    if (enable)
        {
        result = this->writeRegister(Register::BChgDisableClr, (uint8_t)Command::Trigger);
        }
    else
        {
        result = this->writeRegister(Register::BChgDisableSet, (uint8_t)(Command::Trigger));
        }

    return result;
    }

bool cNPM1300::chargerOff()
    {
    bool result;
    result = this->writeRegister(Register::BChgEnableClr, (uint8_t)Command::Trigger);

    return result;
    }

bool cNPM1300::getVterm(uint8_t &vTerm)
    {
    return this->readRegister(Register::BChgVTerm, vTerm);
    }

bool cNPM1300::setVterm(cNPM1300::Vterm vTerm)
    {
    bool result_1, result_2;
    // set voltage at which to stop charging
    result_1 = this->writeRegister(Register::BChgVTerm, (uint8_t)vTerm);
    // set voltage at which to stop charging when the die temp is warm
    result_2 = this->writeRegister(Register::BChgVTermR, (uint8_t)vTerm);

    return (result_1 && result_2);
    }

bool cNPM1300::getChargeStatus(uint8_t &status)
    {
    return this->readRegister(Register::BChgChargeStatus, status);
    }

bool cNPM1300::getErrorReason(uint8_t &reason)
    {
    return this->readRegister(Register::BChgErrReason, reason);
    }

bool cNPM1300::getErrorSensor(uint8_t &sensor)
    {
    return this->readRegister(Register::BChgErrSensor, sensor);
    }

bool cNPM1300::setChargeCurrent(uint16_t limit)
    {
    bool result_1, result_2;

    // make sure value is between 32 and 800
    if(limit < 32)
        {
        limit = 32;
        }
    if(limit > 800)
        {
        limit = 800;
        }

    uint8_t msb = limit/4;
    uint8_t lsb = 0;
    if ((limit/2) % 2)
        {
        lsb = 1;
        }

    // the charger must be disabled while BCHGISET changes; the new value
    // takes effect when charging is enabled (datasheet section 6.2.4).
    // BCHGENABLESET bit 0 reads 1 while the charger is enabled (6.2.14.4).
    uint8_t enable;
    if (! this->readRegister(Register::BChgEnableSet, enable))
        return false;

    const bool fWasOn = (enable & 0x01) != 0;
    if (fWasOn && ! this->writeRegister(Register::BChgEnableClr, (uint8_t)Command::Trigger))
        return false;

    result_1 = this->writeRegister(Register::BChgISetMsb, msb);
    result_2 = this->writeRegister(Register::BChgISetLsb, lsb);

    // turn it back on even if a write failed, so a failure doesn't
    // leave the battery uncharged.
    if (fWasOn && ! this->writeRegister(Register::BChgEnableSet, (uint8_t)Command::Trigger))
        return false;

    return (result_1 && result_2);
    }

bool cNPM1300::setGpioMode(cNPM1300::GpioMode mode, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioModeRegisters[] =
        {
        Register::GpioMode_0,
        Register::GpioMode_1,
        Register::GpioMode_2,
        Register::GpioMode_3,
        Register::GpioMode_4
        };

    result = this->writeRegister(gpioModeRegisters[gpioNumber], (uint8_t)(mode));
    return result;
    }

bool cNPM1300::setGpioDrive(cNPM1300::GpioDrive current, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioDriveRegisters[] =
        {
        Register::GpioDrive_0,
        Register::GpioDrive_1,
        Register::GpioDrive_2,
        Register::GpioDrive_3,
        Register::GpioDrive_4
        };

    result = this->writeRegister(gpioDriveRegisters[gpioNumber], (uint8_t)(current));
    return result;
    }

bool cNPM1300::setGpioPullUp(bool enable, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioPullUpRegisters[] =
        {
        Register::GpioPuEn_0,
        Register::GpioPuEn_1,
        Register::GpioPuEn_2,
        Register::GpioPuEn_3,
        Register::GpioPuEn_4
        };

    result = this->writeRegister(gpioPullUpRegisters[gpioNumber], (uint8_t)(enable));
    return result;
    }

bool cNPM1300::setGpioPullDown(bool enable, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioPullDownRegisters[] =
        {
        Register::GpioPdEn_0,
        Register::GpioPdEn_1,
        Register::GpioPdEn_2,
        Register::GpioPdEn_3,
        Register::GpioPdEn_4
        };

    result = this->writeRegister(gpioPullDownRegisters[gpioNumber], (uint8_t)(enable));
    return result;
    }

bool cNPM1300::setGpioOpenDrain(bool enable, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioOpenDrainRegisters[] =
        {
        Register::GpioOpenDrain_0,
        Register::GpioOpenDrain_1,
        Register::GpioOpenDrain_2,
        Register::GpioOpenDrain_3,
        Register::GpioOpenDrain_4
        };

    result = this->writeRegister(gpioOpenDrainRegisters[gpioNumber], (uint8_t)(enable));
    return result;
    }

bool cNPM1300::setGpioDebounce(bool enable, uint8_t gpioNumber)
    {
    bool result;
    // Validate the GPIO number
    if (gpioNumber >= 5)
        {
        return false;
        }

    static const Register gpioDebounceRegisters[] =
        {
        Register::GpioDebounce_0,
        Register::GpioDebounce_1,
        Register::GpioDebounce_2,
        Register::GpioDebounce_3,
        Register::GpioDebounce_4
        };

    result = this->writeRegister(gpioDebounceRegisters[gpioNumber], (uint8_t)(enable));
    return result;
    }

bool cNPM1300::getGpioStatus(uint8_t &status)
    {
    return this->readRegister(Register::GpioStatus, status);
    }

bool cNPM1300::setLoadSwitchGpioControl_1(uint8_t gpioNumber, bool invert)
    {
    // Validate GPIO number (valid range is 1 to 5, as 0 means no GPIO selected)
    if (gpioNumber > 5)
        {
        return false;
        }

    // LDSW1GPISEL field A (LDSW1GPISEL, bits 2:0): which GPI controls
    // Load Switch 1. Field B (LDSW1GPIINV, bit 3): invert the sense of
    // that GPIO. See datasheet section 6.4.3.6, LDSW1GPISEL.
    constexpr uint8_t kLdswGpiSelMask = 0x07;
    constexpr uint8_t kLdswGpiInvertBit = 0x08;
    std::uint8_t value = (gpioNumber & kLdswGpiSelMask) | (invert ? kLdswGpiInvertBit : 0x00);

    return this->writeRegister(Register::LdswGpiSel_1, value);
    }

bool cNPM1300::setLoadSwitchGpioControl_2(uint8_t gpioNumber, bool invert)
    {
    // Validate GPIO number (valid range is 1 to 5, as 0 means no GPIO selected)
    if (gpioNumber > 5)
        {
        return false;
        }

    // LDSW2GPISEL field A (LDSW2GPISEL, bits 2:0): which GPI controls
    // Load Switch 2. Field B (LDSW2GPIINV, bit 3): invert the sense of
    // that GPIO. See datasheet section 6.4.3.7, LDSW2GPISEL.
    constexpr uint8_t kLdswGpiSelMask = 0x07;
    constexpr uint8_t kLdswGpiInvertBit = 0x08;
    std::uint8_t value = (gpioNumber & kLdswGpiSelMask) | (invert ? kLdswGpiInvertBit : 0x00);

    return this->writeRegister(Register::LdswGpiSel_2, value);
    }

bool cNPM1300::setBuckGpioControl(uint8_t buckNumber, uint8_t gpioNumber, bool invert)
    {
    bool result;
    // Validate buck number (must be 1 or 2)
    if (buckNumber < 1 || buckNumber > 2)
        {
        return false;
        }

    // Validate GPIO number (valid range is 0 to 5)
    if (gpioNumber > 5)
        {
        return false;
        }

    // Read the current value of the register
    std::uint8_t regValue;
    if (!this->readRegister(Register::BuckEnCtrl, regValue))
        {
        return false;
        }

    // BUCKENCTRL fields (see datasheet section 6.3.8.13, BUCKENCTRL):
    // A = BUCK1ENGPISEL (bits 2:0), B = BUCK2ENGPISEL (bits 5:3),
    // C = BUCK1ENGPIINV (bit 6), D = BUCK2ENGPIINV (bit 7).
    constexpr uint8_t kBuck1GpiSelMask = 0x07;
    constexpr uint8_t kBuck1GpiInvertBit = 0x40;
    constexpr uint8_t kBuck2GpiSelShift = 3;
    constexpr uint8_t kBuck2GpiSelMask = 0x07 << kBuck2GpiSelShift;
    constexpr uint8_t kBuck2GpiInvertBit = 0x80;

    // Construct the new value based on the buck number
    if (buckNumber == 1)
        {
        // Clear and set BUCK1ENGPISEL and BUCK1ENGPIINV
        regValue &= ~(kBuck1GpiSelMask | kBuck1GpiInvertBit);
        regValue |= (gpioNumber & kBuck1GpiSelMask);
        regValue |= (invert ? kBuck1GpiInvertBit : 0x00);
        }
    else if (buckNumber == 2)
        {
        // Clear and set BUCK2ENGPISEL and BUCK2ENGPIINV
        regValue &= ~(kBuck2GpiSelMask | kBuck2GpiInvertBit);
        regValue |= ((gpioNumber << kBuck2GpiSelShift) & kBuck2GpiSelMask);
        regValue |= (invert ? kBuck2GpiInvertBit : 0x00);
        }

    // Write the updated value back to the register
    result = this->writeRegister(Register::BuckEnCtrl, regValue);
    return result;
    }

// protected
bool cNPM1300::readRegister(cNPM1300::Register reg, std::uint8_t &value)
    {
    return this->readRegisters(reg, &value, 1);
    }

// protected
bool cNPM1300::readRegisters(cNPM1300::Register reg, std::uint8_t *pBuffer, size_t nBuffer)
    {
    if (pBuffer == nullptr || nBuffer > 32)
        return this->setLastError(Error::InternalInvalidParameter);

    this->m_wire->beginTransmission((uint8_t) this->m_address);
    if (this->m_wire->write((uint8_t)((uint16_t)reg >> 8)) != 1 || this->m_wire->write((uint8_t)((uint16_t)reg & 0xFF)) != 1)
        {
        return this->setLastError(Error::I2cReadRequest);
        }
    if (this->m_wire->endTransmission() != 0)
        {
        return this->setLastError(Error::I2cReadRequest);
        }

    auto nReadFrom = this->m_wire->requestFrom((uint8_t) this->m_address, std::uint8_t(nBuffer));

    if (nReadFrom != nBuffer)
        return this->setLastError(Error::I2cReadRequest);
    auto const nResult = unsigned(this->m_wire->available());

    if (nResult > nBuffer)
        return this->setLastError(Error::I2cReadLong);

    for (unsigned i = 0; i < nResult; ++i)
        pBuffer[i] = this->m_wire->read();

    if (nResult != nBuffer)
        return this->setLastError(Error::I2cReadShort);

    return true;
    }

// protected
bool cNPM1300::writeRegister(cNPM1300::Register reg, std::uint8_t value)
    {
    const std::uint8_t cmdbuf[3] = {
        (std::uint8_t)((uint16_t)reg >> 8),
        (std::uint8_t)((uint16_t)reg & 0xFF),
        value
        };

    this->m_wire->beginTransmission((uint8_t) this->m_address);

    if (this->m_wire->write(cmdbuf, sizeof(cmdbuf)) != sizeof(cmdbuf))
        return this->setLastError(Error::I2cWriteBufferFailed);

    if (this->m_wire->endTransmission() != 0)
        return this->setLastError(Error::I2cWriteFailed);

    return true;
    }

/****************************************************************************\
|   String handling for error routines
\****************************************************************************/

static const char *scanMultiSzString(const char *p, unsigned eIndex)
    {
    // iterate based on error index.
    for (; eIndex > 0; --eIndex)
        {
        // stop when we get to empty string
        if (*p == '\0')
            break;
        // otherwise, skip this one.
        p += strlen(p) + 1;
        }

    // if we have a valid string, return it
    if (*p != '\0')
        return p;
    // otherwise indicate that the input wasn't valid.
    else
        return "<<unknown>>";
    }

const char * cNPM1300::getErrorName(cNPM1300::Error e)
    {
    return scanMultiSzString(m_szErrorMessages, unsigned(e));
    }

const char * cNPM1300::getStateName(cNPM1300::State s)
    {
    return scanMultiSzString(m_szStateNames, unsigned(s));
    }

/**** end of MCCI_Catena_nPM1300.cpp ****/