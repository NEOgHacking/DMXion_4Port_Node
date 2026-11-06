/*

Module: MCCI_Catena_nPM1300.h

Function:
    Top-level include file for MCCI Catena nPM1300 power management IC library.

Copyright and License:
    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau R, MCCI Corporation   May 2025

*/

/// \file

#ifndef _MCCI_CATENA_NPM1300_H_
#define _MCCI_CATENA_NPM1300_H_ /* prevent multiple includes */

#pragma once

#include <cstdint>
#include <Wire.h>
#include <stdlib.h>

/// \brief namespace for this library
namespace McciCatenaNpm1300 {

/// \brief create a version constant uint32_t
static constexpr std::uint32_t
makeVersion(
    std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0
    )
    {
    return ((std::uint32_t)major << 24u) | ((std::uint32_t)minor << 16u) | ((std::uint32_t)patch << 8u) | (std::uint32_t)local;
    }

/// \brief extract major number from version
static constexpr std::uint8_t
getMajor(std::uint32_t v)
    {
    return std::uint8_t(v >> 24u);
    }

/// \brief extract minor number from version
static constexpr std::uint8_t
getMinor(std::uint32_t v)
    {
    return std::uint8_t(v >> 16u);
    }

/// \brief extract patch number from version
static constexpr std::uint8_t
getPatch(std::uint32_t v)
    {
    return std::uint8_t(v >> 8u);
    }

/// \brief extract local number from version
static constexpr std::uint8_t
getLocal(std::uint32_t v)
    {
    return std::uint8_t(v);
    }

/// \brief version of library, for use by clients in static_asserts
static constexpr std::uint32_t kVersion = makeVersion(1,3,0,3);

/// \brief instance object for nPm 1300 IC
class cNPM1300
    {
private:
    static constexpr bool kfDebug = false;

public:
    ///
    /// \brief the address type
    ///
    enum class Address : std::int8_t
        {
        Error = -1,
        NPM1300 = 0x6b,
        };

    // the type for pin assignments, in case the ready pin is used
    using Pin_t = std::int8_t;

    ///
    /// \brief the constructor
    ///
    /// \param [in] wire is the TwoWire bus to use for this sensor.
    /// \param [in] Address is the device address.
    /// \param [in] pinReady is the alert pin. It is set -1 if there is none
    ///
    cNPM1300(TwoWire &wire, Address Address = Address::NPM1300, Pin_t pinReady = -1)
        : m_wire(&wire)
        , m_address(Address)
        , m_pinReady(pinReady)
        {}

    // neither copyable nor movable
    cNPM1300(const cNPM1300&) = delete;
    cNPM1300& operator=(const cNPM1300&) = delete;
    cNPM1300(const cNPM1300&&) = delete;
    cNPM1300& operator=(const cNPM1300&&) = delete;

    /// \brief Register
    enum class Register : std::uint16_t
        {
        /// \brief Event and Interrupt Register
        TaskSwReset                             =   0x0001,     ///< Task Force a full reboot power-cycle
        EventsAdcSet                            =   0x0002,     ///< ADC Events Event Set
        EventsAdcClr                            =   0x0003,     ///< ADC Events Event Clear
        IntEnEventsAdcSet                       =   0x0004,     ///< ADC Events Interrupt Enable Set
        IntEnEventsAdcClr                       =   0x0005,     ///< ADC Events Interrupt Enable Clear
        EventsBChargerSet_0                     =   0x0006,     ///< Battery Charger Temperature Events Event Set
        EventsBChargerClr_0                     =   0x0007,     ///< Battery Charger Temperature Events Event Clear
        IntEnEventsBChargerSet_0                =   0x0008,     ///< Battery Charger Temperature Events Interrupt Enable Set
        IntEnEventsBChargerClr_0                =   0x0009,     ///< Battery Charger Temperature Events Interrupt Enable Clear
        EventsBChargerSet_1                     =   0x000A,     ///< Battery Charger Status Events Event Set
        EventsBChargerClr_1                     =   0x000B,     ///< Battery Charger Status Events Event Clear
        IntEnEventsBChargerSet_1                =   0x000C,     ///< Battery Charger Status Events Interrupt Enable Set
        IntEnEventsBChargerClr_1                =   0x000D,     ///< Battery Charger Status Events Interrupt Enable Clear
        EventsBChargerSet_2                     =   0x000E,     ///< Battery Charger Battery Events Event Set
        EventsBChargerClr_2                     =   0x000F,     ///< Battery Charger Battery Events Event Clear
        IntEnEventsBChargerSet_2                =   0x0010,     ///< Battery Charger Battery Events Interrupt Enable Set
        IntEnEventsBChargerClr_2                =   0x0011,     ///< Battery Charger Battery Events Interrupt Enable Clear
        EventsShpHoldSet                        =   0x0012,     ///< ShipHold pin Events Event Set
        EventsShpHoldClr                        =   0x0013,     ///< ShipHold pin Events Event Clear
        IntEnEventsShpHoldSet                   =   0x0014,     ///< ShipHold pin Events Interrupt Enable Set
        IntEnEventsShpHoldClr                   =   0x0015,     ///< ShipHold pin Events Interrupt Enable Clear
        EventsVbusInSet_0                       =   0x0016,     ///< VBUSIN Voltage Detection Events Event Set
        EventsVbusInClr_0                       =   0x0017,     ///< VBUSIN Voltage Detection Events Event Clear
        IntEnEventsVbusInSet_0                  =   0x0018,     ///< VBUSIN Voltage Detection Events Interrupt Enable Set
        IntEnEventsVbusInClr_0                  =   0x0019,     ///< VBUSIN Voltage Detection Events Interrupt Enable Clear
        EventsVbusInSet_1                       =   0x001A,     ///< VBUSIN Thermal and USB Events Event Set
        EventsVbusInClr_1                       =   0x001B,     ///< VBUSIN Thermal and USB Events Event Clear
        IntEnEventsVbusInSet_1                  =   0x001C,     ///< VBUSIN Thermal and USB Events Interrupt Enable Set
        IntEnEventsVbusInClr_1                  =   0x001D,     ///< VBUSIN Thermal and USB Events Interrupt Enable Clear
        EventsGpioSet                           =   0x0022,     ///< GPIO Event Event Set
        EventsGpioClr                           =   0x0023,     ///< GPIO Event Event Clear
        IntEnEventsGpioSet                      =   0x0024,     ///< GPIO Event Interrupt Enable Set
        IntEnEventsGpioClr                      =   0x0025,     ///< GPIO Event Interrupt Enable Clear
        ///\brief System Regulator Register
        TaskUpdateILimSw                        =   0x0200,     ///< Select Input Current limit for VBUS
        VbusInILim_0                            =   0x0201,     ///< Select Input Current limit for VBUS, Reset value from OTP
        VbusInILimStartup                       =   0x0202,     ///< Select input Current limit for VBUS at Startup
        VbusSuspend                             =   0x0203,     ///< Suspend mode enable
        UsbCDetectStatus                        =   0x0205,     ///< VBUS CC comparator status flags
        VbusInStatus                            =   0x0207,     ///< VBUS status flags
        ///\brief Charger Register
        TaskReleaseErr                          =   0x0300,     ///< Release Charger from Error
        TaskClearChgErr                         =   0x0301,     ///< Clear error registers
        TaskClearSafetyTimer                    =   0x0302,     ///< Clear safety timers
        BChgEnableSet                           =   0x0304,     ///< Charger Enable Set
        BChgEnableClr                           =   0x0305,     ///< Charger Enable Clear
        BChgDisableSet                          =   0x0306,     ///< Charger Disable Recharge Set
        BChgDisableClr                          =   0x0307,     ///< Charger Disable Recharge Clear
        BChgISetMsb                             =   0x0308,     ///< Battery Charger Current Configuration
        BChgISetLsb                             =   0x0309,     ///< Battery Charger Current Configuration
        BChgISetDischargeMsb                    =   0x030A,     ///< Battery Charger Discharge Configuration
        BChgISetDischargeLsb                    =   0x030B,     ///< Battery Charger Discharge Configuration
        BChgVTerm                               =   0x030C,     ///< Battery Charger Termination Voltage Normal temp
        BChgVTermR                              =   0x030D,     ///< Battery Charger Termination Voltage Warm temp
        BChgVTrickleSel                         =   0x030E,     ///< Battery Charger Trickle Level Select
        BChgITermSel                            =   0x030F,     ///< Battery Charger ITERM Level Select
        NtcCold                                 =   0x0310,     ///< NTC thermistor threshold for COLD temperature region
        NtcColdLsb                              =   0x0311,     ///< NTC thermistor threshold for COLD temperature region
        NtcCool                                 =   0x0312,     ///< NTC thermistor threshold for COOL temperature region
        NtcCoolLsb                              =   0x0313,     ///< NTC thermistor threshold for COOL temperature region
        NtcWarm                                 =   0x0314,     ///< NTC thermistor threshold for WARM temperature region
        NtcWarmLsb                              =   0x0315,     ///< NTC thermistor threshold for WARM temperature region
        NtcHot                                  =   0x0316,     ///< NTC thermistor threshold for HOT temperature region
        NtcHotLsb                               =   0x0317,     ///< NTC thermistor threshold for HOT temperature region
        DieTempStop                             =   0x0318,     ///< DIE TEMP threshold for stop charging
        DieTempStopLsb                          =   0x0319,     ///< DIE TEMP threshold for stop charging lsb
        DieTempResume                           =   0x031A,     ///< DIE TEMP threshold for resuming charging
        DieTempResumeLsb                        =   0x031B,     ///< DIE TEMP threshold for resuming charging lsb
        BChgILimStatus                          =   0x032D,     ///< BCHARGER Ilim Status
        NtcStatus                               =   0x0332,     ///< NTC Comparator Status
        DieTempStatus                           =   0x0333,     ///< DieTemp Comparator Status
        BChgChargeStatus                        =   0x0334,     ///< Charging Status
        BChgErrReason                           =   0x0336,     ///< Charger-FSM Error. Latched error reasons. Cleared with TASKS_CLEAR_CHG_ERR
        BChgErrSensor                           =   0x0337,     ///< Charger-FSM Error. Latched sensor values. Cleared with TASKS_CLEAR_CHG_ERR
        BChgConfig                              =   0x033C,     ///< Charger configuration
        ///\brief Buck Regulator Register
        BuckEnaSet_1                            =   0x0400,     ///< BUCK1 Enable pulse
        BuckEnaClr_1                            =   0x0401,     ///< BUCK1 Disable pulse
        BuckEnaSet_2                            =   0x0402,     ///< BUCK2 Enable pulse
        BuckEnaClr_2                            =   0x0403,     ///< BUCK2 Disable pulse
        BuckPwmSet_1                            =   0x0404,     ///< BUCK1 PWM mode enable pulse
        BuckPwmClr_1                            =   0x0405,     ///< BUCK1 PWM mode disable pulse
        BuckPwmSet_2                            =   0x0406,     ///< BUCK2 PWM mode enable pulse
        BuckPwmClr_2                            =   0x0407,     ///< BUCK2 PWM mode disable pulse
        BuckNormVout_1                          =   0x0408,     ///< BUCK1 Output voltage Normal mode
        BuckRetVout_1                           =   0x0409,     ///< BUCK1 Output voltage Retention mode
        BuckNormVout_2                          =   0x040A,     ///< BUCK2 Output voltage Normal mode
        BuckRetVout_2                           =   0x040B,     ///< BUCK2 Output voltage Retention mode
        BuckEnCtrl                              =   0x040C,     ///< BUCK Enable GPIO Select
        BuckVRetCtrl                            =   0x040D,     ///< BUCK Retention Voltage select
        BuckPwmCtrl                             =   0x040E,     ///< BUCK Forced PWM mode GPIO select
        BuckSwCtrlSel                           =   0x040F,     ///< BUCK Software Control select
        BuckVoutStatus_1                        =   0x0410,     ///< BUCK1 VOUT Status register
        BuckVoutStatus_2                        =   0x0411,     ///< BUCK2 VOUT Status register
        BuckCtrl_0                              =   0x0415,     ///< BUCK Auto PFM to PWM Control select
        BuckStatus                              =   0x0434,     ///< BUCK status register
        /// \brief System Monitor Register
        TaskVbatMeasure                         =   0x0500,     ///< Task Take VBAT measurement
        TaskNtcMeasure                          =   0x0501,     ///< Task Take NTC measurement
        TaskTempMeasure                         =   0x0502,     ///< Task Take Die Temperature measurement
        TaskVsysMeasure                         =   0x0503,     ///< Task Take VSYS measurement
        TaskIbatMeasure                         =   0x0506,     ///< Task Take IBATmeasurement
        TaskVbusMeasure_7                       =   0x0507,     ///< Task Take VBUS 7V range measurement
        TaskDelayedVbatMeasure                  =   0x0508,     ///< Task Take delayed VBAT measurement
        AdcConfig                               =   0x0509,     ///< ADC Configuration
        AdcNtcrSel                              =   0x050A,     ///< Select Battery NTC register
        AdcAutoTimConf                          =   0x050B,     ///< Auto measurement intervals
        TaskAutoTimUpdate                       =   0x050C,     ///< update toggle for NTC and Die temp AutoTime register bits
        AdcDelTimConf                           =   0x050D,     ///< Vbat Delay timer control
        AdcIbatMeasureStatus                    =   0x0510,     ///< Battery current measurement status
        AdcVbatResultMsb                        =   0x0511,     ///< ADC VBAT measurement result MSB
        AdcNtcResultMsb                         =   0x0512,     ///< ADC NTC measurement result MSB
        AdcTempResultMsb                        =   0x0513,     ///< ADC DIE TEMP measurement result MSB
        AdcVsysResultMsb                        =   0x0514,     ///< ADC VSYS measurement result MSB
        AdcGpResultLsbs_0                       =   0x0515,     ///< ADC result LSB's (Vbat, Ntc, Temp and Vsys)
        AdcVbatResultMsb_0                      =   0x0516,     ///< ADC VBAT0 Burst measurement result MSB
        AdcVbatResultMsb_1                      =   0x0517,     ///< ADC VBAT1 Burst measurement result MSB
        AdcVbatResultMsb_2                      =   0x0518,     ///< ADC VBAT2 Burst measurement result MSB
        AdcVbatResultMsb_3                      =   0x0519,     ///< ADC VBAT3 Burst or VBUS measurement result MSB
        AdcGpResultLsbs_1                       =   0x051A,     ///< ADC result LSB's (Vbat_burst0, 1, 2 and 3)
        AdcIbatMeasen                           =   0x0524,     ///< Enable auto IBAT measurement
        /// \brief GPIO Register
        GpioMode_0                              =   0x0600,     ///< GPIO Mode Configuration
        GpioMode_1                              =   0x0601,     ///< GPIO Mode Configuration
        GpioMode_2                              =   0x0602,     ///< GPIO Mode Configuration
        GpioMode_3                              =   0x0603,     ///< GPIO Mode Configuration
        GpioMode_4                              =   0x0604,     ///< GPIO Mode Configuration
        GpioDrive_0                             =   0x0605,     ///< GPIO Drive strength Configuration
        GpioDrive_1                             =   0x0606,     ///< GPIO Drive strength Configuration
        GpioDrive_2                             =   0x0607,     ///< GPIO Drive strength Configuration
        GpioDrive_3                             =   0x0608,     ///< GPIO Drive strength Configuration
        GpioDrive_4                             =   0x0609,     ///< GPIO Drive strength Configuration
        GpioPuEn_0                              =   0x060A,     ///< GPIO Pull-up Enable Configuration
        GpioPuEn_1                              =   0x060B,     ///< GPIO Pull-up Enable Configuration
        GpioPuEn_2                              =   0x060C,     ///< GPIO Pull-up Enable Configuration
        GpioPuEn_3                              =   0x060D,     ///< GPIO Pull-up Enable Configuration
        GpioPuEn_4                              =   0x060E,     ///< GPIO Pull-up Enable Configuration
        GpioPdEn_0                              =   0x060F,     ///< GPIO Pull-down Enable Configuration
        GpioPdEn_1                              =   0x0610,     ///< GPIO Pull-down Enable Configuration
        GpioPdEn_2                              =   0x0611,     ///< GPIO Pull-down Enable Configuration
        GpioPdEn_3                              =   0x0612,     ///< GPIO Pull-down Enable Configuration
        GpioPdEn_4                              =   0x0613,     ///< GPIO Pull-down Enable Configuration
        GpioOpenDrain_0                         =   0x0614,     ///< GPIO Open Drain Configuration
        GpioOpenDrain_1                         =   0x0615,     ///< GPIO Open Drain Configuration
        GpioOpenDrain_2                         =   0x0616,     ///< GPIO Open Drain Configuration
        GpioOpenDrain_3                         =   0x0617,     ///< GPIO Open Drain Configuration
        GpioOpenDrain_4                         =   0x0618,     ///< GPIO Open Drain Configuration
        GpioDebounce_0                          =   0x0619,     ///< GPIO Debounce Configuration
        GpioDebounce_1                          =   0x061A,     ///< GPIO Debounce Configuration
        GpioDebounce_2                          =   0x061B,     ///< GPIO Debounce Configuration
        GpioDebounce_3                          =   0x061C,     ///< GPIO Debounce Configuration
        GpioDebounce_4                          =   0x061D,     ///< GPIO Debounce Configuration
        GpioStatus                              =   0x061E,     ///< GPIO Status from GPIO Pads
        /// \brief Timer Register
        TimerSet                                =   0x0700,     ///< Start Timer
        TimerClr                                =   0x0701,     ///< Stop Timer
        TimerTargetStrobe                       =   0x0703,     ///< Strobe for timer Target
        WatchdogKick                            =   0x0704,     ///< Watchdog kick
        TimerConfig                             =   0x0705,     ///< Timer mode selection
        TimerStatus                             =   0x0706,     ///< Timers Status
        TimerHiByte                             =   0x0708,     ///< Timer Most Significant Byte
        TimerMidByte                            =   0x0709,     ///< Timer Middle Byte
        TimerLowByte                            =   0x070A,     ///< Timer Least Significant Byte
        ///\brief Load Switch/LDO Regulator
        TaskLdswSet_1                           =   0x0800,     ///< Enable LDSW1
        TaskLdswClr_1                           =   0x0801,     ///< Disable LDSW1
        TaskLdswSet_2                           =   0x0802,     ///< Enable LDSW2
        TaskLdswClr_2                           =   0x0803,     ///< Disable LDSW2
        LdswStatus                              =   0x0804,     ///< Load Switch Status
        LdswGpiSel_1                            =   0x0805,     ///< Load Switch1 GPIO Control Select
        LdswGpiSel_2                            =   0x0806,     ///< Load Switch2 GPIO Control Select
        LdswConfig                              =   0x0807,     ///< Load Switch Configuration
        LdswLdoSel_1                            =   0x0808,     ///< Load Switch1 / LDO Select
        LdswLdoSel_2                            =   0x0809,     ///< Load Switch2 / LDO Select
        LdswVoutSel_1                           =   0x080C,     ///< LDO1 programmable output voltage
        LdswVoutSel_2                           =   0x080D,     ///< LDO2 programmable output voltage
        /// \brief POF (power-fail comparator) Register
        PofConfig                               =   0x0900,     ///< Power Failure Detection block configuration
        ///\brief LED Regulator
        LedDrvModeSel_0                         =   0x0A00,     ///< Select for LED_0 mode
        LedDrvModeSel_1                         =   0x0A01,     ///< Select for LED_1 mode
        LedDrvModeSel_2                         =   0x0A02,     ///< Select for LED_2 mode
        LedDrvSet_0                             =   0x0A03,     ///< Set LED_0 to be On
        LedDrvClr_0                             =   0x0A04,     ///< Clear LED_0 to be Off
        LedDrvSet_1                             =   0x0A05,     ///< Set LED_1 to be On
        LedDrvClr_1                             =   0x0A06,     ///< Clear LED_1 to be Off
        LedDrvSet_2                             =   0x0A07,     ///< Set LED_2 to be On
        LedDrvClr_2                             =   0x0A08,     ///< Clear LED_2 to be Off
        /// \brief Ship and Hibernate Register
        TaskEnterHibernate                      =   0x0B00,     ///< Task Enter Hibernate
        TaskShpHldCfgStrobe                     =   0x0B01,     ///< Task Ship Hold config
        TaskEnterShipMode                       =   0x0B02,     ///< Task enter ShipMode
        TaskResetCfg                            =   0x0B03,     ///< Request reset config
        ShpHldConfig                            =   0x0B04,     ///< Ship Hold button press timer config
        ShpHldStatus                            =   0x0B05,     ///< Status of the SHPHLD pin
        LpResetConfig                           =   0x0B06,     ///< Long press reset config register
        /// \brief Reset and Error Register
        TaskClrErrLog                           =   0x0E00,     ///< Task to clear the Errlog registers
        Scratch_0                               =   0x0E01,     ///< Scratch register 0
        Scratch_1                               =   0x0E02,     ///< Scratch register 1
        RstCause                                =   0x0E03,     ///< Error log for internal reset causes. Cleared withTASK_CLR_ERRLOG
        ChargerErrReason                        =   0x0E04,     ///< Error log for slowDomain. Cleared with TASK_CLR_ERRLOG
        ChargerErrSensor                        =   0x0E05,     ///< Bcharger Fsm sensor error. Cleared with TASK_CLR_ERRLOG
        };

    /// \brief Voltage Sources
    enum class Source : std::uint8_t
        {
        Buck_1 = 0,
        Buck_2,
        Ldo_1,
        Ldo_2,
        };

    /// \brief Voltage Levels
    enum class Voltage : std::uint8_t
        {
        V1 = 0,                                                 ///< 1V
        V1_1,                                                   ///< 1.1V
        V1_2,                                                   ///< 1.2V
        V1_3,                                                   ///< 1.3V
        V1_4,                                                   ///< 1.4V
        V1_5,                                                   ///< 1.5V
        V1_6,                                                   ///< 1.6V
        V1_7,                                                   ///< 1.7V
        V1_8,                                                   ///< 1.8V
        V1_9,                                                   ///< 1.9V
        V2_0,                                                   ///< 2.0V
        V2_1,                                                   ///< 2.1V
        V2_2,                                                   ///< 2.2V
        V2_3,                                                   ///< 2.3V
        V2_4,                                                   ///< 2.4V
        V2_5,                                                   ///< 2.5V
        V2_6,                                                   ///< 2.6V
        V2_7,                                                   ///< 2.7V
        V2_8,                                                   ///< 2.8V
        V2_9,                                                   ///< 2.9V
        V3_0,                                                   ///< 3.0V
        V3_1,                                                   ///< 3.1V
        V3_2,                                                   ///< 3.2V
        V3_3,                                                   ///< 3.3V
        V3_30,                                                  ///< 3.3V
        };

    /// \brief Current Levels
    enum class Current : std::uint8_t
        {
        MA_500_0 = 0,                                           ///< 500mA
        MA_100,                                                 ///< 100mA
        MA_200,                                                 ///< 200mA
        MA_300,                                                 ///< 300mA
        MA_400,                                                 ///< 400mA
        MA_500,                                                 ///< 500mA
        MA_600,                                                 ///< 600mA
        MA_700,                                                 ///< 700mA
        MA_800,                                                 ///< 800mA
        MA_900,                                                 ///< 900mA
        MA_1000,                                                ///< 1000mA
        MA_1100,                                                ///< 1100mA
        MA_1200,                                                ///< 1200mA
        MA_1300,                                                ///< 1300mA
        MA_1400,                                                ///< 1400mA
        MA_1500,                                                ///< 1500mA
        };

    /// \brief Voltage Levels
    enum class Vterm : std::uint8_t
        {
        V3_50 = 0,                                              ///< 3.50V
        V3_55,                                                  ///< 3.55V
        V3_60,                                                  ///< 3.60V
        V3_65,                                                  ///< 3.65V
        V4_00,                                                  ///< 4.00V
        V4_05,                                                  ///< 4.05V
        V4_10,                                                  ///< 4.10V
        V4_15,                                                  ///< 4.15V
        V4_20,                                                  ///< 4.20V
        V4_25,                                                  ///< 4.25V
        V4_30,                                                  ///< 4.30V
        V4_35,                                                  ///< 4.35V
        V4_40,                                                  ///< 4.40V
        V4_45,                                                  ///< 2.3V
        Default_1,                                              ///< Default (3.60V)
        Default_2,                                              ///< Default (3.60V)
        };

    /// \brief LED operating mode
    enum class LedMode : std::uint8_t
        {
        Error = 0,                                              ///< Error condition from Charger
        Charging,                                               ///< Charging indicator (On during charging)
        Host,                                                   ///< Driven from register LEDDRV_0_SET/CLR
        NotUsed,                                                ///< Not used
        };

    /// \brief Available LEDs
    enum class Led : std::uint8_t
        {
        LED_0 = 0,                                              ///< Error condition from Charger
        LED_1,                                                  ///< Charging indicator (On during charging)
        LED_2,                                                  ///< Driven from register LEDDRV_0_SET/CLR
        };

    /// \brief Avaliable GPIO Modes
    enum class GpioMode : std::uint8_t
        {
        GpiInput = 0,                                           ///< GPI input
        GpiLogic_1,                                             ///< GPI logic 1
        GpiLogic_0,                                             ///< GPI logic 0
        GpiEventRise,                                           ///< GPI rising edge event
        GpiEventFall,                                           ///< GPI falling edge event
        GpoIrq,                                                 ///< GPO interrupt
        GpoReset,                                               ///< GPO reset
        GpoPlw,                                                 ///< GPO power loss warning
        GpoLogic_1,                                             ///< GPO logic 1
        GpoLogic_0,                                             ///< GPO logic 0
        };

    /// \brief GPIO drive strength
    enum class GpioDrive : std::uint8_t
        {
        MA_1 = 0,                                               ///< 1mA
        MA_6,                                                   ///< 6mA
        };

    /// \brief Commands
    enum class Command : std::uint8_t
        {
        NoEffect = 0,
        Trigger,
        };

    /// \brief Error codes
    enum class Error : std::uint8_t
        {
        Success = 0,
        NoWire,
        CommandWriteFailed,
        InternalInvalidParameter,
        I2cReadShort,
        I2cReadRequest,
        I2cReadLong,
        Busy,
        NotMeasuring,
        Crc,
        Uninitialized,
        InvalidDataUnit,
        I2cWriteBufferFailed,
        I2cWriteFailed
        };

    /// \brief whether the board has a battery NTC thermistor.
    ///
    /// This is a property of the board, so begin() requires it.
    enum class Ntc : std::uint8_t
        {
        NotFitted,          ///< no thermistor: the charger ignores the NTC limits.
        Fitted,             ///< thermistor fitted: the charger applies the NTC limits.
        };

    /// \brief state of the meaurement engine
    enum class State : std::uint8_t
        {
        Uninitialized,      ///< this->begin() has never succeeded.
        End,                ///< this->begin() succeeded, followed by this->end()
        Initial,            ///< initial after begin [indeterminate]
        Idle,               ///< idle (not measuring)
        Triggered,          ///< continuous measurement running, no data available.
        Ready,              ///< continuous measurement running, data availble.
        };

private:
    /// \brief table of error messages
    ///
    /// \internal
    // this is internal -- centralize it but require that clients call the
    // public method (which centralizes the strings and the search)
    static constexpr const char * const m_szErrorMessages =
        "Success\0"
        "NoWire\0"
        "CommandWriteFailed\0"
        "InternalInvalidParameter\0"
        "I2cReadShort\0"
        "I2cReadRequest\0"
        "I2cReadLong\0"
        "Busy\0"
        "NotMeasuring\0"
        "Crc\0"
        "Uninitialized\0"
        "InvalidDataUnit\0"
        "I2cWriteBufferFailed\0"
        "I2cWriteFailed\0"
        ;

    /// \brief table of state names, '\0'-separated.
    ///
    /// \internal
    // this is internal -- centralize it but require that clients call the
    // public method (which centralizes the strings and the search)
    static constexpr const char * const m_szStateNames =
        "Uninitialized" "\0"
        "End"           "\0"
        "Initial"       "\0"
        "Idle"          "\0"
        "Triggered"     "\0"
        "Ready"         "\0"
        ;

public:
    /// \brief the constructor
    cNPM1300();
    virtual ~cNPM1300();

    ///
    /// \brief Initialize the NPM1300 IC and start operation.
    ///
    /// \param [in] ntc says whether the board has a battery NTC thermistor.
    ///     There is no default: on a board without one, the charger
    ///     refuses to charge unless it is told to ignore the NTC limits.
    ///     begin() applies the setting, and chargerOn() applies it again.
    ///
    /// \returns
    ///     \c true for success, \c false for failure (in which case the the last
    ///     error is set to the error reason).
    ///
    bool begin(Ntc ntc);

    /// \brief end operation;
    void end();

    /// \brief soft reset the sensor module
    ///
    /// \return
    ///     \c true for success, \c false for failure.
    ///
    bool reset();

    /// \brief convert a state value to a state name.
    static const char *getStateName(State s);

    /// \brief return name of current state.
    const char *getCurrentStateName() const
        {
        return getStateName(this->getState());
        }

    /// \brief return true if the driver is running.
    bool isRunning() const
        {
        return this->m_state > State::End;
        }

    ///
    /// \brief Change state of driver.
    ///
    /// \param [in] s is the new state
    ///
    /// \details
    ///     This function changes the recorded state of the driver instance.
    ///     When debugging, this might also log state changes; you can do that
    ///     by overriding this method in a derived class.
    ///
    virtual void setState(State s)
        {
        this->m_state = s;
        }

    /// \brief return current state of driver.
    State getState() const { return this->m_state; }

    /// \brief get the last error reported from this instance
    Error getLastError() const
        {
        return this->m_lastError;
        }

    /// \brief set the last error code.
    bool setLastError(Error e)
        {
        this->m_lastError = e;
        return e == Error::Success;
        }

    /// \brief return a string for a given error code.
    static const char *getErrorName(Error e);

    /// \brief return the name of the last error.
    const char *getLastErrorName() const
        {
        return getErrorName(this->m_lastError);
        }

    /// \brief measure Vbat
    ///
    /// \param [out] v is set to the battery voltage, in volts, on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure (I2C error, or the
    ///     conversion didn't finish in time). The last error is set in
    ///     case of error.
    ///
    bool measureVbat(float &voltage);

    /// \brief measure Vbus
    ///
    /// \param [out] v is set to the bus voltage, in volts, on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure (I2C error, or the
    ///     conversion didn't finish in time). The last error is set in
    ///     case of error.
    ///
    bool measureVbus(float &voltage);

    /// \brief measure Vsys
    ///
    /// \param [out] v is set to the system voltage, in volts, on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure (I2C error, or the
    ///     conversion didn't finish in time). The last error is set in
    ///     case of error.
    ///
    bool measureVsys(float &voltage);

    /// \brief measure die Temperature
    ///
    /// \param [out] t is set to the die temperature, in degrees C, on
    ///     success.
    ///
    /// \return
    ///     \c true for success, \c false for failure (I2C error, or the
    ///     conversion didn't finish in time). The last error is set in
    ///     case of error.
    ///
    bool measureTemperature(float &temperature);

    /// \brief get the status of Vbus
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getVbusStatus(uint8_t &status);

    /// \brief get the status of Buck
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getBuckStatus(uint8_t &status);

    /// \brief get the status of the load switches (LDSWSTATUS)
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getLoadSwitchStatus(uint8_t &status);

    /// \brief enter ship mode (TASKENTERSHIPMODE)
    ///
    /// All rails turn off, so on success the host loses power and this
    /// does not return. With VBUS present the nPM1300 doesn't enter ship
    /// mode until VBUS is removed. The nPM1300 leaves ship mode when VBUS
    /// rises above VBUSPOR or SHPHLD is held low; the host then starts
    /// from a power-on reset.
    ///
    /// \return
    ///     \c false if the write fails (the last error is set), or
    ///     if the host is still running (VBUS present).
    ///
    bool enterShipMode();

    /// \brief enable or disable the Buck 1
    ///
    /// \param [in] state true to enable and false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setBuck_1(bool state);

    /// \brief enable or disable the Buck 2
    ///
    /// \param [in] state true to enable and false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setBuck_2(bool state);

    /// \brief set the Vout voltage from Buck 1
    ///
    /// \param [in] voltage buck 1 voltage level.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setBuckVoltage_1(cNPM1300::Voltage voltage);

    /// \brief set the Vout voltage from Buck 2
    ///
    /// \param [in] voltage buck 2 voltage level.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setBuckVoltage_2(cNPM1300::Voltage voltage);

    /// \brief select the operating mode between LDO 1 and load switch 1
    ///
    /// \param [in] mode true to select LDO mode false to select Load Switch mode.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool selectLdoLoadSwitchMode_1(bool mode);

    /// \brief select the operating mode between LDO 2 and load switch 2
    ///
    /// \param [in] mode true to select LDO mode false to select Load Switch mode.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool selectLdoLoadSwitchMode_2(bool mode);

    /// \brief enable or disable the Load Switch 1
    ///
    /// \param [in] state true to enable and false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLoadSwitch_1(bool state);

    /// \brief enable or disable the Load Switch 2
    ///
    /// \param [in] state true to enable and false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLoadSwitch_2(bool state);

    /// \brief set the Vout voltage from LDO 1
    ///
    /// \param [in] voltage LDO 1 voltage level.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLdoVoltage_1(cNPM1300::Voltage voltage);

    /// \brief set the Vout voltage from LDO 2
    ///
    /// \param [in] voltage LDO 2 voltage level.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLdoVoltage_2(cNPM1300::Voltage voltage);

    /// \brief set LED operating mode
    ///
    /// \param [in] led LED in use.
    /// \param [in] mode is selected LED operating mode.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledSetMode(cNPM1300::Led led, cNPM1300::LedMode mode);

    /// \brief turn on selected LED
    ///
    /// \param [in] led LED in use.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledOn(cNPM1300::Led led);

    /// \brief turn off selected LED
    ///
    /// \param [in] led LED in use.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledOff(cNPM1300::Led led);

    /// \brief set LED state
    ///
    /// \param [in] led LED in use.
    /// \param [in] state is state of the LED (ON/OFF). True - ON, False - OFF
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledSetState(cNPM1300::Led led, bool state);

    /// \brief get LED state
    ///
    /// \param [in] led LED in use.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledGetState(cNPM1300::Led led);

    /// \brief toggle the selected LED state
    ///
    /// \param [in] led LED in use.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool ledToggle(cNPM1300::Led led);

    /// \brief select the between Vbus startup and Vbus limit
    ///
    /// \param [in] limit true to use Vbus limit current false to use Vbus startup current.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setTaskUpdateILimit(bool limit);

    /// \brief set the input current limit for Vbus limit
    ///
    /// \param [in] limit is the value to set between 100mA to 1500mA.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setInCurrentLimitVbus(cNPM1300::Current limit);

    /// \brief read the Vbus limit current
    ///
    /// \param [out] limit is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p limit is
    ///     unchanged on failure.
    ///
    bool readInCurrentLimitVbus(uint8_t &limit);

    /// \brief set the input current linit for Vbus
    ///
    /// \param [in] limit is the value to set between 100mA to 1500mA.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setStartupInCurrentLimitVbus(cNPM1300::Current limit);

    /// \brief read the Vbus startup current
    ///
    /// \param [out] limit is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p limit is
    ///     unchanged on failure.
    ///
    bool readStartupInCurrentLimitVbus(uint8_t &limit);

    /// \brief set the suspend mode for Vbus
    ///
    /// \param [in] mode true to set suspend mode false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool suspendVbus(bool mode);

    /// \brief get the Vbus comparator status
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getVbusComparatorStatus(uint8_t &status);

    /// \brief turn on the charger
    ///
    /// Applies the NTC setting given to begin(), then starts the charger.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool chargerOn();

    /// \brief return the NTC setting given to begin().
    Ntc getNtc() const { return this->m_ntc; }

    /// \brief enable recharging of battery once charged
    ///
    /// \param [in] enable true enable false to disable.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool enableRecharge(bool enable);

    /// \brief turn off the charger
    bool chargerOff();

    /// \brief get the battery charger normal termination voltage
    ///
    /// \param [out] vTerm is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p vTerm is
    ///     unchanged on failure.
    ///
    bool getVterm(uint8_t &vTerm);

    /// \brief set the battery charger normal termination voltage
    ///
    /// \param [in] vTerm is the value of voltage to be set.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setVterm(cNPM1300::Vterm vTerm);

    /// \brief get the battery charging status
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getChargeStatus(uint8_t &status);

    /// \brief get the charger error reason
    ///
    /// \param [out] reason is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p reason is
    ///     unchanged on failure.
    ///
    bool getErrorReason(uint8_t &reason);

    /// \brief get the charger error reason for external sensor
    ///
    /// \param [out] sensor is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p sensor is
    ///     unchanged on failure.
    ///
    bool getErrorSensor(uint8_t &sensor);

    /// \brief set charger current limit
    ///
    /// \param [in] limit is the current limit to be set, in mA: 32 to 800,
    ///     in 2 mA steps; values outside the range are clamped.
    ///
    /// If the charger is enabled, it is disabled while the registers
    /// change and then enabled again, so the new value takes effect at
    /// once (datasheet section 6.2.4). A disabled charger stays disabled.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setChargeCurrent(uint16_t limit);

    /// \brief set GPIO mode
    ///
    /// \param [in] mode is the available GPIO mode.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioMode(cNPM1300::GpioMode mode, uint8_t gpioNumber);

    /// \brief set GPIO drive strength configuration
    ///
    /// \param [in] current is the drive strength. 0 for 1mA and 1 for 6mA drive strength.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioDrive(cNPM1300::GpioDrive current, uint8_t gpioNumber);

    /// \brief set GPIO pull up enable configuration
    ///
    /// \param [in] enable set the pull up. false for disable and true for enable.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioPullUp(bool enable, uint8_t gpioNumber);

    /// \brief set GPIO pull down enable configuration
    ///
    /// \param [in] enable set the pull down. false for disable and true for enable.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioPullDown(bool enable, uint8_t gpioNumber);

    /// \brief set GPIO open drain configuration
    ///
    /// \param [in] enable set the open drain configuration. false for disable and true for enable.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioOpenDrain(bool enable, uint8_t gpioNumber);

    /// \brief set GPIO debounce configuration
    ///
    /// \param [in] enable set the debounce configuration. false for disable and true for enable.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setGpioDebounce(bool enable, uint8_t gpioNumber);

    /// \brief get GPIO status
    ///
    /// \param [out] status is set to the register value on success.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error; \p status is
    ///     unchanged on failure.
    ///
    bool getGpioStatus(uint8_t &status);

    /// \brief set load switch 1 GPIO control
    ///
    /// \param [in] invert inverts the sense of selected GPIO. false for normal sense and true for inverted sense.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLoadSwitchGpioControl_1(uint8_t gpioNumber, bool invert);

    /// \brief set load switch 2 GPIO control
    ///
    /// \param [in] invert inverts the sense of selected GPIO. false for normal sense and true for inverted sense.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setLoadSwitchGpioControl_2(uint8_t gpioNumber, bool invert);

    /// \brief set GPIO control for selected buck regulator
    ///
    /// \param [in] buckNumber refers to the buck regulator that needed to be configured.
    /// \param [in] gpioNumber refers to the GPIO that needed to be configured.
    /// \param [in] invert inverts the sense of selected GPIO. false for normal sense and true for inverted sense.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool setBuckGpioControl(uint8_t buckNumber, uint8_t gpioNumber, bool invert);

    /// \brief Return true if the debug is enabled
    static constexpr bool isDebug() { return kfDebug; }

protected:
    ///
    /// \brief read a byte from a given register.
    ///
    /// \param [in] reg indicates the register to be read.
    /// \param [out] value is set to the value read (if no error).
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool readRegister(cNPM1300::Register reg, std::uint8_t &value);

    ///
    /// \brief read a series of bytes starting with a given register.
    ///
    /// \param [in] reg indicates the starting register to be read.
    /// \param [out] pBuffer points to the buffer to receive the data
    /// \param [in] nBuffer is the number of bytes to read.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool readRegisters(cNPM1300::Register reg, std::uint8_t *pBuffer, size_t nBuffer);

    ///
    /// \brief Write a byte to a given register.
    ///
    /// \param [in] reg selects the register to write
    /// \param [in] value is the value to be written.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool writeRegister(cNPM1300::Register reg, std::uint8_t value);

    ///
    /// \brief wait for an ADC conversion to finish.
    ///
    /// \param [in] eventBit selects the EVENTADC*RDY bit (in
    ///     EVENTSADCSET / EVENTSADCCLR) for the conversion that was just
    ///     triggered; see datasheet section 7.7.1.2.
    ///
    /// \return
    ///     \c true once the bit is set (and it's been cleared again, so
    ///     the next conversion starts clean); \c false on an I2C error,
    ///     or if the bit doesn't get set within a generous timeout, in
    ///     which case Error::NotMeasuring is set as the last error.
    ///
    bool waitForAdcReady(std::uint8_t eventBit);

    ///
    /// \brief Make sure the driver is running
    ///
    /// If not running, set last error to Error::Uninitialized, and return \c false.
    /// Otherwise return \c true.
    ///
    /// Normally used in the following pattern:
    ///
    /// \code
    ///     if (! this->checkRunning())
    ///         return false;
    ///     // otherwise do some work...
    /// \endcode
    ///
    bool checkRunning()
        {
        if (! this->isRunning())
            return this->setLastError(Error::Uninitialized);
        else
            return true;
        }

private:
    /// \brief tell the charger whether to apply the NTC limits, from m_ntc.
    bool applyNtc();

    TwoWire *m_wire {nullptr};                              ///< pointer to bus to be used for this device
    std::uint32_t m_tReady {0};                             ///< estimated time next measurement will be ready (millis)
    Address m_address {Address::NPM1300};                   ///< I2C address to be used
    Pin_t m_pinReady {-1};                                  ///< alert pin, or -1 if none.
    Error m_lastError {Error::Success};                     ///< last error.
    State m_state                                           ///< current state
        { State::Uninitialized };                           ///< initially not yet started.
    bool ledState[3] = {false};                             ///< LEDDRV state
    Ntc m_ntc {Ntc::NotFitted};                             ///< set by begin()
    };

} // end namespace McciCatenaNpm1300

#endif /* _MCCI_CATENA_NPM1300_H_ */