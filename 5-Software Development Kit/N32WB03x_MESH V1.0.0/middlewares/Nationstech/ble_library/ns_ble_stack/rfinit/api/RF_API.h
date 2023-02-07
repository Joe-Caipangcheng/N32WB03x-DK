/********************************************************************
                 Nationz Technologies Co. Ltd
        Copyright (c) 2000��2016 All Rights Reserved
*********************************************************************

API Header for Cooper RF Control
Author: zengjun

Version 0.2

Version History
V0.1    2016.12.01            Original
V0.2    2016.12.09            Functions Added

********************************************************************/
#ifndef _RF_API_H
#define _RF_API_H

#include "TypeDefine.h"

/*==============================================================================
Power Supply Selection

This function is to select VDCDC/VDCDC-RF from DC-DC or external voltage source

<input parameter>
None
==============================================================================*/
void PwrSel(UINT8 Mode);

/*==============================================================================
Enable Bus Clock for the related modules in Setup_Rf function

This function is to enable bus clock which are usded in Setup_Rf function

<input parameter>
None
==============================================================================*/
void EnClock(void);

/*==============================================================================
Disable Bus Clock for the related modules in Setup_Rf function

This function is to disable bus clock which are usded in Setup_Rf function

<input parameter>
None
==============================================================================*/
void DisableClock(void);

/*==============================================================================
Analog LDOs Control

This function is to control the internal analog LDOs.

<input parameter>
Mode, En

Mode
1: Controlled by logic
0: Manually control through software

En <LDO9, LDO7, LDO6, LDO5, LDO4, LDO3, LDO2, LDO1>
1: Enable the respective LDO, invalid under logic control mode
0: Disable the respective LDO, invalid under logic control mode

LDO9: IQ_ADC Asynchronous SAR logic
LDO7: XTAL_32M
LDO6: PA(partial)
LDO5: PA/LNA logic, LS
LDO4: LNA(partial),
LDO3: ABB, Mixer(TIA), IQ-ADC(partial)
LDO2: DAC(partial), PFD/CP/Gain control bias/LD/Divider/AFC
LDO1: DAC(partial), LO, PLL LF/Gain Control, VCO
==============================================================================*/
void AnalogLdoCtrl(UINT8 Mode, UINT8 En);

/*==============================================================================
Initialization

This function is to initialize RF registers according to measurement results, in
order to optimize performance or reduce current supply

<input parameter>
None
==============================================================================*/
void Init_Rf(void);

/*==============================================================================
Two Point Modulation Calibration

This function is to calibrate two point modulation in order to meet the requirement
from protocol.

<input parameter>
Mode
1: With GP-ADC
0: Without GP-ADC
CalFrequency
PLL Frequency while doing calibration
==============================================================================*/
void Cal_Modulation(UINT8 Mode, UINT8 Alg, UINT16 CalFrequency, UINT16 TwoPara);

/*==============================================================================
Rx ABB Bandwidth Calibration

This function is to calibrate ABB filter in order to get correct bandwidth.

<input parameter>
None
==============================================================================*/
void Cal_Filter(UINT8 Mode);

/*==============================================================================
RC32MHz Oscillator Frequency Calibration

This function is to calibrate RC32MHz Oscillator frequency.

<input parameter>
Accuracy: determine the calibration accuracy
==============================================================================*/
void Cal_RC32M(UINT8 Accuracy);

/*==============================================================================
RC32KHz Oscillator Frequency Calibration

This function is to calibrate RC32KHz Oscillator frequency.

<input parameter>
TrimCycle in 32KHz cycle: 0 for 10 trmming cycles, 1 for 100 trmming cycles
Accuracy: determine the calibration accuracy
==============================================================================*/
void Cal_RC32K(UINT8 TrimCycle, UINT8 Accuracy);

/*==============================================================================
GP-ADC Input Source    Selection

This function is to select input source for GP-ADC.

<input parameter>
Source
1: PLL LPF
0: Temperature Sensor
==============================================================================*/
void InputSel_GPADC(UINT8 Source);

/*==============================================================================
GP-ADC Enable/Disable

This function is to enable or disable the internal GP-ADC.

<input parameter>
En
1: Enable
0: Disable
==============================================================================*/
void En_GPADC(UINT8 En);

/*==============================================================================
Temperature Sensor Enable/Disable

This function is to enable or disable the internal temperature sensor.

<input parameter>
En
1: Enable
0: Disable
==============================================================================*/
void En_TS(UINT8 En);

/*==============================================================================
Temperature-Voltage Look Up Table

This function is to get temperature by voltage in mV.

<input parameter>
Volt
==============================================================================*/
UINT8 Lut_TempVolt(UINT16 Volt);

/*==============================================================================
Temperature Acquisition in Celsius

This function is to obtain temperature in celsius.

<input parameter>
None
==============================================================================*/
UINT8 Get_Temp(void);

/*==============================================================================
PLL LPF voltage Acquisition in mV

This function is to obtain PLL LPF voltage in mV.

<input parameter>
None
==============================================================================*/
UINT16 Get_Volt(void);

/*==============================================================================
Tx Output Power Adjustment According to RSSI in last event

This function is to adjust PA output power for power consumption reduction.

<input parameter>
RSSI
==============================================================================*/
void TxPwrSet(UINT8 RSSI);

/*==============================================================================
Manually open LDO_LNA test port for bug clear

This function is to manually open LDO_LNA test port for bug clear.

<input parameter>
None
==============================================================================*/
void LdoLnaTestEn(void);

/*==============================================================================
Set PLL Operation Mode

This function is to set PLL operation mode.

<input parameter>
Mode

Mode
0: Controlled by hardware
1: Controlled by software
==============================================================================*/
void PllEnModeSel(UINT8 Mode);

/*==============================================================================
Set PLL Frequency by software

This function is to set PLL frequency through software.

<input parameter>
Mode, Frequency

Mode
0: Shutdown AFC and PLL
1: Bypass AFC, execute PLL loop locking
2: Only AFC, shutdown PLL loop locking
3: Run AFC and PLL loop locking

Frequency
in MHz
==============================================================================*/
void SetPllFreq(UINT8 Mode, UINT16 Frequency);

/*==============================================================================
RF Initialization

This function is the main function to initialize RF

<input parameter>
None
==============================================================================*/
void Setup_Rf(void);

void Cal_LdoRet(UINT16 VoltLdoRetExp,UINT8 Accuracy);

void AGC_Scaling_Configurable(void);

UINT32 Cal_SleepCycleRC32K(void);

UINT32 cal_cycles_rc32k(uint8_t mode);

void rf_set_modem_register(void);

void rf_set_agc_value_befor_sleep(void);

void rf_set_rfc_pa_value(void);

void rf_set_80_register(void);

UINT32 start_cal_rc32k(uint8_t mode);

UINT32 comp_rc32k_cnt(uint8_t mode);

void init_wakeup_register(void);

#endif
