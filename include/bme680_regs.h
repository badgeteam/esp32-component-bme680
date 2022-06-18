#pragma once

#define BME680_REG_STATUS       0x73u
#define BME680_REG_RESET        0xE0u
#define BME680_REG_CHIP_ID      0xD0u
#define BME680_REG_CONFIG       0x75u
#define BME680_REG_CTRL_MEAS    0x74u
#define BME680_REG_CTRL_HUM     0x72u
#define BME680_REG_CTRL_GAS1    0x71u
#define BME680_REG_CTRL_GAS0    0x70u
#define BME680_REG_GAS_WAIT0    0x64u
#define BME680_REG_RES_HEAT0    0x5Au
#define BME680_REG_IDAC_HEAT0   0x50u
#define BME680_REG_GAS_R_LSB    0x2Bu
#define BME680_REG_GAS_R_MSB    0x2Au
#define BME680_REG_HUM_LSB      0x26u
#define BME680_REG_HUM_MSB      0x25u
#define BME680_REG_TEMP_XLSB    0x24u
#define BME680_REG_TEMP_LSB     0x23u
#define BME680_REG_TEMP_MSB     0x22u
#define BME680_REG_PRESS_XLSB   0x21u
#define BME680_REG_PRESS_LSB    0x20u
#define BME680_REG_PRESS_MSB    0x1Fu
#define BME680_REG_EAS_STATUS0  0x1Du

#define BME680_REG_PAR_T1_LSB   0xE9u
#define BME680_REG_PAR_T1_MSB   0xEAu
#define BME680_REG_PAR_T2_LSB   0x8Au
#define BME680_REG_PAR_T2_MSB   0x8Bu
#define BME680_REG_PAR_T3       0x8Cu

#define BME680_REG_PAR_P1_LSB   0x8Eu
#define BME680_REG_PAR_P1_MSB   0x8Fu
#define BME680_REG_PAR_P2_LSB   0x90u
#define BME680_REG_PAR_P2_MSB   0x91u
#define BME680_REG_PAR_P3       0x92u
#define BME680_REG_PAR_P4_LSB   0x94u
#define BME680_REG_PAR_P4_MSB   0x95u
#define BME680_REG_PAR_P5_LSB   0x96u
#define BME680_REG_PAR_P5_MSB   0x97u
#define BME680_REG_PAR_P6       0x99u
#define BME680_REG_PAR_P7       0x98u
#define BME680_REG_PAR_P8_LSB   0x9Cu
#define BME680_REG_PAR_P8_MSB   0x9Du
#define BME680_REG_PAR_P9_LSB   0x9Eu
#define BME680_REG_PAR_P9_MSB   0x9Fu
#define BME680_REG_PAR_P10      0xA0u

#define BME680_REG_PAR_H1_LSB   0xE2u
#define BME680_REG_PAR_H1_MSB   0xE3u
#define BME680_REG_PAR_H2_LSB   0xE2u
#define BME680_REG_PAR_H2_MSB   0xE1u
#define BME680_REG_PAR_H3       0xE4u
#define BME680_REG_PAR_H4       0xE5u
#define BME680_REG_PAR_H5       0xE6u
#define BME680_REG_PAR_H6       0xE7u
#define BME680_REG_PAR_H7       0xE8u

#define BME680_REG_PAR_G1           0xEDu
#define BME680_REG_PAR_G2_LSB       0xEBu
#define BME680_REG_PAR_G2_MSB       0xECu
#define BME680_REG_PAR_G3           0xEEu
#define BME680_REG_RES_HEAT_RANGE   0x02u
#define BME680_REG_RES_HEAT_VAL     0x00u
#define BME680_REG_RANGE_ERR        0x04u