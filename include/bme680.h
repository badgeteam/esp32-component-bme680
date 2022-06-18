#pragma once

#include <esp_err.h>
#include <stdint.h>

#define BME680_CHIP_ID 0x61

/** Calibration data */
typedef struct bme680_calib {
    uint16_t t1;
    uint16_t t2;
    uint8_t  t3;
    uint16_t p1;
    uint16_t p2;
    uint8_t  p3;
    uint16_t p4;
    uint16_t p5;
    uint8_t  p6;
    uint8_t  p7;
    uint16_t p8;
    uint16_t p9;
    uint8_t  p10;
    uint16_t h1;
    uint16_t h2;
    uint8_t  h3;
    uint8_t  h4;
    uint8_t  h5;
    uint8_t  h6;
    uint8_t  h7;
    uint8_t  g1;
    uint16_t g2;
    uint8_t  g3;
    uint8_t  res_heat_range;
    uint8_t  res_heat_val;
} bme680_calib_t;

typedef struct BME680 {
    int              i2c_bus;
    int              i2c_address;
    bme680_calib_t   calibration_data;
} BME680;

typedef struct bme680_status {
    bool new_data;               /** Flag - the measured data is stored into the output data registers. */
    bool gas_measuring;          /** Flag - gas measurement ongoing. */
    bool measuring;              /** Flag - temperature, pressure, humidity or gas measurement ongoing. */
    uint8_t gas_measuring_index; /** Current gas heater set setpoint index. */
    bool gas_valid;              /** Flag - gas value measurement finished. */
    bool heater_stable;          /** Flag - heater temperature is stable for target heater resistance. */
} bme680_status_t;

typedef enum {
    BME680_MEAS_TEMPERATURE         = 0,
    BME680_MEAS_HUMIDITY            = 1,
    BME680_MEAS_PRESSURE            = 2,
    BME680_MEAS_GAS_RESISTANCE      = 3,
} bme680_meas_type_t;

typedef enum {
    BME680_MODE_SLEEP   = 0,
    BME680_MEAS_FORCED  = 1,
} bme680_mode_t;

typedef enum {
    BME680_OVERSAMPLING_SKIP    = 0,
    BME680_OVERSAMPLING_X1      = 1,
    BME680_OVERSAMPLING_X2      = 2,
    BME680_OVERSAMPLING_X4      = 3,
    BME680_OVERSAMPLING_X8      = 4,
    BME680_OVERSAMPLING_X16     = 5,
} bme680_oversampling_t;

typedef enum {
    BME680_HEATER_PROFILE_0 = 0,
    BME680_HEATER_PROFILE_1 = 1,
    BME680_HEATER_PROFILE_2 = 2,
    BME680_HEATER_PROFILE_3 = 3,
    BME680_HEATER_PROFILE_4 = 4,
    BME680_HEATER_PROFILE_5 = 5,
    BME680_HEATER_PROFILE_6 = 6,
    BME680_HEATER_PROFILE_7 = 7,
    BME680_HEATER_PROFILE_8 = 8,
    BME680_HEATER_PROFILE_9 = 9,
} bme680_heater_profile_t;

typedef enum {
    BME680_GAS_WAIT_FACTOR_1  = 0,
    BME680_GAS_WAIT_FACTOR_4  = 1,
    BME680_GAS_WAIT_FACTOR_16 = 2,
    BME680_GAS_WAIT_FACTOR_64 = 3,
} bme680_gas_wait_factor_t;

esp_err_t bme680_init(BME680* device);
esp_err_t bme680_deinit(BME680* device);
esp_err_t bme680_check_id(BME680* device);
esp_err_t bme680_reset(BME680* device);
esp_err_t bme680_set_mode(BME680* device, bme680_mode_t mode);
esp_err_t bme680_set_oversampling(BME680* device, bme680_oversampling_t os_humidity,
                                                    bme680_oversampling_t os_temperature,
                                                    bme680_oversampling_t os_pressure);
esp_err_t bme680_run_gas(BME680* device);
esp_err_t bme680_set_heater_profile(BME680* device, bme680_heater_profile_t profile);
esp_err_t bme680_set_heater_resistance(BME680* device, bme680_heater_profile_t profile, uint8_t resistance);
esp_err_t bme680_set_gas_wait(BME680* device, bme680_heater_profile_t profile, bme680_gas_wait_factor_t factor, uint8_t time_ms);

esp_err_t bme680_get_status(BME680* device, bme680_status_t* status);

/**
 * Gets compensated temperature measurement in Celsius.
 * */
esp_err_t bme680_get_temperature(BME680* device, double* temperature);

/**
 * Gets compensated relative humidity measurement in percent.
 * Needs temperature (in Celsius) for evaluation of compensated measurement.
 * */
esp_err_t bme680_get_humidity(BME680* device, double* humidity, double temperature);

/**
 * Gets compensated pressure measurement in Pascals.
 * Needs temperature (in Celsius) for evaluation of compensated measurement.
 * */
esp_err_t bme680_get_pressure(BME680* device, double* pressure, double temperature);

/**
 * Gets compensated gas sensor resistance measurement in Ohms.
 * Needs temperature (in Celsius) for evaluation of compensated measurement.
 * */
esp_err_t bme680_get_gas_resistance(BME680* device, double* gas_resistance, double temperature);
