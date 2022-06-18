/**
 * Driver for Bosch BME680 environmental sensor.
 *
 * Copyright (c) 2022 Nicolai Electronics, Andrejs Bondarevs.
 *
 * SPDX-License-Identifier: MIT
 */

#include <sdkconfig.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "bme680.h"
#include "bme680_regs.h"
#include "managed_i2c.h"

static const char *TAG = "BME680";

/** Lookup table 1 for gas sensor resistance calculation. */
static const double gas_range_array1[16] = {
    [0] = 1.0,
    [1] = 1.0,
    [2] = 1.0,
    [3] = 1.0,
    [4] = 1.0,
    [5] = 0.99,
    [6] = 1.0,
    [7] = 0.992,
    [8] = 1.0,
    [9] = 1.0,
    [10] = 0.998,
    [11] = 0.995,
    [12] = 1.0,
    [13] = 0.99,
    [14] = 1.0,
    [15] = 1.0,
};

/** Lookup table 2 for gas sensor resistance calculation. */
static const double gas_range_array2[16] = {
    [0] = 8000000,
    [1] = 4000000,
    [2] = 2000000,
    [3] = 1000000,
    [4] = 499500.4995,
    [5] = 248262.1648,
    [6] = 125000,
    [7] = 63004.03226,
    [8] = 31281.28128,
    [9] = 15625,
    [10] = 7812.5,
    [11] = 3906.25,
    [12] = 1953.125,
    [13] = 976.5625,
    [14] = 488.28125,
    [15] = 244.140625,
};

/** Gets raw ADC value for pressure sensor. */
static esp_err_t s_get_pressure_raw(BME680* device, uint32_t* pressure) {
    uint8_t data[3] = {0,};
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PRESS_MSB, data, 3);
    if (res == ESP_OK) {
        *pressure = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    }
    return res;
}

/** Gets raw ADC value for temperature sensor. */
static esp_err_t s_get_temperature_raw(BME680* device, uint32_t* temperature) {
    uint8_t data[3] = {0,};
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_TEMP_MSB, data, 3);
    if (res == ESP_OK) {
        *temperature = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    }
    return res;
}

/** Gets raw ADC value for relative humidity sensor. */
static esp_err_t s_get_humidity_raw(BME680* device, uint16_t* humidity) {
    uint8_t data[2] = {0,};
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_HUM_MSB, data, 2);
    if (res == ESP_OK) {
        *humidity = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    }
    return res;
}

/** Gets raw ADC value for gas sensor. */
static esp_err_t s_get_gas_resistance_raw(BME680* device, uint16_t* gas_resistance) {
    uint8_t data[2] = {0,};
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_GAS_R_MSB, data, 2);
    if (res == ESP_OK) {
        *gas_resistance = ((uint16_t)data[0] << 2) | ((uint16_t)data[1] >> 6);
    }
    return res;
}

/** Gets ADC range of the measured gas sensor resistance. */
static esp_err_t s_get_gas_range_raw(BME680* device, uint16_t* gas_range) {
    uint8_t data = 0;
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_GAS_R_LSB, &data, 1);
    if (res == ESP_OK) {
        *gas_range = data & (uint16_t)0b00001111;
    }
    return res;
}

/** Gets calibration parameter for gas sensor resistance calculation. */
static esp_err_t s_get_gas_range_error(BME680* device, uint8_t* gas_range_error) {
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_RANGE_ERR, gas_range_error, 1);
    return res;
}

/** Gets calibration parameters.
 * Parameters are constant and need to be read only once.
*/
static esp_err_t s_get_calibration(BME680* device) {
    uint8_t data[3] = {0,};
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_T1_LSB, data, 2);
    if (res == ESP_OK) {
        device->calibration_data.t1 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_T2_LSB, data, 2);
        device->calibration_data.t2 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_T3, &device->calibration_data.t3, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P1_LSB, data, 2);
        device->calibration_data.p1 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P2_LSB, data, 2);
        device->calibration_data.p2 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P3, &device->calibration_data.p3, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P4_LSB, data, 2);
        device->calibration_data.p4 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P5_LSB, data, 2);
        device->calibration_data.p5 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P6, &device->calibration_data.p6, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P7, &device->calibration_data.p7, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P8_LSB, data, 2);
        device->calibration_data.p8 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P9_LSB, data, 2);
        device->calibration_data.p9 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_P10, &device->calibration_data.p10, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H2_MSB, data, 3);
        device->calibration_data.h1 = ((uint16_t)data[2] << 4) | ((uint16_t)data[1] & 0b00001111);
        device->calibration_data.h2 = ((uint16_t)data[0] << 4) | ((uint16_t)data[1] >> 4);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H3, &device->calibration_data.h3, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H4, &device->calibration_data.h4, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H5, &device->calibration_data.h5, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H6, &device->calibration_data.h6, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_H7, &device->calibration_data.h7, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_G1, &device->calibration_data.g1, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_G2_LSB, data, 2);
        device->calibration_data.g2 = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_PAR_G3, &device->calibration_data.g3, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_RES_HEAT_RANGE, data, 1);
        device->calibration_data.res_heat_range = ((uint16_t)data[0] & (uint16_t)0b00110000) >> 4;
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_RES_HEAT_VAL, &device->calibration_data.res_heat_val, 1);
    }

    return res;
}

/** Calculates compensated temperature measurement */
double s_eval_temperature(BME680* device, uint32_t temperature_raw) {
    double var1 = ((double)temperature_raw / 16384.0 - (double)device->calibration_data.t1 / 1024.0) *
                   (double)device->calibration_data.t2;

    double var2 = ((double)temperature_raw / 131072.0 - (double)device->calibration_data.t1 / 8192.0) *
                  ((double)temperature_raw / 131072.0 - (double)device->calibration_data.t1 / 8192.0) *
                  (double)device->calibration_data.t3 * 16.0;

    double temperature = (var1 + var2) / 5120.0;

    return temperature;
}

/** Calculates compensated pressure measurement */
double s_eval_pressure(BME680* device, uint32_t pressure_raw, double temperature) {
    double var1, var2, var3, pressure;

    var1 = ((double)temperature * 2560.0) - 64000.0;
    var2 = var1 * var1 * ((double)device->calibration_data.p6 / 131072.0);
    var2 = var2 + (var1 * (double)device->calibration_data.p5 * 2.0);
    var2 = (var2 / 4.0) + ((double)device->calibration_data.p4 * 65536.0);
    var1 = ((((double)device->calibration_data.p3 * var1 * var1) / 16384.0) +
            ((double)device->calibration_data.p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (double)device->calibration_data.p1;
    pressure = 1048576.0 - (double)pressure_raw;
    pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1;
    var1 = ((double)device->calibration_data.p9 * pressure * pressure) / 2147483648.0;
    var2 = pressure * ((double)device->calibration_data.p8 / 32768.0);
    var3 = (pressure / 256.0) * (pressure / 256.0) * (pressure / 256.0) * (device->calibration_data.p10 / 131072.0);
    pressure = pressure + (var1 + var2 + var3 + ((double)device->calibration_data.p7 * 128.0)) / 16.0;

    return pressure;
}

/** Calculates compensated relative humidity measurement */
double s_eval_humidity(BME680* device, uint16_t humidity_raw, double temperature) {
    double var1, var2, var3, var4, humidity;

    var1 = (double)humidity_raw - (((double)device->calibration_data.h1 * 16.0) +
            (((double)device->calibration_data.h3 / 2.0) * temperature));
    var2 = var1 * (((double)device->calibration_data.h2 / 262144.0) *
            (1.0 + (((double)device->calibration_data.h4 / 16384.0) * temperature) +
            (((double)device->calibration_data.h5 / 1048576.0) * temperature * temperature)));
    var3 = (double)device->calibration_data.h6 / 16384.0;
    var4 = (double)device->calibration_data.h7 / 2097152.0;
    humidity = var2 + ((var3 + (var4 * temperature)) * var2 * var2);

    return humidity;
}

/** Calculates compensated gas heater resistance measurement */
uint8_t s_eval_heater_resistance(BME680* device, uint32_t humidity_raw, uint16_t temperature, uint16_t temperature_target) {
    int32_t var1, var2, var3, var4, var5, res_heat_x100, res_heat;

    var1 = (((int32_t)temperature * (int32_t)device->calibration_data.g3) / 10) << 8;
    var2 = (device->calibration_data.g1 + 784) *
            ((((((int32_t)device->calibration_data.g2 + 154009) * temperature_target * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 >> 1);
    var4 = (var3 / ((int32_t)device->calibration_data.res_heat_range + 4));
    var5 = (131 * (int32_t)device->calibration_data.res_heat_val) + 65536;
    res_heat_x100 = (int32_t)((((int32_t)var4 / (int32_t)var5) - 250) * 34);
    res_heat = (uint8_t)((res_heat_x100 + 50) / 100);

    return res_heat;
}

/** Calculates compensated gas sensor resistance measurement */
double s_eval_gas_resistance(BME680* device, uint32_t gas_resistance_raw, uint16_t gas_range, uint8_t range_error) {
    double var1, gas_resistance;

    var1 = (1340.0 + 5.0 * range_error) * gas_range_array1[gas_range];
    gas_resistance = var1 * gas_range_array2[gas_range] / (gas_resistance_raw - 512.0 + var1);

    return gas_resistance;
}

esp_err_t bme680_init(BME680* device) {
    esp_err_t res;
    res = bme680_reset(device);
    if (res == ESP_OK) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        res = bme680_check_id(device);
    }

    if (res == ESP_OK) {
        s_get_calibration(device);
    }

    return res;
}

esp_err_t bme680_deinit(BME680* device) {
    return bme680_reset(device);
}

esp_err_t bme680_check_id(BME680* device) {
    uint8_t chip_id;
    esp_err_t res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CHIP_ID, &chip_id, 1);
    if ((res == ESP_OK) && (chip_id != BME680_CHIP_ID)) {
        ESP_LOGE(TAG, "Unexpected chip id value 0x%02X, expected 0x%02X", chip_id, BME680_CHIP_ID);
        res = ESP_FAIL;
    }
    return res;
}

esp_err_t bme680_reset(BME680* device) {
    uint8_t value = 0xFF;
    esp_err_t res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_RESET, &value, 1);
    return res;
}

esp_err_t bme680_set_mode(BME680* device, bme680_mode_t mode) {
    uint8_t reg_val;
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_MEAS, &reg_val, 1);
    if (res == ESP_OK) {
        reg_val = reg_val | ((uint8_t)mode);
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_MEAS, &reg_val, 1);
    }
    return res;
}

esp_err_t bme680_set_oversampling(BME680* device, bme680_oversampling_t os_humidity,
                                                    bme680_oversampling_t os_temperature,
                                                    bme680_oversampling_t os_pressure) {
    uint8_t reg_val;
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_HUM, &reg_val, 1);
    if (res == ESP_OK) {
        reg_val = reg_val | (uint8_t)os_humidity;
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_HUM, &reg_val, 1);
    }

    if (res == ESP_OK) {
        res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_MEAS, &reg_val, 1);
    }

    if (res == ESP_OK) {
        reg_val = reg_val | ((uint8_t)os_temperature << 5) | ((uint8_t)os_pressure << 2);
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_MEAS, &reg_val, 1);
    }

    return res;
}

esp_err_t bme680_run_gas(BME680* device) {
    uint8_t reg_val;
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_GAS1, &reg_val, 1);
    if (res == ESP_OK) {
        reg_val = reg_val | (1 << 4);
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_GAS1, &reg_val, 1);
    }
    return res;
}

esp_err_t bme680_set_heater_profile(BME680* device, bme680_heater_profile_t profile) {
    uint8_t reg_val;
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_GAS1, &reg_val, 1);
    if (res == ESP_OK) {
        reg_val = reg_val | (uint8_t)profile;
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BME680_REG_CTRL_GAS1, &reg_val, 1);
    }
    return res;
}

esp_err_t bme680_set_heater_resistance(BME680* device, bme680_heater_profile_t profile, uint8_t resistance) {
    uint8_t reg = BME680_REG_RES_HEAT0 + (uint8_t)profile;
    esp_err_t res;

    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, reg, &resistance, 1);

    return res;
}

esp_err_t bme680_set_gas_wait(BME680* device, bme680_heater_profile_t profile, bme680_gas_wait_factor_t factor, uint8_t time_ms) {
    uint8_t reg = BME680_REG_GAS_WAIT0 + (uint8_t)profile;
    uint8_t reg_val = ((uint8_t)factor << 6) | (time_ms & 0b111111);
    esp_err_t res;

    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, reg, &reg_val, 1);

    return res;
}

esp_err_t bme680_get_status(BME680* device, bme680_status_t* status) {
    uint8_t data;
    esp_err_t res;

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_EAS_STATUS0, &data, 1);
    if (res == ESP_OK) {
        status->new_data = ((0b10000000 & data) != 0);
        status->gas_measuring = ((0b01000000 & data) != 0);
        status->measuring = ((0b00100000 & data) != 0);
        status->gas_measuring_index = 0b1111 & data;
    }

    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BME680_REG_GAS_R_LSB, &data, 1);
    if (res == ESP_OK) {
        status->gas_valid = ((0b00100000 & data) != 0);
        status->heater_stable = ((0b00010000 & data) != 0);
    }

    return res;
}

esp_err_t bme680_get_temperature(BME680* device, double* temperature) {
    uint32_t temperature_raw;
    esp_err_t res;

    res = s_get_temperature_raw(device, &temperature_raw);
    if (res == ESP_OK) {
        *temperature = s_eval_temperature(device, temperature_raw);
    }
    return res;
}

esp_err_t bme680_get_humidity(BME680* device, double* humidity, double temperature) {
    uint16_t humidity_raw;
    esp_err_t res;

    res = s_get_humidity_raw(device, &humidity_raw);
    if (res == ESP_OK) {
        *humidity = s_eval_humidity(device, humidity_raw, temperature);
    }

    return res;
}

esp_err_t bme680_get_pressure(BME680* device, double* pressure, double temperature) {
    uint32_t pressure_raw;
    esp_err_t res;

    res = s_get_pressure_raw(device, &pressure_raw);
    if (res == ESP_OK) {
        *pressure = s_eval_pressure(device, pressure_raw, temperature);
    }

    return res;
}

esp_err_t bme680_get_gas_resistance(BME680* device, double* gas_resistance, double temperature) {
    uint16_t gas_resistance_raw, gas_range_raw;
    uint8_t gas_range_error;
    esp_err_t res;

    res = s_get_gas_resistance_raw(device, &gas_resistance_raw);
    if (res == ESP_OK) {
        res = s_get_gas_range_raw(device, &gas_range_raw);
    }
    if (res == ESP_OK) {
        res = s_get_gas_range_error(device, &gas_range_error);
    }
    if (res == ESP_OK) {
        *gas_resistance = s_eval_gas_resistance(device, gas_resistance_raw, gas_range_raw, gas_range_error);
    }

    return res;
}
