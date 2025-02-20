/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"

#include "../bh1750_defs.h"
#include "../htu21_defs.h"
#include "../ulp_error.h"

#define LP_I2C_TRANS_TIMEOUT_CYCLES 5000
#define LP_I2C_TRANS_WAIT_FOREVER   -1

#define LUX_THRESHOLD_LOW           5
#define LUX_THRESHOLD_HIGH          1000


static uint32_t sensor_on = 0;
static uint32_t res_update_done = 0;
volatile uint32_t lux = 0;
volatile uint32_t humidity = 0;
volatile uint32_t temperature = 0;
volatile uint32_t error_no = ULP_NOT_READY;
volatile uint32_t work_around = 0;

static void bh1750_read()
{
    uint8_t data_wr = 0;
    esp_err_t ret = ESP_OK;

    if (!sensor_on) {
        /* Power ON the sensor */
        data_wr = BH1750_POWER_ON;
        ret = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, BH1750_I2C_ADDR, &data_wr, sizeof(data_wr), LP_I2C_TRANS_WAIT_FOREVER);
        if (ret != ESP_OK) {
            // Bail and try again
            error_no = ULP_ERROR_BH1750_POWER_ON;
            return;
        }
        sensor_on = 1;
    }

    if (!res_update_done) {
        data_wr = EXAMPLE_RES_MODE;
        ret = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, BH1750_I2C_ADDR, &data_wr, sizeof(data_wr), LP_I2C_TRANS_WAIT_FOREVER);
        if (ret != ESP_OK) {
            // Bail and try again
            error_no = ULP_ERROR_BH1750_RES_UPDATE;
            return;
        }
        res_update_done = 1;
    }

    uint8_t data_rd[2];
    ret = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, BH1750_I2C_ADDR, data_rd, sizeof(data_rd), LP_I2C_TRANS_TIMEOUT_CYCLES);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        error_no = ULP_ERROR_BH1750_READ;
        return;
    }

    /* Calculate light intensity value */
    uint16_t level = ((data_rd[0] << 8) | data_rd[1]);
    lux = (level * 10) / 12;

    /* Wakeup main CPU if the Lux breaches the thresholds */
    if (lux < LUX_THRESHOLD_LOW || lux > LUX_THRESHOLD_HIGH) {
        ulp_lp_core_wakeup_main_processor();
    }
}



static void htu21_read()
{
    uint8_t buf[3] = {0};
    esp_err_t ret = ESP_OK;



    // send temperature read command
    buf[0] = HTU21_TRIGGER_TEMP_MEASUREMENT;
    ret = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, HTU21_I2C_ADDR, buf, 1, LP_I2C_TRANS_WAIT_FOREVER);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        error_no = ULP_ERROR_HTU21_TRIGGER_TEMP_MEASUREMENT;
        return;
    }

    ulp_lp_core_delay_us(50000); // wait for the sensor to complete the measurement

    // receive temperature data
    ret = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, HTU21_I2C_ADDR, buf, sizeof(buf), LP_I2C_TRANS_TIMEOUT_CYCLES);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        error_no = ULP_ERROR_HTU21_READ_TEMP;
        return;
    }

    if ((buf[1] & 0x02) != 0) {
        error_no = ULP_ERROR_HTU21_NOT_TEMP;
        return;
    }
    // calculate temperature
    temperature = ((buf[0]) << 8) | (buf[1] & 0xFC);



    // send humidity read command
    buf[0] = HTU21_TRIGGER_HUMI_MEASUREMENT;
    ret = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, HTU21_I2C_ADDR, buf, 1, LP_I2C_TRANS_WAIT_FOREVER);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        error_no = ULP_ERROR_HTU21_TRIGGER_HUMI_MEASUREMENT;
        return;
    }

    
    ulp_lp_core_delay_us(50000); // wait for the sensor to complete the measurement

    // receive humidity data
    ret = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, HTU21_I2C_ADDR, buf, sizeof(buf), LP_I2C_TRANS_TIMEOUT_CYCLES);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        error_no = ULP_ERROR_HTU21_READ_HUMI;
        return;
    }

    if ((buf[1] & 0x02) != 0x02) {
        error_no = ULP_ERROR_HTU21_NOT_HUMI;
        return;
    }

    // calculate humidity
    humidity = ((buf[0]) << 8) | (buf[1] & 0xFC);

    error_no = ULP_ERROR_NONE;
    ulp_lp_core_wakeup_main_processor();
    // Wakeup main CPU if the temperature or humidity breaches the thresholds
    // if (temperature < TEMP_THRESHOLD_LOW || temperature > TEMP_THRESHOLD_HIGH ||
    //     humidity < HUMI_THRESHOLD_LOW || humidity > HUMI_THRESHOLD_HIGH) {
    //     ulp_lp_core_wakeup_main_processor();
    // }
}
#define STATE_MON_PIN         LP_IO_NUM_1
int main (void)
{
    error_no = ULP_NOT_READY;

    /* Read BH1750 sensor data */
    bh1750_read();

    /* Read HTU21 sensor data */
    htu21_read();

    // temperature = 0;
    // humidity = 0;
    // lux = 0;
    // ulp_lp_core_gpio_set_level(STATE_MON_PIN, 0);
    // ulp_lp_core_delay_us(1000); // wait for the sensor to complete the measurement
    // ulp_lp_core_gpio_set_level(STATE_MON_PIN, 1);


    work_around += 1;
    ulp_lp_core_gpio_set_level(STATE_MON_PIN, 0);
    ulp_lp_core_delay_us(1000); // wait for the sensor to complete the measurement
    ulp_lp_core_gpio_set_level(STATE_MON_PIN, 1);

    return 0;
}
