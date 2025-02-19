/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

#include <device.h>
#include <esp_matter.h>
#include <led_driver.h>

#include <app_priv.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp_lp_core.h"
#include "lp_core_i2c.h"
#include "ulp_main.h"
#include "driver/i2c_master.h"
#include "bh1750_defs.h"


using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";


extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);

#define PWR_MON_PIN         GPIO_NUM_1
#define PWR_OFF_PIN         GPIO_NUM_2


#define I2C_SDA_PIN      GPIO_NUM_6
#define I2C_SCL_PIN      GPIO_NUM_7

#if 0
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;
i2c_master_dev_handle_t htu21_dev_handle;

#define LP_I2C_TRANS_TIMEOUT_CYCLES 5000
#define LP_I2C_TRANS_WAIT_FOREVER   -1

#define LUX_THRESHOLD_LOW           5
#define LUX_THRESHOLD_HIGH          1000

static uint32_t sensor_on = 0;
static uint32_t res_update_done = 0;
volatile uint32_t lux = 0;

int hp_i2c_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { 
            .enable_internal_pullup = true,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG, "HP I2C initialized successfully\n");

    i2c_device_config_t i2c_dev_conf = {
        .device_address = BH1750_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        return 1;
    }

    i2c_device_config_t htu21_dev_conf = {
        .device_address = 0x40,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus_handle, &htu21_dev_conf, &htu21_dev_handle) != ESP_OK) {
        return 1;
    }

    return 0;
}

void htu21_read()
{
    uint8_t data_rd[3];
    esp_err_t ret = ESP_OK;
    while (1) {
        // read temperature

        // send temperature read command
        data_rd[0] = 0xF3;
        ret = i2c_master_transmit(htu21_dev_handle, data_rd, 1, LP_I2C_TRANS_TIMEOUT_CYCLES);
        if (ret != ESP_OK) {
            printf("HTU21D write failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // wait for the sensor to complete the measurement

        // receive temperature data
        ret = i2c_master_receive(htu21_dev_handle, data_rd, sizeof(data_rd), LP_I2C_TRANS_TIMEOUT_CYCLES);
        if (ret != ESP_OK) {
            printf("HTU21D read failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if ((data_rd[1] & 0x02) != 0) {
            printf("HTU21D read temperature failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint16_t temperature = ((data_rd[0]) << 8) | (data_rd[1] & 0xFC);

        // read humidity

        // send humidity read command
        data_rd[0] = 0xF5;
        ret = i2c_master_transmit(htu21_dev_handle, data_rd, 1, LP_I2C_TRANS_TIMEOUT_CYCLES);
        if (ret != ESP_OK) {
            printf("HTU21D write failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // wait for the sensor to complete the measurement

        // receive humidity data
        ret = i2c_master_receive(htu21_dev_handle, data_rd, sizeof(data_rd), LP_I2C_TRANS_TIMEOUT_CYCLES);
        if (ret != ESP_OK) {
            printf("HTU21D read failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if ((data_rd[1] & 0x02) != 0x02) {
            printf("HTU21D read humidity failed\n");
            printf("data_rd[1]: %d\n", data_rd[1]);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint16_t humidity = ((data_rd[0]) << 8) | (data_rd[1] & 0xFC);
        float rh = -6.0f + 125.0f * humidity / (1 << 16);
        float temp = -46.85f + 175.72f * temperature / (1 << 16);
        printf("Humidity: %f, Temperature: %f\n", rh, temp);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void bh1750_read()
{
    uint8_t data_rd[2];
    esp_err_t ret = i2c_master_receive(dev_handle, data_rd, sizeof(data_rd), LP_I2C_TRANS_TIMEOUT_CYCLES);
    if (ret != ESP_OK) {
        // Skip this round of calculation and return
        return;
    }

    /* Calculate light intensity value */
    uint16_t level = ((data_rd[0] << 8) | data_rd[1]);
    lux = (level * 10) / 12;

    /* Wakeup main CPU if the Lux breaches the thresholds */
    if (lux < LUX_THRESHOLD_LOW || lux > LUX_THRESHOLD_HIGH) {
        printf("Lux: %ld\n", lux);
    }
}


int i2c_main (void)
{
    uint8_t data_wr = 0;
    esp_err_t ret = ESP_OK;

    while (1) {
        if (!sensor_on) {
            /* Power ON the sensor */
            data_wr = BH1750_POWER_ON;
            ret = i2c_master_transmit(dev_handle, &data_wr, sizeof(data_wr), LP_I2C_TRANS_WAIT_FOREVER);
            if (ret != ESP_OK) {
                // Bail and try again
                continue;
            }
            sensor_on = 1;
        }

        if (!res_update_done) {
            data_wr = EXAMPLE_RES_MODE;
            ret = i2c_master_transmit(dev_handle, &data_wr, sizeof(data_wr), LP_I2C_TRANS_WAIT_FOREVER);
            if (ret != ESP_OK) {
                // Bail and try again
                continue;
            }
            res_update_done = 1;
        }

        /* Read BH1750 sensor data */
        bh1750_read();
    }

    return 0;
}
#endif

static void lp_i2c_init(void)
{
    esp_err_t ret = ESP_OK;

    /* Initialize LP I2C with default configuration */
    const lp_core_i2c_cfg_t i2c_cfg = {
      .i2c_pin_cfg = {
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
      },
      .i2c_timing_cfg = {
        .clk_speed_hz = 100000,
      },
      .i2c_src_clk = LP_I2C_SCLK_LP_FAST,
    };

    ret = lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg);
    if (ret != ESP_OK) {
        printf("LP I2C init failed\n");
        abort();
    }

    printf("LP I2C initialized successfully\n");
}

#define STATE_MON_PIN         GPIO_NUM_1

void ulp_driver_init(void)
{

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(STATE_MON_PIN);
    rtc_gpio_set_direction(STATE_MON_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_en(STATE_MON_PIN);
    rtc_gpio_pullup_dis(STATE_MON_PIN);

    rtc_gpio_set_level(STATE_MON_PIN, 0);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    /* not a wakeup from ULP, load the firmware */
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        lp_i2c_init();
        printf("Not a ULP wakeup, initializing it! \n");
        init_ulp_program();
    }


    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup());

}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_lp_core_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    /* Start the program */
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
        .lp_timer_sleep_duration_us = 5000000,
    };

    err = ulp_lp_core_run(&cfg);
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "ULP program started\n");
    vTaskDelay(pdMS_TO_TICKS(500));
}


esp_err_t background_task_create(void *pvParameters)
{
    esp_err_t err = ESP_OK;

    /* Create a task to periodically read temperature and humidity from ULP variables */
    xTaskCreate([](void *pvParameters) {
        while (1) {
            /* Read temperature and humidity values from ULP shared variables */
            float humidity = -6.0f + 125.0f * ulp_humidity / (1 << 16);
            float temp = -46.85f + 175.72f * ulp_temperature / (1 << 16);

            ESP_LOGI(TAG, "Workaround: %ld, Lux: %ld, Temperature: %.2f°C, Humidity: %.2f%%, Error: %ld", ulp_work_around, ulp_lux, temp, humidity, ulp_error_no);

            if(ulp_error_no == 0) {
                /* Update Matter sensor attributes */

                /* Update humidity sensor attribute */
                esp_matter_attr_val_t humidity_val = esp_matter_uint16((uint16_t)(humidity * 100));
                esp_matter::attribute::update(1, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &humidity_val);

                /* Update temperature sensor attribute */
                esp_matter_attr_val_t temp_val = esp_matter_int16((int16_t)(temp * 100));
                esp_matter::attribute::update(2, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &temp_val);

                /* Update light sensor attribute */
                esp_matter_attr_val_t lux_val = esp_matter_uint16(10000*std::log10(ulp_lux) + 1);
                esp_matter::attribute::update(3, IlluminanceMeasurement::Id, IlluminanceMeasurement::Attributes::MeasuredValue::Id, &lux_val);

            }

            /* Read every 5 seconds */
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }, "sensor_read", 4096, nullptr, 24, nullptr);
    return err;
}


esp_err_t app_driver_init()
{
    esp_err_t err = ESP_OK;
    ulp_driver_init();
    // hp_i2c_init();
    // i2c_main();
    // htu21_read();
    background_task_create(nullptr);
    return err;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    return err;
}