/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

/***************************************************
 * ulp error codes
 ***************************************************/

enum {
    ULP_ERROR_NONE = 0,
    ULP_ERROR_BH1750_POWER_ON = 1,
    ULP_ERROR_BH1750_RES_UPDATE = 2,
    ULP_ERROR_BH1750_READ = 3,
    ULP_ERROR_HTU21_POWER_ON = 4,
    ULP_ERROR_HTU21_TRIGGER_TEMP_MEASUREMENT = 5,
    ULP_ERROR_HTU21_READ_TEMP = 6,
    ULP_ERROR_HTU21_TRIGGER_HUMI_MEASUREMENT = 7,
    ULP_ERROR_HTU21_READ_HUMI = 8,
};