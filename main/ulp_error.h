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
    ULP_NOT_READY,
    ULP_ERROR_BH1750_POWER_ON,
    ULP_ERROR_BH1750_RES_UPDATE,
    ULP_ERROR_BH1750_READ,
    ULP_ERROR_HTU21_POWER_ON,
    ULP_ERROR_HTU21_TRIGGER_TEMP_MEASUREMENT,
    ULP_ERROR_HTU21_READ_TEMP,
    ULP_ERROR_HTU21_TRIGGER_HUMI_MEASUREMENT,
    ULP_ERROR_HTU21_READ_HUMI,
    ULP_ERROR_HTU21_NOT_HUMI,
    ULP_ERROR_HTU21_NOT_TEMP,
};