/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

/***************************************************
 * HTU21 Register Addresses
 ***************************************************/
#define HTU21_I2C_ADDR                 0x40
#define HTU21_SOFT_RESET               0xFE
#define HTU21_WRITE_USER_REG           0xE6
#define HTU21_READ_USER_REG            0xE7
#define HTU21_TRIGGER_TEMP_MEASUREMENT 0xF3 // No Hold master
#define HTU21_TRIGGER_HUMI_MEASUREMENT 0xF5 // No Hold master
#define HTU21_READ_TEMP_HUMI_BLOCK     0xE0
#define HTU21_READ_STATUS              0xE7
#define HTU21_CLEAR_STATUS             0x01
#define HTU21_WRITE_STATUS             0xE6

