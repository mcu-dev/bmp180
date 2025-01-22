
/**
 ******************************************************************************
 * @file    bmp180.h
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    21-Jan-2025
 * @brief   Contains all the prototypes for the bmp180.C
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef BMP180_H
#define BMP180_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "utils/i2c.h"
#include "utils/utils.h"

/*******************************CONSTANTS**************************************/
#define ATM_CONV_FACTOR 101325.0f
#define BMP180_RST_VAL  0xB6
#define BMP180_UT_CMD   0x2E
#define BMP180_UP_CMD   0x34

/*******************************DATASET****************************************/
typedef struct {
  uint16_t ut;
  uint16_t up;
  int32_t pressure_pa;
  float pressure_atm;
  float temp;
} bmp180_data;

/*******************************STATUSES***************************************/
typedef enum {
  BMP180_STATUS_SUCCESS   = 0,
  BMP180_STATUS_API_ERR   = -1,
  BMP180_STATUS_INPUT_ERR = -2,
  BMP180_STATUS_INIT_ERR  = -3,
} BMP180_RETURN_STATUS;

/*****************************ID REGISTERS*************************************/

#define BMP180_I2C_ADDRESS 0x77
#define BMP180_DEV_ID      0x55

/*********************************REGISTERS************************************/

#define BMP180_REG_AC1_H    0xAA
#define BMP180_REG_AC1_L    0xAB
#define BMP180_REG_AC2_H    0xAC
#define BMP180_REG_AC2_L    0xAD
#define BMP180_REG_AC3_H    0xAE
#define BMP180_REG_AC3_L    0xAF
#define BMP180_REG_AC4_H    0xB0
#define BMP180_REG_AC4_L    0xB1
#define BMP180_REG_AC5_H    0xB2
#define BMP180_REG_AC5_L    0xB3
#define BMP180_REG_AC6_H    0xB4
#define BMP180_REG_AC6_L    0xB5
#define BMP180_REG_B1_H     0xB6
#define BMP180_REG_B1_L     0xB7
#define BMP180_REG_B2_H     0xB8
#define BMP180_REG_B2_L     0xB9
#define BMP180_REG_MB_H     0xBA
#define BMP180_REG_MB_L     0xBB
#define BMP180_REG_MC_H     0xBC
#define BMP180_REG_MC_L     0xBD
#define BMP180_REG_MD_H     0xBE
#define BMP180_REG_MD_L     0xBF
#define BMP180_REG_DEV_ID   0xD0
#define BMP180_REG_SOFT_RST 0xE0
#define BMP180_REG_CTL      0xF4
#define BMP180_REG_DT_H     0xF6
#define BMP180_REG_DT_L     0xF7
#define BPM180_REG_DT_XL    0xF8

/*********************************DESCRIPTORS**********************************/
typedef enum {
  BMP180_ULTRA_LOW_POWER  = 0x00,
  BMP180_STANDARD_POWER   = 0x01,
  BMP180_HIGH_POWER       = 0x02,
  BMP180_ULTRA_HIGH_POWER = 0x03
} bmp180_power_mode;

typedef struct {
  int16_t ac1;
  int16_t ac2;
  int16_t ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t b1;
  int16_t b2;
  int16_t mb;
  int16_t mc;
  int16_t md;
  int16_t b5;
} bmp180_calibration_data;

/**********************************HANDLES*************************************/
typedef struct {
  bmp180_calibration_data cal_data;
  bmp180_power_mode power_mode;
  bool is_Setup;
} bmp180_dev;

typedef struct {
  bmp180_calibration_data cal_data;
  bmp180_power_mode power_mode;
  bool is_Setup;
} bpm180_init_param;

/*******************************PROTOTYPES*************************************/

int8_t bmp180_write_register_value(uint8_t dev_addr, uint8_t *data_buffer);

int8_t bmp180_read_register_value(uint8_t address, uint8_t *val);

int8_t bmp180_setup(bmp180_dev *device, bpm180_init_param init_param);

bool bmp180_online(void);

int8_t bmp180_soft_reset(void);

int8_t bmp180_get_calibration_ac1(bmp180_dev *device);

int8_t bmp180_get_calibration_ac2(bmp180_dev *device);

int8_t bmp180_get_calibration_ac3(bmp180_dev *device);

int8_t bmp180_get_calibration_ac4(bmp180_dev *device);

int8_t bmp180_get_calibration_ac5(bmp180_dev *device);

int8_t bmp180_get_calibration_ac6(bmp180_dev *device);

int8_t bmp180_get_calibration_b1(bmp180_dev *device);

int8_t bmp180_get_calibration_b2(bmp180_dev *device);

int8_t bmp180_get_calibration_mb(bmp180_dev *device);

int8_t bmp180_get_calibration_mc(bmp180_dev *device);

int8_t bmp180_get_calibration_md(bmp180_dev *device);

int8_t bmp180_get_uncompensated_temperature(bmp180_dev *device,
                                            bmp180_data *data);

int8_t bmp180_get_uncompensated_pressure(bmp180_dev *device, bmp180_data *data);

int8_t bmp180_get_temperature(bmp180_dev *device, bmp180_data *data);

int8_t bmp180_get_pressure(bmp180_dev *device, bmp180_data *data);

#endif /*BMP180_H*/
