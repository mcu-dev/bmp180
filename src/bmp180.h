
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

#endif /*BMP180_H*/