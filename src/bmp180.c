/**
 ******************************************************************************
 * @file    bmp180.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    21-Jan-2025
 * @brief   Contains all the functionalities to control the BMP180
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

#include "bmp180.h"

/**
 * @brief Write a register value from the BMP180.
 *
 * @param dev_addr The register address to write the value from.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */

int8_t bmp180_write_register_value(uint8_t dev_addr, uint8_t *data_buffer) {
  return i2c_write_bytes(dev_addr, data_buffer);
}

/**
 * @brief Read a register value from the BMP180.
 *
 * @param address The register address to read the value from.
 * @param val Pointer to a variable to store the read register value.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t bmp180_read_register_value(uint8_t address, uint8_t *val) {
  if (i2c_read_byte(BMP180_I2C_ADDRESS, address, val) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_setup(bmp180_dev *device, bpm180_init_param init_param) {
  int8_t ret = 0;

  if (!i2c_init()) {
    return BMP180_STATUS_INIT_ERR;
  }

  ret |= bmp180_get_calibration_ac1(device);
  ret |= bmp180_get_calibration_ac2(device);
  ret |= bmp180_get_calibration_ac3(device);
  ret |= bmp180_get_calibration_ac4(device);
  ret |= bmp180_get_calibration_ac5(device);
  ret |= bmp180_get_calibration_ac6(device);
  ret |= bmp180_get_calibration_b1(device);
  ret |= bmp180_get_calibration_b2(device);
  ret |= bmp180_get_calibration_mb(device);
  ret |= bmp180_get_calibration_mc(device);
  ret |= bmp180_get_calibration_md(device);

  if (ret == BMP180_STATUS_SUCCESS)
    device->is_Setup = true;
  else
    device->is_Setup = false;

  return ret;
}

bool bmp180_online(void) {
  uint8_t val = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_DEV_ID, &val) !=
      BMP180_STATUS_SUCCESS) {
    return false;
  }
  if (val != BMP180_DEV_ID) {
    return false;
  }
  return true;
}

int8_t bmp180_soft_reset(void) {
  uint8_t data_buffer[] = {BMP180_REG_SOFT_RST, BMP180_RST_VAL};
  return i2c_write_bytes(BMP180_I2C_ADDRESS, data_buffer);
}

int8_t bmp180_get_calibration_ac1(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC1_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC1_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac1 = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_ac2(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC2_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC2_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac2 = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_ac3(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC3_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC3_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac3 = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_ac4(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC4_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC4_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac4 = (uint16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_ac5(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC5_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC5_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac5 = (uint16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_ac6(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC6_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_AC6_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.ac6 = (uint16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_b1(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_B1_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_B1_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.b1 = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_b2(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_B2_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_B2_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.b2 = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_mb(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MB_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MB_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.mb = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_mc(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MC_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MC_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.mc = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_calibration_md(bmp180_dev *device) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MD_H, &val_h) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_MD_L, &val_l) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  device->cal_data.md = (int16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_uncompensated_temperature(bmp180_dev *device,
                                            bmp180_data *data) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  uint8_t data_buffer[] = {BMP180_REG_CTL, BMP180_UT_CMD};
  if (i2c_write_bytes(BMP180_I2C_ADDRESS, data_buffer) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  delay_ms(5);

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_DT_H, &val_h) != 0) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_DT_L, &val_l) != 0) {
    return BMP180_STATUS_API_ERR;
  }

  data->ut = (uint16_t)((val_h << 8) | val_l);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_uncompensated_pressure(bmp180_dev *device,
                                         bmp180_data *data) {
  uint8_t val_h  = 0x00;
  uint8_t val_l  = 0x00;
  uint8_t val_xl = 0x00;

  uint8_t data_buffer[] = {BMP180_REG_CTL, BMP180_UP_CMD};
  if (i2c_write_bytes(BMP180_I2C_ADDRESS, data_buffer) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }
  delay_ms(5);

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_DT_H, &val_h) != 0) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BMP180_REG_DT_L, &val_l) != 0) {
    return BMP180_STATUS_API_ERR;
  }

  if (i2c_read_byte(BMP180_I2C_ADDRESS, BPM180_REG_DT_XL, &val_xl) != 0) {
    return BMP180_STATUS_API_ERR;
  }

  data->up = (uint16_t)(((val_h << 16) | (val_l << 8) | val_xl) >> 8);

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_temperature(bmp180_dev *device, bmp180_data *data) {
  int16_t x1, x2;

  if (bmp180_get_uncompensated_temperature(device, data) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  x1 = ((data->ut - device->cal_data.ac6) * device->cal_data.ac5) / (1 << 15);
  x2 = (device->cal_data.mc * (1 << 11)) / (x1 + device->cal_data.md);
  device->cal_data.b5 = x1 + x2;
  data->temp          = (float)(device->cal_data.b5 + 8) / 160;

  return BMP180_STATUS_SUCCESS;
}

int8_t bmp180_get_pressure(bmp180_dev *device, bmp180_data *data) {
  int16_t b3, b6, x1, x2, x3;
  uint32_t b4, b7;
  int32_t p = 0;

  // Get the uncompensated pressure
  if (bmp180_get_uncompensated_pressure(device, data) !=
      BMP180_STATUS_SUCCESS) {
    return BMP180_STATUS_API_ERR;
  }

  b6 = device->cal_data.b5 - 4000;

  x1 = (device->cal_data.b2 * ((b6 * b6) / (1 << 12))) / (1 << 11);
  x2 = (device->cal_data.ac2 * b6) / (1 << 11);
  x3 = x1 + x2;

  b3 = ((((device->cal_data.ac1) * 4 + x3)) + 2) / 4;

  x1 = (device->cal_data.ac3 * b6) / (1 << 13);
  x2 = (device->cal_data.b1 * ((b6 * b6) / (1 << 12))) / (1 << 16);
  x3 = ((x1 + x2) + 2) / 4;

  b4 = ((device->cal_data.ac4 * (uint32_t)(x3 + 32768))) / (1 << 15);
  b7 = ((uint32_t)(data->up - b3) * (50000));

  if (b7 < 0x80000000) {
    p = (b7 << 1) / b4;
  } else {
    p = (b7 / b4) * 2;
  }

  x1 = (p * p) / (1 << 16);
  x1 = (x1 * 3038) / (1 << 16);
  x2 = (-7357 * p) / (1 << 16);
  p += (x1 + x2 + 3791) / (1 << 4);

  data->pressure_pa  = p;
  data->pressure_atm = (float)(p / ATM_CONV_FACTOR);

  return BMP180_STATUS_SUCCESS;
}
