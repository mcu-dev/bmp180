#include "stdint.h"

#include "cmock.h"
#include "unity.h"

#include "bmp180.h"
#include "mock_i2c.h"
#include "mock_utils.h"

static bmp180_dev dev;
static bmp180_data data;
static bmp180_calibration_data dummy_cal_data;

void setUp(void) {
  dummy_cal_data.ac1 = 7468;
  dummy_cal_data.ac2 = -1139;
  dummy_cal_data.ac3 = -14664;
  dummy_cal_data.ac4 = 33334;
  dummy_cal_data.ac5 = 24647;
  dummy_cal_data.ac6 = 18052;
  dummy_cal_data.b1  = 6515;
  dummy_cal_data.b2  = 43;
  dummy_cal_data.mb  = -32768;
  dummy_cal_data.mc  = -11786;
  dummy_cal_data.md  = 3035;
}

void test_bmp180_online(void) {
  uint8_t val = 0x55;
  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_DEV_ID, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val);

  TEST_ASSERT_TRUE(bmp180_online());
}

void test_bmp180_get_calibration_ac1(void) {
  uint8_t val_h = 0x1D;
  uint8_t val_l = 0x2C;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC1_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC1_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac1(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac1, dev.cal_data.ac1);
}

void test_bmp180_get_calibration_ac2(void) {
  uint8_t val_h = 0xFB;
  uint8_t val_l = 0x8D;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC2_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC2_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac2(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac2, dev.cal_data.ac2);
}

void test_bmp180_get_calibration_ac3(void) {
  uint8_t val_h = 0xC6;
  uint8_t val_l = 0xB8;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC3_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC3_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac3(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac3, dev.cal_data.ac3);
}

void test_bmp180_get_calibration_ac4(void) {
  uint8_t val_h = 0x82;
  uint8_t val_l = 0x36;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC4_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC4_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac4(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac4, dev.cal_data.ac4);
}

void test_bmp180_get_calibration_ac5(void) {
  uint8_t val_h = 0x60;
  uint8_t val_l = 0x47;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC5_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC5_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac5(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac5, dev.cal_data.ac5);
}

void test_bmp180_get_calibration_ac6(void) {
  uint8_t val_h = 0x46;
  uint8_t val_l = 0x84;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC6_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_AC6_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_ac6(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.ac6, dev.cal_data.ac6);
}

void test_bmp180_get_calibration_b1(void) {
  uint8_t val_h = 0x19;
  uint8_t val_l = 0x73;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_B1_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_B1_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_b1(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.b1, dev.cal_data.b1);
}

void test_bmp180_get_calibration_b2(void) {
  uint8_t val_h = 0x00;
  uint8_t val_l = 0x2B;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_B2_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_B2_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_b2(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.b2, dev.cal_data.b2);
}

void test_bmp180_get_calibration_mb(void) {
  uint8_t val_h = 0x80;
  uint8_t val_l = 0x00;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MB_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MB_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_mb(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.mb, dev.cal_data.mb);
}

void test_bmp180_get_calibration_mc(void) {
  uint8_t val_h = 0xD1;
  uint8_t val_l = 0xF6;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MC_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MC_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_mc(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.mc, dev.cal_data.mc);
}

void test_bmp180_get_calibration_md(void) {
  uint8_t val_h = 0x0B;
  uint8_t val_l = 0xDB;

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MD_H, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_MD_L, NULL,
                                BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_calibration_md(&dev));
  TEST_ASSERT_EQUAL(dummy_cal_data.md, dev.cal_data.md);
}

void test_bmp180_get_temperature(void){
  uint8_t val_h = 0x6A;
  uint8_t val_l = 0x17;

  i2c_write_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, NULL, BMP180_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  delay_ms_Ignore();

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_DT_H, NULL, BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_DT_L, NULL, BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_temperature(&dev, &data));
  TEST_ASSERT_EQUAL(27159, data.ut);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 27.59, data.temp);
}

void test_bmp180_get_pressure(void){
  uint8_t val_h = 0x9F;
  uint8_t val_l = 0x6C;
  uint8_t val_xl = 0x00;

  i2c_write_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, NULL, BMP180_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();

  delay_ms_Ignore();

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_DT_H, NULL, BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BMP180_REG_DT_L, NULL, BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  i2c_read_byte_ExpectAndReturn(BMP180_I2C_ADDRESS, BPM180_REG_DT_XL, NULL, BMP180_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_xl);

  TEST_ASSERT_EQUAL(BMP180_STATUS_SUCCESS, bmp180_get_pressure(&dev, &data));
  TEST_ASSERT_EQUAL(40812, data.up);
  TEST_ASSERT_EQUAL(100354, data.pressure_pa);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.991, data.pressure_atm);

}