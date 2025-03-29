/*
 * lsm6dsl_hal.c
 *
 *  Created on: Mar 26, 2025
 *      Author: lapchong
 */

#include "lsm6dsl_hal.h"
#include "lsm6dsl_reg.h"
#include "stm32f4xx_hal.h"
#include "my_print.h"

#define SENSOR_BUS hi2c2
#define TX_BUF_DIM          1000
#define    BOOT_TIME          15 //ms

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

void lsm6dsl_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  whoamI = 0;
  lsm6dsl_device_id_get(&dev_ctx, &whoamI);

  if ( whoamI != LSM6DSL_ID ) {
	  my_printf("device id is wrong: %d\r\n", whoamI);
	  while (1);
	  /*manage here device not found */
  } else {
	  my_printf("device id correct\r\n");
  }


  /* Restore default configuration */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
  my_printf("restore default config correct\r\n");
  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);
  my_printf("reset done\r\n");
  /* Enable Block Data Update */
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5);
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
  /* Set full scale */
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
  lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
  /* Accelerometer - LPF1 + LPF2 path */
  lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx,
                               LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
  /* Accelerometer - High Pass / Slope path */
  //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
  my_printf("setup parameter done\r\n");

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read output only if new value is available */
    lsm6dsl_reg_t reg;
    lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
      /* Read magnetic field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg(
                             data_raw_acceleration[2]);
      my_printf( "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }

    if (reg.status_reg.gda) {
      /* Read magnetic field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] = lsm6dsl_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[0]);
      angular_rate_mdps[1] = lsm6dsl_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[1]);
      angular_rate_mdps[2] = lsm6dsl_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[2]);
      my_printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    }

    if (reg.status_reg.tda) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dsl_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dsl_from_lsb_to_celsius(
                           data_raw_temperature );
      my_printf("Temperature [degC]:%6.2f\r\n", temperature_degC );
    }
  }
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}
