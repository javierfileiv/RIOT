/*
 * Copyright (C) 2020 javier.fileiv@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_vl53l0x VL53L0X TOF sensor
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver interface for the VL53L0X time of flight sensor
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief       Device driver interface for the VL53L0X TOF sensor.
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "saul.h"
#include "periph/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Integration times
 */
typedef enum veml6070_integrationtime {
  VEML6070_HALF_T = 0,              /**< 1/2 T integration time */
  VEML6070_1_T,                     /**< 1 T integration time */
  VEML6070_2_T,                     /**< 2 T integration time */
  VEML6070_4_T,                     /**< 4 T integration time */
} veml6070_itime_t;



#ifdef __cplusplus
}
#endif

#endif VL53L0X_H /* VL53L0X_H */
/** @} */