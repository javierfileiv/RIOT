/*
 * Copyright (C) 2020 javier.fileiv@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_vl53l0x
 *
 * @{
 * @file
 * @brief       Default configuration for VL53L0X TOF sensor.
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */

#ifndef VL53L0X_PARAMS_H
#define VL53L0X_PARAMS_H

#include "board.h"
#include "vl53l0x.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the VL53L0X
 * @{
 */
#ifndef VL53L0X_PARAM_I2C_DEV
#define VL53L0X_PARAM_I2C_DEV          I2C_DEV(0)
#endif
#ifndef VL53L0X_PARAM_ITIME
#define VL53L0X_PARAM_ITIME            VL53L0X_1_T
#endif

#ifndef VL53L0X_PARAMS
#define VL53L0X_PARAMS                { .i2c_dev = VL53L0X_PARAM_I2C_DEV, \
                                        .itime   = VL53L0X_PARAM_ITIME }
#endif
#ifndef VL53L0X_SAUL_INFO
#define VL53L0X_SAUL_INFO             { .name = "vl53l0x" }
#endif
/**@}*/

/**
 * @brief   Configure VL53L0X
 */
static const vl53l0x_params_t vl53l0x_params[] =
{
    VL53L0X_PARAMS
};

/**
 * @brief   Configure SAUL registry entries
 */
static const saul_reg_info_t vl53l0x_saul_info[] =
{
    VL53L0X_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_PARAMS_H */
/** @} */
