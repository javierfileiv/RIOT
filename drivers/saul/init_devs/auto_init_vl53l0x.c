/*
 * Copyright (C) 2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of VL53L0X TOF driver.
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 *
 * @}
 */

#include "assert.h"
#include "log.h"
#include "saul_reg.h"
#include "vl53l0x.h"
#include "vl53l0x_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define VL53L0X_NUM    ARRAY_SIZE(vl53l0x_params)

/**
 * @brief   Allocation of memory for device descriptors
 */
static vl53l0x_t vl53l0x_devs[VL53L0X_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[VL53L0X_NUM];

/**
 * @brief   Define the number of saul info
 */
#define VL53L0X_INFO_NUM    ARRAY_SIZE(vl53l0x_saul_info)

/**
 * @brief   Reference the driver structs.
 */
extern const saul_driver_t vl53l0x_uv_saul_driver;

void auto_init_vl53l0x(void)
{
    assert(VL53L0X_NUM == VL53L0X_INFO_NUM);

    for (unsigned i = 0; i < VL53L0X_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing vl53l0x #%u\n", i);

        if (vl53l0x_init(&vl53l0x_devs[i],
                          &vl53l0x_params[i]) != VL53L0X_OK) {
            LOG_ERROR("[auto_init_saul] error initializing vl53l0x #%u\n", i);
            continue;
        }

        saul_entries[(i)].dev = &(vl53l0x_devs[i]);
        saul_entries[(i)].name = vl53l0x_saul_info[i].name;
        saul_entries[(i)].driver = &vl53l0x_uv_saul_driver;

        /* register to saul */
        saul_reg_add(&(saul_entries[(i)]));
    }
}
