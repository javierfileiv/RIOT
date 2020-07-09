/*
 * Copyright (C) 2020 javier.fileiv@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the VL53L0X pressure and temperature sensor
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include "vl53l0x.h"
#include "vl53l0x_params.h"
#include "xtimer.h"
#include "board.h"

int main(void)
{
    vl53l0x_t dev;
    int result;

    puts("VL53L0X test application\n");

    printf("+------------Initializing------------+\n");
    result = vl53l0x_init(&dev, &vl53l0x_params[0]);
    if (result == -VL53L0X_ERR_NODEV) {
        puts("[Error] The sensor did not answer correctly on the given address");
        return 1;
    }
    else if (result == -VL53L0X_ERR_NOCAL) {
        puts("[Error] Cannot read the sensor calibration values");
        return 1;
    }
    else {
        puts("Initialization successful\n");
    }

    printf("+------------Calibration------------+\n");
    printf("AC1: %i\n", dev.calibration.ac1);
    printf("AC2: %i\n", dev.calibration.ac2);
    printf("AC3: %i\n", dev.calibration.ac3);
    printf("AC4: %i\n", dev.calibration.ac4);
    printf("AC5: %i\n", dev.calibration.ac5);
    printf("AC6: %i\n", dev.calibration.ac6);
    printf("B1: %i\n", dev.calibration.b1);
    printf("B2: %i\n", dev.calibration.b2);
    printf("MB: %i\n", dev.calibration.mb);
    printf("MC: %i\n", dev.calibration.mc);
    printf("MD: %i\n", dev.calibration.md);
    printf("\n+--------Starting Measurements--------+\n");
    while (1) {
        /* Get temperature in deci degrees celsius */
        int16_t temperature = vl53l0x_read_temperature(&dev);

        /* Get pressure in Pa */
        uint32_t pressure = vl53l0x_read_pressure(&dev);

        /* Get pressure at sealevel in Pa */
        uint32_t pressure_0 = vl53l0x_sealevel_pressure(&dev, (int16_t)TEST_ALTITUDE);

        /* Get altitude in meters */
        int16_t altitude = vl53l0x_altitude(&dev, pressure_0);

        printf("Temperature [°C]: %i.%d\n"
               "Pressure [hPa]: %lu.%d\n"
               "Pressure at see level [hPa]: %lu.%d\n"
              "Altitude [m]: %i\n"
               "\n+-------------------------------------+\n",
               (int)(temperature / 10), abs(temperature % 10),
               (unsigned long)pressure / 100, (int)(pressure % 100),
               (unsigned long)pressure_0 / 100, (int)(pressure_0 % 100),
               (int)altitude);

        xtimer_sleep(2);
    }

    return 0;
}
