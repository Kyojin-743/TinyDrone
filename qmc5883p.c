/*
 * qmc5883p.c
 *
 *  Created on: Mar 24, 2026
 *      Author: kyojin
 */
#include "qmc5883p.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c1.h"
#include "wait.h"

bool qmc_verify(void) {
    return QMC5883P_CHIPID_ID == readI2c1Register(QMC5883P_ADDR, QMC5883P_CHIPID_R);
}

void qmc_clear_int(void) {
    readI2c1Register(QMC5883P_ADDR, QMC5883P_STAT_R);
}

void qmc_init(void) {

    if(!qmc_verify()) for(;;);

    //Set to Default
    writeI2c1Register(QMC5883P_ADDR, QMC5883P_CTRLB_R, QMC5883P_CTRLB_SOFT_RESET);
    waitMicrosecond(10);

    //Documentation describes changing the sign (flips Y and Z)
    writeI2c1Register(QMC5883P_ADDR, QMC5883P_SIGN_R, 0x06);

    writeI2c1Register(QMC5883P_ADDR, QMC5883P_CTRLB_R,
                      QMC5883P_CTRLB_RNG_12GA);

    writeI2c1Register(QMC5883P_ADDR, QMC5883P_CTRLA_R,
                      QMC5883P_CTRLA_MODE_CONTINUOUS |
                      QMC5883P_CTRLA_ODR_200HZ |
                      QMC5883P_CTRLA_OSR1_8 |
                      QMC5883P_CTRLA_OSR2_1);
}

//      QMC5883P Magnetometer Conversion
/* +-------------+-----------------+ *
 * |     FSR     |     LSB Sens.   | *
 * +--------------------+----------+ *
 * |   ± 30 G    |  1000  LSB/G    | *
 * |   ± 12 G    |  2500  LSB/G    | *
 * |   ± 8  G    |  3750  LSB/G    | *
 * |   ± 2  G    |  15000 LSB/G    | *
 * +-------------+-----------------+ */
#define MAG_SENS    2500

Vec3f qmc_read(void) {
    uint8_t bytes[6];
    Vec3f raws;
    readI2c1Registers(QMC5883P_ADDR, QMC5883P_DOUT_XL_R, bytes, 6);
    float inv_sense = 1.0f / MAG_SENS;
    raws.x = (float)((int16_t)(bytes[1] << 8 | bytes[0])) * inv_sense;
    raws.y = (float)((int16_t)(bytes[3] << 8 | bytes[2])) * inv_sense;
    raws.z = (float)((int16_t)(bytes[5] << 8 | bytes[4])) * inv_sense;
    return raws;
}
