/**
* Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bme68x_defs.h
* @date       2021-11-09
* @version    v4.4.7
*
*/

/*! @cond DOXYGEN_SUPRESS */

#ifndef BME68X_DEFS_H_
#define BME68X_DEFS_H_

/********************************************************* */
/*!             Header includes                           */
/********************************************************* */
#include <stdint.h>
#include <stddef.h>

/********************************************************* */
/*!               Common Macros                           */
/********************************************************* */

/* BME68X unique chip identifier */
#define BME68X_CHIP_ID                            UINT8_C(0x61)

/* BME68X lower I2C address */
#define BME68X_I2C_ADDR                       UINT8_C(0x76)

/* Soft reset command */
#define BME68X_SOFT_RESET_CMD                     UINT8_C(0xb6)

/* ------------------------Register map addresses in I2C------------------------ */
/* Register for 3rd group of coefficients */
#define BME68X_REG_COEFF3                         UINT8_C(0x00)

/* 0th Field address*/
#define BME68X_REG_FIELD0                         UINT8_C(0x1d)

/* 0th Current DAC address*/
#define BME68X_REG_IDAC_HEAT0                     UINT8_C(0x50)

/* 0th Res heat address */
#define BME68X_REG_RES_HEAT0                      UINT8_C(0x5a)

/* 0th Gas wait address */
#define BME68X_REG_GAS_WAIT0                      UINT8_C(0x64)

/* Shared heating duration address */
#define BME68X_REG_SHD_HEATR_DUR                  UINT8_C(0x6E)

/* CTRL_GAS_0 address */
#define BME68X_REG_CTRL_GAS_0                     UINT8_C(0x70)

/* CTRL_GAS_1 address */
#define BME68X_REG_CTRL_GAS_1                     UINT8_C(0x71)

/* CTRL_HUM address */
#define BME68X_REG_CTRL_HUM                       UINT8_C(0x72)

/* CTRL_MEAS address */
#define BME68X_REG_CTRL_MEAS                      UINT8_C(0x74)

/* CONFIG address */
#define BME68X_REG_CONFIG                         UINT8_C(0x75)

/* MEM_PAGE address */
#define BME68X_REG_MEM_PAGE                       UINT8_C(0xf3)

/* Unique ID address */
#define BME68X_REG_UNIQUE_ID                      UINT8_C(0x83)

/* Register for 1st group of coefficients */
#define BME68X_REG_COEFF1                         UINT8_C(0x8a)

/* Chip ID address */
#define BME68X_REG_CHIP_ID                        UINT8_C(0xd0)

/* Soft reset address */
#define BME68X_REG_SOFT_RESET                     UINT8_C(0xe0)

/* Register for 2nd group of coefficients */
#define BME68X_REG_COEFF2                         UINT8_C(0xe1)

/* Variant ID Register */
#define BME68X_REG_VARIANT_ID                     UINT8_C(0xF0)

/* ------------------------Enable/Disable macros------------------------ */

/* Enable */
#define BME68X_ENABLE                             UINT8_C(0x01)

/* Disable */
#define BME68X_DISABLE                            UINT8_C(0x00)

/* ------------------------Variant ID macros------------------------ */

/* Low Gas variant */
#define BME68X_VARIANT_GAS_LOW                    UINT8_C(0x00)

/* High Gas variant */
#define BME68X_VARIANT_GAS_HIGH                   UINT8_C(0x01)

/* ------------------------Oversampling setting macros------------------------ */

/* Switch off measurement */
#define BME68X_OS_NONE                            UINT8_C(0)

/* Perform 1 measurement */
#define BME68X_OS_1X                              UINT8_C(1)

/* Perform 2 measurements */
#define BME68X_OS_2X                              UINT8_C(2)

/* Perform 4 measurements */
#define BME68X_OS_4X                              UINT8_C(3)

/* Perform 8 measurements */
#define BME68X_OS_8X                              UINT8_C(4)

/* Perform 16 measurements */
#define BME68X_OS_16X                             UINT8_C(5)

/* ------------------------IIR Filter settings------------------------ */

/* Switch off the filter */
#define BME68X_FILTER_OFF                         UINT8_C(0)

/* Filter coefficient of 2 */
#define BME68X_FILTER_SIZE_1                      UINT8_C(1)

/* Filter coefficient of 4 */
#define BME68X_FILTER_SIZE_3                      UINT8_C(2)

/* Filter coefficient of 8 */
#define BME68X_FILTER_SIZE_7                      UINT8_C(3)

/* Filter coefficient of 16 */
#define BME68X_FILTER_SIZE_15                     UINT8_C(4)

/* Filter coefficient of 32 */
#define BME68X_FILTER_SIZE_31                     UINT8_C(5)

/* Filter coefficient of 64 */
#define BME68X_FILTER_SIZE_63                     UINT8_C(6)

/* Filter coefficient of 128 */
#define BME68X_FILTER_SIZE_127                    UINT8_C(7)

/* ------------------------ODR/Standby time macros------------------------ */

/* Standby time of 0.59ms */
#define BME68X_ODR_0_59_MS                        UINT8_C(0)

/* Standby time of 62.5ms */
#define BME68X_ODR_62_5_MS                        UINT8_C(1)

/* Standby time of 125ms */
#define BME68X_ODR_125_MS                         UINT8_C(2)

/* Standby time of 250ms */
#define BME68X_ODR_250_MS                         UINT8_C(3)

/* Standby time of 500ms */
#define BME68X_ODR_500_MS                         UINT8_C(4)

/* Standby time of 1s */
#define BME68X_ODR_1000_MS                        UINT8_C(5)

/* Standby time of 10ms */
#define BME68X_ODR_10_MS                          UINT8_C(6)

/* Standby time of 20ms */
#define BME68X_ODR_20_MS                          UINT8_C(7)

/* No standby time */
#define BME68X_ODR_NONE                           UINT8_C(8)

/* ------------------------Operating mode macros------------------------ */

/* Sleep operation mode */
#define BME68X_SLEEP_MODE                         UINT8_C(0)

/* Forced operation mode */
#define BME68X_FORCED_MODE                        UINT8_C(1)

/* Parallel operation mode */
#define BME68X_PARALLEL_MODE                      UINT8_C(2)

/* Sequential operation mode */
#define BME68X_SEQUENTIAL_MODE                    UINT8_C(3)

/* Length of the field */
#define BME68X_LEN_FIELD                          UINT8_C(17)

/* Length between two fields */
#define BME68X_LEN_FIELD_OFFSET                   UINT8_C(17)

/* Length of the configuration register */
#define BME68X_LEN_CONFIG                         UINT8_C(5)

/* Length of the interleaved buffer */
#define BME68X_LEN_INTERLEAVE_BUFF                UINT8_C(20)

/* ------------------------Gas measurement macros------------------------ */

/* Disable gas measurement */
#define BME68X_DISABLE_GAS_MEAS                   UINT8_C(0x00)

/* Enable gas measurement low */
#define BME68X_ENABLE_GAS_MEAS_L                  UINT8_C(0x01)

/* Enable gas measurement high */
#define BME68X_ENABLE_GAS_MEAS_H                  UINT8_C(0x02)

/* ------------------------Heater control macros------------------------ */

/* Enable heater */
#define BME68X_ENABLE_HEATER                      UINT8_C(0x00)

/* Disable heater */
#define BME68X_DISABLE_HEATER                     UINT8_C(0x01)

#ifdef BME68X_USE_FPU

/* 0 degree Celsius */
#define BME68X_MIN_TEMPERATURE                    INT16_C(0)

/* 60 degree Celsius */
#define BME68X_MAX_TEMPERATURE                    INT16_C(60)

/* 900 hecto Pascals */
#define BME68X_MIN_PRESSURE                       UINT32_C(90000)

/* 1100 hecto Pascals */
#define BME68X_MAX_PRESSURE                       UINT32_C(110000)

/* 20% relative humidity */
#define BME68X_MIN_HUMIDITY                       UINT32_C(20)

/* 80% relative humidity*/
#define BME68X_MAX_HUMIDITY                       UINT32_C(80)
#else

/* 0 degree Celsius */
#define BME68X_MIN_TEMPERATURE                    INT16_C(0)

/* 60 degree Celsius */
#define BME68X_MAX_TEMPERATURE                    INT16_C(6000)

/* 900 hecto Pascals */
#define BME68X_MIN_PRESSURE                       UINT32_C(90000)

/* 1100 hecto Pascals */
#define BME68X_MAX_PRESSURE                       UINT32_C(110000)

/* 20% relative humidity */
#define BME68X_MIN_HUMIDITY                       UINT32_C(20000)

/* 80% relative humidity*/
#define BME68X_MAX_HUMIDITY                       UINT32_C(80000)

#endif

#define BME68X_HEATR_DUR1                         UINT16_C(1000)
#define BME68X_HEATR_DUR2                         UINT16_C(2000)
#define BME68X_HEATR_DUR1_DELAY                   UINT32_C(1000000)
#define BME68X_HEATR_DUR2_DELAY                   UINT32_C(2000000)
#define BME68X_N_MEAS                             UINT8_C(6)
#define BME68X_LOW_TEMP                           UINT8_C(150)
#define BME68X_HIGH_TEMP                          UINT16_C(350)

/* ------------------------Mask macros------------------------ */
/* Mask for number of conversions */
#define BME68X_NBCONV_MSK                         UINT8_C(0X0f)

/* Mask for IIR filter */
#define BME68X_FILTER_MSK                         UINT8_C(0X1c)

/* Mask for ODR[3] */
#define BME68X_ODR3_MSK                           UINT8_C(0x80)

/* Mask for ODR[2:0] */
#define BME68X_ODR20_MSK                          UINT8_C(0xe0)

/* Mask for temperature oversampling */
#define BME68X_OST_MSK                            UINT8_C(0Xe0)

/* Mask for pressure oversampling */
#define BME68X_OSP_MSK                            UINT8_C(0X1c)

/* Mask for humidity oversampling */
#define BME68X_OSH_MSK                            UINT8_C(0X07)

/* Mask for heater control */
#define BME68X_HCTRL_MSK                          UINT8_C(0x08)

/* Mask for run gas */
#define BME68X_RUN_GAS_MSK                        UINT8_C(0x30)

/* Mask for operation mode */
#define BME68X_MODE_MSK                           UINT8_C(0x03)

/* Mask for res heat range */
#define BME68X_RHRANGE_MSK                        UINT8_C(0x30)

/* Mask for range switching error */
#define BME68X_RSERROR_MSK                        UINT8_C(0xf0)

/* Mask for new data */
#define BME68X_NEW_DATA_MSK                       UINT8_C(0x80)

/* Mask for gas index */
#define BME68X_GAS_INDEX_MSK                      UINT8_C(0x0f)

/* Mask for gas range */
#define BME68X_GAS_RANGE_MSK                      UINT8_C(0x0f)

/* Mask for gas measurement valid */
#define BME68X_GASM_VALID_MSK                     UINT8_C(0x20)

/* Mask for heater stability */
#define BME68X_HEAT_STAB_MSK                      UINT8_C(0x10)

/* Mask for SPI memory page */
#define BME68X_MEM_PAGE_MSK                       UINT8_C(0x10)

/* Mask for reading a register in SPI */
#define BME68X_SPI_RD_MSK                         UINT8_C(0x80)

/* Mask for writing a register in SPI */
#define BME68X_SPI_WR_MSK                         UINT8_C(0x7f)

/* Mask for the H1 calibration coefficient */
#define BME68X_BIT_H1_DATA_MSK                    UINT8_C(0x0f)

/* ------------------------Position macros------------------------ */

/* Filter bit position */
#define BME68X_FILTER_POS                         UINT8_C(2)

/* Temperature oversampling bit position */
#define BME68X_OST_POS                            UINT8_C(5)

/* Pressure oversampling bit position */
#define BME68X_OSP_POS                            UINT8_C(2)

/* Humidity oversampling bit position */
#define BME68X_OSH_POS                            UINT8_C(0)

/* ODR[3] bit position */
#define BME68X_ODR3_POS                           UINT8_C(7)

/* ODR[2:0] bit position */
#define BME68X_ODR20_POS                          UINT8_C(5)

/* Run gas bit position */
#define BME68X_RUN_GAS_POS                        UINT8_C(4)

/* Heater control bit position */
#define BME68X_HCTRL_POS                          UINT8_C(3)

/* SPI 3 wire interrupt enable bit position */
#define BME68X_SPI3WINTEN_POS                     UINT8_C(6)

/* mode bit position */
#define BME68X_MODE_POS                           UINT8_C(0)

/* Macro to combine two 8 bit data's to form a 16 bit data */
#define BME68X_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/* Macro to set bits */
#define BME68X_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

/* Macro to get bits */
#define BME68X_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))

/* Macro to set bits starting from position 0 */
#define BME68X_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

/* Macro to get bits starting from position 0 */
#define BME68X_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))


#endif /* BME68X_DEFS_H_ */
/*! @endcond */