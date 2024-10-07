/**\
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * The license is available at root folder
 *
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include "bme280.h"
#include "common.h"

/******************************************************************************/
/*!                       Macros                                              */

#define SAMPLE_COUNT  UINT8_C(5)
#define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to get compensated humidity data.
 *
 *  @param[in] period   : Contains the delay in microseconds.
 *  @param[in] dev      : Structure instance of bme280_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_humidity(uint32_t period, struct bme280_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    int8_t rslt;
    uint32_t period;
    struct bme280_dev dev;
    struct bme280_settings settings;

    /* X-HEEP Set-up */
    // Initialize the DMA
    dma_init(NULL);
    
    soc_ctrl_t soc_ctrl;
    soc_ctrl.base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS);

    if ( get_spi_flash_mode(&soc_ctrl) == SOC_CTRL_SPI_FLASH_MODE_SPIMEMIO ) {
        PRINTF("This application cannot work with the memory mapped SPI FLASH"
            "module - do not use the FLASH_EXEC linker script for this application\n");
        return EXIT_SUCCESS;
    }

    // Pick the correct spi device based on simulation type
    spi_host_t* spi;
    spi = spi_host1;

    // Configure SPI on CSID 0
    configure_bme280_spi(spi);

    // Enable SPI host device
    spi_set_enable(spi, true);

    // Enable SPI output
    spi_output_enable(spi, true);

    // Set CSID
    spi_set_csid(spi, 0);

    /* Interface selection is to be updated as parameter
     * For I2C :  BME280_I2C_INTF
     * For SPI :  BME280_SPI_INTF
     */
    rslt = bme280_interface_selection(&dev, BME280_SPI_INTF, spi);
    bme280_error_codes_print_result("bme280_interface_selection", rslt);

    rslt = bme280_init(&dev);
    bme280_error_codes_print_result("bme280_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&settings, &dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
    bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(&period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

    printf("\nHumidity calculation (Data displayed are compensated values)\n");
    printf("Measurement time : %lu us\n\n", (long unsigned int)period);

    rslt = get_humidity(period, &dev);
    bme280_error_codes_print_result("get_humidity", rslt);

    return 0;
}

/*!
 *  @brief This internal API is used to get compensated humidity data.
 */
static int8_t get_humidity(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

#ifdef BME280_DOUBLE_ENABLE
            printf("Humidity[%d]:   %lf %%RH\n", idx, comp_data.humidity);
#else
            printf("Humidity[%d]:   %lu %%RH\n", idx, (long unsigned int)comp_data.humidity);
#endif
            idx++;
        }
    }

    return rslt;
}