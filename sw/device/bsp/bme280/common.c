/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "bme280.h"
#include "common.h"

/******************************************************************************/
/*!                               Macros                                      */

#define BME280_SHUTTLE_ID  UINT8_C(0x33)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map 
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    //TBD
    return BME280_OK;
}

/*!
 * I2C write function map 
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    //TBD
    return BME280_OK;
}

/*!
 * SPI read function map 
 */
BME280_INTF_RET_TYPE bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    //printf("SPI Reading...\n");
    //printf("Writing first byte [0x%x] to SPI TX FIFO\n", reg_addr);
    // Write the register address to initiate read
    result = spi_write_byte(spi, reg_addr);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return BME280_E_COMM_FAIL;
    }
    //spi_wait_for_ready(spi);

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_read_2 = spi_create_command((spi_command_t){
        .len        = length,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_BIDIR      // Read only
    });
    result = spi_set_command(spi, cmd_read_2);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI read command failed\n");
        return BME280_E_COMM_FAIL;
    }
    spi_wait_for_ready(spi);

    bme280_delay_us(BME280_SPICOMM_DELAY, NULL);

    //printf("Getting [%d] bytes from SPI RX FIFO\n", length);
    // Read the requested number of bytes
    uint32_t read_value;
    for (uint32_t i = 0; i < length; i++) {
        result = spi_read_word(spi, &read_value);
        if (result != SPI_FLAG_OK) {
            printf("The SPI RX fifo only contained [%d] bytes\n", i-1);
            return BME280_E_COMM_FAIL;
        }
        reg_data[i] = (uint8_t)((read_value >> 8) & 0xFF);
    }

    //printf("READ VALUE: 0x%x,\n", read_value);
    //printf("REG_data VALUE: 0x%x,\n", reg_data[0]);

    return BME280_OK;
}

/*!
 * SPI write function map 
 */
BME280_INTF_RET_TYPE bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    //printf("SPI Writing...\n");

    // Write the register address
    result = spi_write_byte(spi, reg_addr);
    if (result != SPI_FLAG_OK) {
        return BME280_E_COMM_FAIL;
    }
    //spi_wait_for_ready(spi);
    // Set up segment parameters -> read length bytes
    const uint32_t cmd_write1 = spi_create_command((spi_command_t){
        .len        = 0,          // len bytes
        .csaat      = true,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_TX_ONLY      // Read only
    });
    spi_set_command(spi, cmd_write1);
    spi_wait_for_ready(spi);

    // Write the data bytes
    //for (uint32_t i = 0; i < length; i++) {
    //    result = spi_write_byte(spi, reg_data[i]);
    //    if (result != SPI_FLAG_OK) {
    //        return BME280_E_COMM_FAIL;
    //    }
    //    //spi_wait_for_tx_not_full(spi);
    //}
    // Write the register address
    result = spi_write_byte(spi, reg_data[0]);
    if (result != SPI_FLAG_OK) {
        return BME280_E_COMM_FAIL;
    }

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_write = spi_create_command((spi_command_t){
        .len        = length-1,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_TX_ONLY      // Read only
    });
    spi_set_command(spi, cmd_write);
    spi_wait_for_ready(spi);

    bme280_delay_us(BME280_SPICOMM_DELAY, NULL);

    return BME280_OK;
}

/*!
 * Delay function for bme280
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    // Calculate the number of NOPs needed for the given delay period (period in microseconds)
    uint32_t nop_count = (period*10000)/(666);
    for (volatile uint32_t i = 0; i < nop_count; i++) {
        __asm__ volatile ("nop");
    }
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BME280_E_COMM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BME280_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BME280_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
    else
    {
        printf("%s, SUCCESS! \n", api_name);
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_interface_selection(struct bme280_dev *dev, uint8_t intf,  void *intf_ptr)
{
    int8_t rslt = BME280_OK;

    if (dev != NULL)
    {
      printf("Configuring the interface for the BME280...\n");

        /* Bus configuration : I2C */
        if (intf == BME280_I2C_INTF)
        {
            printf("I2C Interface not implemented\n");
            rslt = BME280_E_COMM_FAIL;
            //TBD
        }
        /* Bus configuration : SPI */
        else if (intf == BME280_SPI_INTF)
        {
            printf("SPI Interface\n");

            dev->read = bme280_spi_read;
            dev->write = bme280_spi_write;
            dev->intf = BME280_SPI_INTF;
        }

        /* Holds SPI interface pointer */
        dev->intf_ptr = intf_ptr;

        /* Configure delay in microseconds */
        dev->delay_us = bme280_delay_us;
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to configure with user parameters the SPI
 */
void configure_bme280_spi(spi_host_t* spi) {
    // Configure SPI clock
    uint32_t core_clk = soc_ctrl_peri->SYSTEM_FREQUENCY_HZ;
    uint16_t clk_div = 0;
    printf("Core clock: [%d] Hz\n", core_clk);
    if(BME280_MAX_FREQ < core_clk/2){
        clk_div = (core_clk/(BME280_MAX_FREQ) - 2)/2; // The value is truncated
        if (core_clk/(2 + 2 * clk_div) > BME280_MAX_FREQ) clk_div += 1; // Adjust if the truncation was not 0
    }
    // SPI Configuration
    // Configure chip 0 (flash memory)
    const uint32_t chip_cfg = spi_create_configopts((spi_configopts_t){
        .clkdiv     = clk_div,
        .csnidle    = 0xF,
        .csntrail   = 0x4F,
        .csnlead    = 0x4F,
        .fullcyc    = true,
        .cpha       = 0,
        .cpol       = 0
    });
    if(spi_set_configopts(spi, 0, chip_cfg) != SPI_FLAG_OK)
    {
        printf("Config for SPI failed\n");
    }
}