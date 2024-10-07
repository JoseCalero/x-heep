
/******************************************************************************/
/*!                 Header Files                                              */
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include "ads1298.h"

/******************************************************************************/
/*!                       Macros                                              */

#define SAMPLE_COUNT  UINT32_C(1000*10)
#define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)

/******************************************************************************/
/*!                       Global Variables                                    */

plic_result_t plic_res;

uint8_t gpio_intr_flag = 0;

/******************************************************************************/
/*!         Static Function Declaration                                       */

/******************************************************************************/
/*!            Functions                                        */

/*!
 *  @brief This internal API is used to get compensated temperature data.
 */
static int get_ECG(void * intf_ptr);

/* This function starts the execution of program. */
int main(void)
{
    int rslt;

    /* X-HEEP Set-up */

    //Init pad control and PLIC
    pad_control_t pad_control;
    pad_control.base_addr = mmio_region_from_addr((uintptr_t)PAD_CONTROL_START_ADDRESS);
    // rv_plic_params.base_addr = mmio_region_from_addr((uintptr_t)RV_PLIC_START_ADDRESS);
    plic_res = plic_Init();
    if (plic_res != kPlicOk) {
        PRINTF("Init PLIC failed\n\r;");
        return -1;
    }

   plic_res = plic_irq_set_priority(ADS_INT_INTR, 1);
    if (plic_res != kPlicOk) {
        PRINTF("Failed\n\r;");
        return -1;
    }

    // Enabling irqs
    PRINTF("Enabling ISRs...\n");
    plic_res = plic_irq_set_enabled(ADS_INT_INTR, kPlicToggleEnabled);
    if (plic_res != kPlicOk) {
        PRINTF("Failed\n\r;");
        return -1;
    }

    // Enable interrupt on processor side
    // Enable global interrupt for machine-level interrupts
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    // Set mie.MEIE bit to one to enable machine-level external interrupts
    const uint32_t mask = 1 << 11;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    //gpio_reset_all();
    gpio_result_t gpio_res;
    
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

    // Configure SPI connection on CSID 0
    configure_ads1298_spi(spi);

    // Enable SPI host device
    spi_set_enable(spi, true);

    // Enable SPI output
    spi_output_enable(spi, true);

    // Set CSID
    spi_set_csid(spi, 0);

    // Set Reset and Start Pins
    gpio_cfg_t cfg_reset = {
         .pin = ADS_PIN_Reset,
         .mode = GpioModeOutPushPull,
    };
    gpio_res = gpio_config(cfg_reset);
    if (gpio_res != GpioOk) {
         PRINTF("Failed Reset GPIO config\n;");
         return -1;
    }

    //set reset pin to low and high
    gpio_res = gpio_write(ADS_PIN_Reset, false);
    if (gpio_res != GpioOk) {
         PRINTF("Failed Setting Reset GPIO\n;");
         return -1;
    }
    ads1298_delay_us(100*1000);
    gpio_write(ADS_PIN_Reset, true);
    ads1298_delay_us(100*1000);

    gpio_cfg_t cfg_start = {
         .pin = ADS_PIN_Start,
         .mode = GpioModeOutPushPull,
         .en_intr = false,
    };
    gpio_res = gpio_config(cfg_start);
    if (gpio_res != GpioOk) {
         PRINTF("Failed Start GPIO Config\n;");
         return -1;
    }

    //set the start pin to low
    gpio_res = gpio_write(ADS_PIN_Start, false);
    if (gpio_res != GpioOk) {
         PRINTF("Failed Setting Start GPIO\n;");
         return -1;
    }

    // Init ADS1298
    PRINTF("Init and Reset device...\n");
    rslt = ADS1298_init(spi);

    // Config ADS1298
    PRINTF("Config device...\n");
    rslt = ADS1298_config(spi);

    gpio_cfg_t cfg_in = {
         .pin = ADS_INT_PIN,
         .mode = GpioModeIn,
         .en_input_sampling = true,
         .en_intr = true,
         .intr_type = GpioIntrEdgeFalling
    };
    gpio_res = gpio_config(cfg_in);
    if (gpio_res != GpioOk) {
         PRINTF("Failed\n;");
         return -1;
    }
    CSR_CLEAR_BITS(CSR_REG_MSTATUS, 0x8);
    gpio_assign_irq_handler(ADS_INT_INTR, &ADS1298_interrupt_handler);
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    new_data_rdy = 0;

    PRINTF("Reseting flags...\n");
    PRINTF("Starting Measurement...\n");
    //new_data_rdy = 0;
    //gpio_intr_clear_stat(ADS_INT_PIN);
    //gpio_res = gpio_intr_en (ADS_INT_PIN, GpioIntrEdgeFalling);
    //if (gpio_res != GpioOk) {
    //     PRINTF("Failed Setting DRYin GPIO\n;");
    //     return -1;
    //}
    //ADS1298_send_cmd(WAKEUP_CMD, spi);
    //ads1298_delay_us(10000);
    gpio_write(ADS_PIN_Start, true);
    //ads1298_delay_us(10000);
    //ADS1298_send_cmd(STOP_CMD, spi);
    //ads1298_delay_us(2*1000);
    //ADS1298_send_cmd(START_CMD, spi);
    //ads1298_delay_us(2*1000);
    //ADS1298_send_cmd(RDATAC_CMD, spi);
    //ads1298_delay_us(10000);

    // Get ECG from CH1
    PRINTF("Getting ECG...\n");
    rslt = get_ECG(spi);

    PRINTF("Finishing Measurement...\n");
    gpio_write(ADS_PIN_Start, false);
    ads1298_delay_us(100*1000);

    return 0;
}

/*!
 *  @brief This internal API is used to get compensated temperature data.
 */
static int get_ECG(void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    int8_t idx = 0;
    int rslt = 0;
    uint8_t data_ecg[3];
    uint32_t data_ecg_concat = 0;

    //new_data_rdy = 0;
    //CSR_CLEAR_BITS(CSR_REG_MSTATUS, 0x8);
    while (idx < SAMPLE_COUNT)
    {

        // Read data in non-continuous mode...
        //CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
        rslt = ADS1298_thread_func(spi, data_ecg);
        //CSR_CLEAR_BITS(CSR_REG_MSTATUS, 0x8);
        if (rslt)
        {
            return rslt;
        }
        data_ecg_concat = ((data_ecg[0] & 0x0000FF) << 16) | ((data_ecg[1] & 0x0000FF) << 8) | ((data_ecg[2] & 0x0000FF));
        printf("ECG[%d]:   %ld a.u.\n", idx, data_ecg_concat);
        idx++;
    }

    return 0;
}
