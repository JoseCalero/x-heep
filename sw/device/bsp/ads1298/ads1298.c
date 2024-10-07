/***************************************************************************/
/***************************************************************************/

/**
* @file   ADS1298.c
* @date   DD/MM/YY
* @brief  This is the main header of ADS1298.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _ADS1298_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "ads1298.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/* ADS1298 configuration */
#define VCONF_ADS1298_FS        VCONF_ADS1298_FS_1000
#define VCONF_ADS1298_GAIN      VCONF_ADS1298_GAIN_12
#define VCONF_ADS1298_SUBSAMPLING_FACTOR    4

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! Buffer for the SPI transfer */
uint8_t ads_tx_buf[20];

uint8_t new_data_rdy = 0;

/*! Flag to check if the last transfer was successful */
bool ads_last_transfer_succeeded = false;

int vconf_ads1298_fs = VCONF_ADS1298_FS;
int vconf_ads1298_gain = VCONF_ADS1298_GAIN;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int ADS1298_init(void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    int ret_val = 0;

    // Reset the ADS1298
    printf("Sending reset command\n");
    ADS1298_send_cmd(RESET_CMD, spi);
    ads1298_delay_us(10000);
    ADS1298_send_cmd(START_CMD, spi);
    ads1298_delay_us(10000);
    ADS1298_send_cmd(WAKEUP_CMD, spi);
    ads1298_delay_us(10000);

    // stop the continuous data transfer
    ADS1298_send_cmd(SDATAC_CMD, spi);
    ads1298_delay_us(300*1000);

    // check if the ADS1298 is present
    if(ADS1298_check_present(spi) != 0)
    {
        return -2;
    }

    printf("ADS1298_init SUCCESS\n");
    return ret_val;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_check_present(void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    uint8_t len_data = 3;
    uint8_t data[3];
    int err_code = 0;
    data[2] = 0;

    // Check if the ADS1298 is present by reading the ID register
    printf("Reading ID register...\n");
    //do
    //{
        err_code = ADS1298_read_reg(REG_ID_Addr, data, spi);
    //} while (data[0]!=0x92);
    
    
    if (err_code != 0)
    {
        printf("ADS1298_read_reg failed with error code: %d\n", err_code);
        return -1;
    }

    if (data[0] != 0x92)
    {
        printf("ADS1298 not present\n");
        return -1;
    }

    printf("ADS1298 present\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_send_cmd(uint8_t cmd, void *intf_ptr )
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    // Write the register address
    result = spi_write_byte(spi, cmd);
    if (result != SPI_FLAG_OK) {
        return -2;
    }

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_write = spi_create_command((spi_command_t){
        .len        = 0,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_TX_ONLY      // Read only
    });
    result = spi_set_command(spi, cmd_write);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI read command failed\n");
        return -2;
    }
    spi_wait_for_ready(spi);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_read_reg(uint8_t addr, uint8_t *data, void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x20;
    ads_tx_buf[1] = 0x00;
    ads_tx_buf[2] = 0x00;

    //printf("SPI Reading...\n");
    //printf("Writing first byte [0x%x] to SPI TX FIFO\n", reg_addr);
    // Write the register address to initiate read
    result = spi_write_byte(spi, ads_tx_buf[0]);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }

    result = spi_write_byte(spi, ads_tx_buf[1]);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_read_2 = spi_create_command((spi_command_t){
        .len        = 2,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_BIDIR      // Read only
    });
    result = spi_set_command(spi, cmd_read_2);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI read command failed\n");
        return -2;
    }
    spi_wait_for_ready(spi);

    ads1298_delay_us(100);

    //printf("Getting 1 bytes from SPI RX FIFO\n");
    // Read the requested number of bytes
    uint32_t read_value;
    for (uint32_t i = 0; i < 1; i++) {
        result = spi_read_word(spi, &read_value);
        if (result != SPI_FLAG_OK) {
            printf("The SPI RX fifo only contained [%d] bytes\n", i-1);
            return -2;
        }
        data[i] = (uint8_t)((read_value >> 16) & 0xFF);
        //data[i] = (uint8_t)(read_value & 0xFF);
        //printf("READ VALUE: 0x%x,\n", read_value);
        //printf("REG_data VALUE: 0x%x,\n", data[i]);
    }

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_write_reg(uint8_t addr, uint8_t data, void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x40;
    ads_tx_buf[1] = 0x00;
    ads_tx_buf[2] = data;
    //printf("SPI Writing...\n");
    //printf("Writing first byte [0x%x] to SPI TX FIFO\n", reg_addr);
    // Write the register address to initiate read
    result = spi_write_byte(spi, ads_tx_buf[0]);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }
    result = spi_write_byte(spi, ads_tx_buf[1]);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }
    result = spi_write_byte(spi, ads_tx_buf[2]);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_write = spi_create_command((spi_command_t){
        .len        = 2,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_TX_ONLY      // Read only
    });
    spi_set_command(spi, cmd_write);
    spi_wait_for_ready(spi);

    ads1298_delay_us(100);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_read_data(uint8_t *data, void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    // set the transfer descriptor
    ads_tx_buf[0] = RDATA_CMD;
    // ads_tx_buf, 1, data, 1+27);

    //printf("SPI Reading...\n");
    //printf("Writing first byte [0x%x] to SPI TX FIFO\n", reg_addr);
    // Write the register address to initiate read
    result = spi_write_byte(spi, ads_tx_buf);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI write fifo failed\n");
        return -2;
    }
    //spi_wait_for_ready(spi);

    // Set up segment parameters -> read length bytes
    const uint32_t cmd_read_2 = spi_create_command((spi_command_t){
        .len        = 6,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_BIDIR      // Read only
    });
    result = spi_set_command(spi, cmd_read_2);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI read command failed\n");
        return -2;
    }
    spi_wait_for_ready(spi);

    ads1298_delay_us(100);

    //printf("Getting [%d] bytes from SPI RX FIFO\n", length);
    // Read the requested number of bytes
    uint32_t read_value;
    for (uint32_t i = 0; i < 7; i++) {
        result = spi_read_word(spi, &read_value);
        if (result != SPI_FLAG_OK) {
            printf("The SPI RX fifo only contained [%d] bytes\n", i-1);
            return -2;
        }
        data[i] = (uint8_t)((read_value >> 16) & 0xFF);
    }

    return 0;
}

int ADS1298_read_datac(uint8_t *data, void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    spi_return_flags_e result;

    // set the transfer descriptor
    ads_tx_buf[0] = RDATA_CMD;
    // ads_tx_buf, 1, data, 1+27);

    //printf("SPI Reading...\n");
    // Set up segment parameters -> read length bytes
    const uint32_t cmd_read_2 = spi_create_command((spi_command_t){
        .len        = 26,          // len bytes
        .csaat      = false,             // End command
        .speed      = SPI_SPEED_STANDARD, // Single speed
        .direction  = SPI_DIR_RX_ONLY     // Read only
    });
    result = spi_set_command(spi, cmd_read_2);
    if (result != SPI_FLAG_OK) 
    {
        printf("SPI read command failed\n");
        return -2;
    }
    spi_wait_for_ready(spi);

    ads1298_delay_us(100);

    //printf("Getting [%d] bytes from SPI RX FIFO\n", length);
    // Read the requested number of bytes
    uint32_t read_value;
    for (uint32_t i = 0; i < 27; i++) {
        result = spi_read_word(spi, &read_value);
        if (result != SPI_FLAG_OK) {
            printf("The SPI RX fifo only contained [%d] bytes\n", i-1);
            return -2;
        }
        data[i] = (uint8_t)((read_value >> 16) & 0xFF);
        printf("READ VALUE: 0x%x,\n", read_value);
        printf("REG_data VALUE: 0x%x,\n", data[i]);
    }

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_config(void * intf_ptr)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;

     ADS1298_REG reg;
    reg.REG_CONFIG1.b.HR = 0b0;
    reg.REG_CONFIG1.b.DAISY_EN = 0b0;
    reg.REG_CONFIG1.b.CLK_EN = 0b0;
    reg.REG_CONFIG1.b._reserved_3 = 0b0;
    reg.REG_CONFIG1.b._reserved_4 = 0b0;
    reg.REG_CONFIG1.b.DR = vconf_ads1298_fs;
    ADS1298_write_reg(REG_CONFIG1_Addr, reg.REG_CONFIG1.w, spi);
    reg.REG_CONFIG2.b._reserved_7 = 0b0;
    reg.REG_CONFIG2.b._reserved_6 = 0b0;
    reg.REG_CONFIG2.b.WCT_CHOP = 0b0;
    reg.REG_CONFIG2.b.INT_TEST = 0b1;
    reg.REG_CONFIG2.b._reserved_3 = 0b0;
    reg.REG_CONFIG2.b.TEST_AMP = 0b0;
    reg.REG_CONFIG2.b.TEST_FREQ = 0b0;
    ADS1298_write_reg(REG_CONFIG2_Addr, reg.REG_CONFIG2.w, spi);
    reg.REG_CONFIG3.b.PD_REFBUF = 0b1;
    reg.REG_CONFIG3.b._reserved_6 = 0b1;
    reg.REG_CONFIG3.b.VREF_4V = 0b1;
    reg.REG_CONFIG3.b.RLD_MEAS = 0b0;
    reg.REG_CONFIG3.b.RLDREF_INT = 0b1;
    reg.REG_CONFIG3.b.PD_RLD = 0b1;
    reg.REG_CONFIG3.b.RLD_LOFF_SENS = 0b0;
    reg.REG_CONFIG3.b.RLD_STAT = 0b0;
    ADS1298_write_reg(REG_CONFIG3_Addr, reg.REG_CONFIG3.w, spi);
    reg.REG_LOFF.b.COMP_TH = 0b000;
    reg.REG_LOFF.b.VLEAD_OFF_EN = 0b0;
    reg.REG_LOFF.b.ILEAD_OFF = 0b00;
    reg.REG_LOFF.b.FLEAD_OFF = 0b00;
    ADS1298_write_reg(REG_LOFF_Addr, reg.REG_LOFF.w, spi);
    reg.REG_CH1SET.b.PD1 = 0b0;
    reg.REG_CH1SET.b.GAIN1 = vconf_ads1298_gain;
    reg.REG_CH1SET.b._reserved_3 = 0b0;
    reg.REG_CH1SET.b.MUX1 = 0b000;
    ADS1298_write_reg(REG_CH1SET_Addr, reg.REG_CH1SET.w, spi);
    reg.REG_CH2SET.b.PD2 = 0b1;
    reg.REG_CH2SET.b.GAIN2 = vconf_ads1298_gain;
    reg.REG_CH2SET.b._reserved_3 = 0b0;
    reg.REG_CH2SET.b.MUX2 = 0b001;
    ADS1298_write_reg(REG_CH2SET_Addr, reg.REG_CH2SET.w, spi);
    reg.REG_CH3SET.b.PD3 = 0b1;
    reg.REG_CH3SET.b.GAIN3 = vconf_ads1298_gain;
    reg.REG_CH3SET.b._reserved_3 = 0b0;
    reg.REG_CH3SET.b.MUX3 = 0b001;
    ADS1298_write_reg(REG_CH3SET_Addr, reg.REG_CH3SET.w, spi);
    reg.REG_CH4SET.b.PD4 = 0b1;
    reg.REG_CH4SET.b.GAIN4 = vconf_ads1298_gain;
    reg.REG_CH4SET.b._reserved_3 = 0b0;
    reg.REG_CH4SET.b.MUX4 = 0b001;
    ADS1298_write_reg(REG_CH4SET_Addr, reg.REG_CH4SET.w, spi);
    reg.REG_CH5SET.b.PD5 = 0b1;
    reg.REG_CH5SET.b.GAIN5 = vconf_ads1298_gain;
    reg.REG_CH5SET.b._reserved_3 = 0b0;
    reg.REG_CH5SET.b.MUX5 = 0b001;
    ADS1298_write_reg(REG_CH5SET_Addr, reg.REG_CH5SET.w, spi);
    reg.REG_CH6SET.b.PD6 = 0b1;
    reg.REG_CH6SET.b.GAIN6 = vconf_ads1298_gain;
    reg.REG_CH6SET.b._reserved_3 = 0b0;
    reg.REG_CH6SET.b.MUX6 = 0b001;
    ADS1298_write_reg(REG_CH6SET_Addr, reg.REG_CH6SET.w, spi);
    reg.REG_CH7SET.b.PD7 = 0b1;
    reg.REG_CH7SET.b.GAIN7 = vconf_ads1298_gain;
    reg.REG_CH7SET.b._reserved_3 = 0b0;
    reg.REG_CH7SET.b.MUX7 = 0b001;
    ADS1298_write_reg(REG_CH7SET_Addr, reg.REG_CH7SET.w, spi);
    reg.REG_CH8SET.b.PD8 = 0b1;
    reg.REG_CH8SET.b.GAIN8 = vconf_ads1298_gain;
    reg.REG_CH8SET.b._reserved_3 = 0b0;
    reg.REG_CH8SET.b.MUX8 = 0b001;
    ADS1298_write_reg(REG_CH8SET_Addr, reg.REG_CH8SET.w, spi);

    printf("ADS1298_config SUCCESS\n");

    return 0;
}

/****************************************************************************/
/****************************************************************************/

__attribute__((optimize("O0"))) void ADS1298_interrupt_handler(void)
{
    // Notify the thread that new data is ready
    //k_sem_give(&new_data_rdy);
    //printf("GP!\n");
    new_data_rdy = 1;
    //gpio_intr_clear_stat(ADS_INT_PIN);
    //CSR_CLEAR_BITS(CSR_REG_MSTATUS, 0x8);
    //CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_thread_func(void * intf_ptr, uint8_t *data_ecg)
{
    spi_host_t *spi = (spi_host_t *)intf_ptr;
    uint8_t data[7]; //24 bits header + 24 bits per channel 
    int err_code;

    // Wait for the new data to be ready
    while(new_data_rdy == 0);

    err_code = ADS1298_read_data(data, spi);
    if (err_code != 0)
    {
        printf("ADS1298_read_data failed with error code: %d\n", err_code);
        return err_code;
    }
    

    new_data_rdy = 0;
    data_ecg = &data[4];
    return 0;
}

/****************************************************************************/
/****************************************************************************/

/*!
 * Delay function for bme280
 */
void ads1298_delay_us(uint32_t period)
{
    // Calculate the number of NOPs needed for the given delay period (period in microseconds)
    uint32_t nop_count = (period*10000)/(666);
    for (volatile uint32_t i = 0; i < nop_count; i++) {
        __asm__ volatile ("nop");
    }
}

/*!
 *  @brief This API is used to configure with user parameters the SPI
 */
void configure_ads1298_spi(spi_host_t* spi) {
    // Configure SPI clock
    uint32_t core_clk = soc_ctrl_peri->SYSTEM_FREQUENCY_HZ;
    uint16_t clk_div = 0;
    printf("Core clock: [%d] Hz\n", core_clk);
    // SPI Configuration
    // Configure chip 0 (flash memory)
    const uint32_t chip_cfg = spi_create_configopts((spi_configopts_t){
        .clkdiv     = 8,
        .csnidle    = 0x4F,
        .csntrail   = 0x03,
        .csnlead    = 0x03,
        .fullcyc    = false,
        .cpha       = 1,
        .cpol       = 0
    });
    if(spi_set_configopts(spi, 0, chip_cfg) != SPI_FLAG_OK)
    {
        printf("Config for SPI failed\n");
    }
}


/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/