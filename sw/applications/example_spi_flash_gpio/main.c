#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "core_v_mini_mcu.h"
#include "gpio.h"
#include "pad_control.h"
#include "pad_control_regs.h"
#include "rv_plic.h"
#include "x-heep.h"
#include "timer_sdk.h"
#include "w25q128jw.h"
#include "soc_ctrl.h"
#include "csr.h"
#include "x-heep.h"

#define PRINTF_IN_FPGA  1
#define PRINTF_IN_SIM   0
#if TARGET_SIM && PRINTF_IN_SIM
    #define PRINTF(fmt, ...) printf(fmt, ## __VA_ARGS__)
#elif PRINTF_IN_FPGA && !TARGET_SIM
    #define PRINTF(fmt, ...) printf(fmt, ## __VA_ARGS__)
#else
    #define PRINTF(...)
#endif

#define GPIO_BUTTON         10
#define GPIO_INTR           GPIO_INTR_10
#define LENGTH              1024
#define DEBOUNCE_DELAY_US   50000  // 50 ms

// Flash target buffer (linked into SPI flash)
uint8_t __attribute__((section(".xheep_data_flash_only"))) __attribute__((aligned(16)))
flash_write_target[LENGTH];

// Data to write (in RAM)
uint8_t flash_write_data[LENGTH];

// Buffer to read from flash (in RAM)
uint8_t flash_read_data[LENGTH];

// Button interrupt flags
volatile uint8_t button_pressed = 0;
volatile uint8_t button_debounced = 0;

// ISR
void handler_button_press() {
    if (!button_debounced) {
        button_pressed = 1;
        button_debounced = 1;  // Block further triggers until released
    }
}

// Buffer comparison
uint32_t check_result(uint8_t *expected, uint8_t *actual, uint32_t len) {
    uint32_t errors = 0;
    for (uint32_t i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            PRINTF("Error @ %u: expected 0x%02X, got 0x%02X\n", i, expected[i], actual[i]);
            errors++;
            if (errors > 10) {
                PRINTF("...too many errors, stopping check.\n");
                break;
            }
        }
    }
    return errors;
}

int main(void) {
    PRINTF("==== Button-triggered SPI Flash Write+Read ====\n");

    // Fill flash_write_data with known pattern
    for (uint32_t i = 0; i < LENGTH; i++) {
        flash_write_data[i] = (uint8_t)(i & 0xFF);
    }

    // Init SPI
    spi_host_t *spi = spi_flash;
    if (w25q128jw_init(spi) != FLASH_OK) {
        PRINTF("[ERROR] SPI init failed!\n");
        return EXIT_FAILURE;
    }

    // Init PLIC
    if (plic_Init() != kPlicOk ||
        plic_irq_set_priority(GPIO_INTR, 1) != kPlicOk ||
        plic_irq_set_enabled(GPIO_INTR, kPlicToggleEnabled) != kPlicOk) {
        PRINTF("[ERROR] PLIC setup failed!\n");
        return EXIT_FAILURE;
    }

    // Enable global interrupt
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    CSR_SET_BITS(CSR_REG_MIE, 1 << 11);

    // GPIO setup
    gpio_cfg_t cfg_btn = {
        .pin = GPIO_BUTTON,
        .mode = GpioModeIn,
        .en_input_sampling = true,
        .en_intr = true,
        .intr_type = GpioIntrEdgeRising
    };
    if (gpio_config(cfg_btn) != GpioOk) {
        PRINTF("[ERROR] GPIO config failed!\n");
        return EXIT_FAILURE;
    }
    gpio_assign_irq_handler(GPIO_INTR, handler_button_press);

    // Timer init
    soc_ctrl_t soc_ctrl = { .base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS) };
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);
    timer_cycles_init();

    PRINTF("System ready. Press the button on GPIO %d to perform flash write+read.\n", GPIO_BUTTON);

    while (1) {
        if (button_pressed) {
            button_pressed = 0;
            PRINTF("\n[EVENT] Button pressed!\n");
            //timer_wait_us(DEBOUNCE_DELAY_US);  // debounce delay
            PRINTF("[INFO] Debounce delay passed.\n");

            // Wait until button is released
            bool still_pressed = true;
            while (still_pressed) {
                gpio_read(GPIO_BUTTON, &still_pressed);
            }
            PRINTF("[INFO] Button released, ready for next trigger.\n");
            button_debounced = 0;

            // Resolve flash address
            uintptr_t flash_offset = heep_get_flash_address_offset(flash_write_target);
            PRINTF("[INFO] Flash offset: 0x%08lx\n", flash_offset);

            memset(flash_read_data, 0, LENGTH);

            timer_start();

            // Write and read flash
            w25q_error_codes_t err = w25q128jw_erase_and_write_standard(
                (void *)flash_offset, flash_write_data, LENGTH
            );
            if (err != FLASH_OK) {
                PRINTF("[ERROR] Flash write failed! Code: %d\n", err);
                continue;
            }

            err = w25q128jw_read_standard((void *)flash_offset, flash_read_data, LENGTH);
            if (err != FLASH_OK) {
                PRINTF("[ERROR] Flash read failed! Code: %d\n", err);
                continue;
            }

            uint32_t cycles = timer_stop();
            uint32_t time_us = get_time_from_cycles(cycles);
            uint32_t errors = check_result(flash_write_data, flash_read_data, LENGTH);

            PRINTF("[RESULT] Transaction took %u cycles (%u us @ %u Hz). %s\n",
                   cycles, time_us, freq_hz,
                   errors == 0 ? "✅ PASS" : "❌ FAIL");

            // Print preview of read buffer
            PRINTF("[INFO] First 16 bytes of read buffer:\n");
            for (uint32_t i = 0; i < 16; i++) {
                PRINTF("0x%02X ", flash_read_data[i]);
            }
            PRINTF("\n");
        }
    }

    return EXIT_SUCCESS;
}
