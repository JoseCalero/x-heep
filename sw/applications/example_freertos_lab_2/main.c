/*
 * FreeRTOS Lab - Button-triggered SPI Flash Write+Read
 * Includes UART logging, timing benchmarks, heap stats, stack watermarks, and mutex protection
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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
#include "timers.h"

#define GPIO_BUTTON         10
#define GPIO_INTR           GPIO_INTR_10
#define FLASH_LENGTH        1024
#define TASK_STACK_SIZE     512
#define FLASH_TASK_PRIO     (tskIDLE_PRIORITY + 2)
#define BUTTON_TASK_PRIO    (tskIDLE_PRIORITY + 1)
#define CHECK_TASK_PRIO     (tskIDLE_PRIORITY + 1)

static QueueHandle_t xButtonQueue;
static TimerHandle_t xFlashTimer;
static SemaphoreHandle_t xFlashSem;

// Flash buffers
uint8_t flash_write_data[FLASH_LENGTH];
uint8_t flash_read_data[FLASH_LENGTH];
uint8_t __attribute__((section(".xheep_data_flash_only"))) __attribute__((aligned(16))) flash_write_target[FLASH_LENGTH];

// For logging with UART safely from multiple tasks
#define UART_LOCK()    taskENTER_CRITICAL()
#define UART_UNLOCK()  taskEXIT_CRITICAL()
#define UART_PRINTF(...) do { UART_LOCK(); printf(__VA_ARGS__); UART_UNLOCK(); } while(0)

// Helper to toggle LED for result indication (reuse LD5_R)
#define GPIO_LED        11
static void toggle_led_passfail(bool pass) {
    gpio_write(GPIO_LED, pass);
}

void gpio_button_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t event = 1;
    xQueueSendFromISR(xButtonQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vTaskButtonWait(void *pvParams) {
    uint8_t received;
    for (;;) {
        if (xQueueReceive(xButtonQueue, &received, portMAX_DELAY) == pdPASS) {
            UART_PRINTF("[EVENT] Button press received in task.\n");
            xTaskNotifyGive((TaskHandle_t)pvParams); // notify Flash task
        }
    }
}

void vTaskFlashHandler(void *pvParams) {
    spi_host_t *spi = spi_flash;
    soc_ctrl_t soc_ctrl = { .base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS) };
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);

    if (w25q128jw_init(spi) != FLASH_OK) {
        UART_PRINTF("[ERROR] SPI init failed!\n");
        vTaskSuspend(NULL);
    }

    // Fill known data
    for (uint32_t i = 0; i < FLASH_LENGTH; i++) flash_write_data[i] = (uint8_t)(i & 0xFF);

    uintptr_t flash_offset = heep_get_flash_address_offset(flash_write_target);
    UART_PRINTF("[INIT] Flash offset: 0x%08lx\n", flash_offset);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for button task

        memset(flash_read_data, 0, FLASH_LENGTH);
        uint32_t t_start = xTaskGetTickCount();

        // Currently, only one task uses the flash.
        // The semaphore is included as a forward-looking protection and
        // to demonstrate correct mutual exclusion techniques.
        xSemaphoreTake(xFlashSem, portMAX_DELAY);
        w25q_error_codes_t err = w25q128jw_erase_and_write_standard((void *)flash_offset, flash_write_data, FLASH_LENGTH);
        if (err != FLASH_OK) {
            UART_PRINTF("[ERROR] Flash write failed!\n");
            xSemaphoreGive(xFlashSem);
            continue;
        }

        err = w25q128jw_read_standard((void *)flash_offset, flash_read_data, FLASH_LENGTH);
        xSemaphoreGive(xFlashSem);

        if (err != FLASH_OK) {
            UART_PRINTF("[ERROR] Flash read failed!\n");
            continue;
        }

        uint32_t t_end = xTaskGetTickCount();
        uint32_t t_elapsed = (t_end - t_start) * portTICK_PERIOD_MS;
        UART_PRINTF("[BENCH] Flash R/W took %u ms.\n", t_elapsed);
        UART_PRINTF("[HEAP] Free heap: %u bytes\n", xPortGetFreeHeapSize());
        UART_PRINTF("[STACK] FlashTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));

        xTaskNotifyGive((TaskHandle_t)pvParams); // Notify checker task
    }
}

void vTaskCheckCompare(void *pvParams) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t errors = 0;
        for (uint32_t i = 0; i < FLASH_LENGTH; i++) {
            if (flash_write_data[i] != flash_read_data[i]) {
                errors++;
                if (errors < 10) UART_PRINTF("[MISMATCH] @%u: 0x%02X != 0x%02X\n", i, flash_write_data[i], flash_read_data[i]);
            }
        }
        bool pass = (errors == 0);
        UART_PRINTF("[RESULT] %s\n", pass ? "PASS ✅" : "FAIL ❌");
        UART_PRINTF("[STACK] CheckerTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
        toggle_led_passfail(pass);
    }
}

void app_main(void) {
    gpio_cfg_t cfg_btn = {
        .pin = GPIO_BUTTON,
        .mode = GpioModeIn,
        .en_input_sampling = true,
        .en_intr = true,
        .intr_type = GpioIntrEdgeRising
    };
    gpio_cfg_t cfg_led = {
        .pin = GPIO_LED,
        .mode = GpioModeOutPushPull
    };

    gpio_config(cfg_btn);
    gpio_config(cfg_led);
    gpio_write(GPIO_LED, false);

    plic_Init();
    plic_irq_set_priority(GPIO_INTR, 1);
    plic_irq_set_enabled(GPIO_INTR, kPlicToggleEnabled);
    gpio_assign_irq_handler(GPIO_INTR, gpio_button_isr);

    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    CSR_SET_BITS(CSR_REG_MIE, 1 << 11);

    xButtonQueue = xQueueCreate(4, sizeof(uint8_t));
    xFlashSem = xSemaphoreCreateMutex();

    TaskHandle_t xFlashTaskHandle = NULL;
    TaskHandle_t xCheckTaskHandle = NULL;

    xTaskCreate(vTaskCheckCompare, "Checker", TASK_STACK_SIZE, NULL, CHECK_TASK_PRIO, &xCheckTaskHandle);
    xTaskCreate(vTaskFlashHandler, "FlashRW", TASK_STACK_SIZE, (void *)xCheckTaskHandle, FLASH_TASK_PRIO, &xFlashTaskHandle);
    xTaskCreate(vTaskButtonWait, "Button", TASK_STACK_SIZE, (void *)xFlashTaskHandle, BUTTON_TASK_PRIO, NULL);

    UART_PRINTF("[BOOT] FreeRTOS Flash Test Ready. Press GPIO %d.\n", GPIO_BUTTON);

    vTaskStartScheduler();

    // Should never reach here
    for(;;);
}

int main(void) {
    app_main();
    return 0;
}
