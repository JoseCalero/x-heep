/*
 * FreeRTOS Flash Lab - Refined Version with CLI and Heap Monitoring
 * Features:
 * - UART debug and timing benchmarks
 * - Heap stats and stack watermark logging
 * - Mutex-based Flash resource protection
 * - LED heartbeat task
 * - Tick timer setup for FreeRTOS
 * - UART CLI task for dynamic interaction
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>
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
#include "uart.h"

#define GPIO_BUTTON         10
#define GPIO_INTR           GPIO_INTR_10
#define GPIO_LED            11
#define FLASH_LENGTH        1024
#define TASK_STACK_SIZE     512
#define FLASH_TASK_PRIO     (tskIDLE_PRIORITY + 2)
#define BUTTON_TASK_PRIO    (tskIDLE_PRIORITY + 1)
#define CHECK_TASK_PRIO     (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIO (tskIDLE_PRIORITY + 1)
#define CLI_TASK_PRIO       (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_DELAY_MS  1000
#define TICK_COUNT          50

__attribute__((section(".heap"), used)) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

static QueueHandle_t xButtonQueue;
static SemaphoreHandle_t xFlashSem;
static TaskHandle_t xFlashTaskHandle = NULL;
static TaskHandle_t xCheckTaskHandle = NULL;
static rv_timer_t timer_0_1;

uint8_t flash_write_data[FLASH_LENGTH];
uint8_t flash_read_data[FLASH_LENGTH];
uint8_t __attribute__((section(".xheep_data_flash_only"))) __attribute__((aligned(16))) flash_write_target[FLASH_LENGTH];

#define UART_LOCK()    taskENTER_CRITICAL()
#define UART_UNLOCK()  taskEXIT_CRITICAL()
#define UART_PRINTF(...) printf(__VA_ARGS__)

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
            xTaskNotifyGive((TaskHandle_t)pvParams);
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

    for (uint32_t i = 0; i < FLASH_LENGTH; i++) flash_write_data[i] = (uint8_t)(i & 0xFF);
    uintptr_t flash_offset = heep_get_flash_address_offset(flash_write_target);
    UART_PRINTF("[INIT] Flash offset: 0x%08lx\n", flash_offset);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        memset(flash_read_data, 0, FLASH_LENGTH);
        uint32_t t_start = xTaskGetTickCount();

        xSemaphoreTake(xFlashSem, portMAX_DELAY);
        w25q_error_codes_t err = w25q128jw_erase_and_write_standard((void *)flash_offset, flash_write_data, FLASH_LENGTH);
        if (err == FLASH_OK)
            err = w25q128jw_read_standard((void *)flash_offset, flash_read_data, FLASH_LENGTH);
        xSemaphoreGive(xFlashSem);

        uint32_t t_elapsed = (xTaskGetTickCount() - t_start) * portTICK_PERIOD_MS;
        UART_PRINTF("[BENCH] Flash R/W took %u ms.\n", t_elapsed);
        UART_PRINTF("[HEAP] Free heap: %u bytes\n", xPortGetFreeHeapSize());
        UART_PRINTF("[STACK] FlashTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));

        if (err != FLASH_OK) {
            UART_PRINTF("[ERROR] Flash R/W failed!\n");
            continue;
        }
        xTaskNotifyGive((TaskHandle_t)pvParams);
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
        UART_PRINTF("[RESULT] %s\n", errors == 0 ? "PASS ✅" : "FAIL ❌");
        UART_PRINTF("[STACK] CheckerTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
        gpio_write(GPIO_LED, errors == 0);
    }
}

void vTaskLEDHeartBeat(void *pvParams) {
    bool state = false;
    for (;;) {
        gpio_write(GPIO_LED, state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_DELAY_MS));
    }
}

void vTaskCLIMonitor(void *pvParams) {
    soc_ctrl_t soc_ctrl;
    soc_ctrl.base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS);

    uart_t uart;
    uart.base_addr   = mmio_region_from_addr((uintptr_t)UART_START_ADDRESS);
    uart.baudrate    = UART_BAUDRATE;
    uart.clk_freq_hz = soc_ctrl_get_frequency(&soc_ctrl);
#ifdef UART_NCO
    uart.nco         = UART_NCO;
#else
    uart.nco         = ((uint64_t)uart.baudrate << (NCO_WIDTH + 4)) / uart.clk_freq_hz;
#endif
    uart_init(&uart);

    char cmd_buf[32];
    while (1) {
        UART_PRINTF("[CLI] Enter command: ");
        int idx = 0;
        while (1) {
            uint8_t c;
            uart_getchar(&uart, &c); // Blocking read from UART
            if (c == '\r' || c == '\n') {
                UART_PRINTF("\r\n");
                cmd_buf[idx] = '\0';
                break;
            } else if (c == '\b' && idx > 0) {
                idx--;
                UART_PRINTF("\b \b");
            } else if (c >= 32 && c < 127 && idx < (int)sizeof(cmd_buf) - 1) {
                cmd_buf[idx++] = (char)c;
                UART_PRINTF("%c", c);
            }
        }
        if (cmd_buf[0] == '\0') {
            continue; // Ignore empty commands
        }

        if (strncmp(cmd_buf, "heap", 4) == 0) {
            UART_PRINTF("[HEAP] Free: %u bytes\n", xPortGetFreeHeapSize());
        } else if (strncmp(cmd_buf, "stack", 5) == 0) {
            UART_PRINTF("[STACK] Flash: %u, Checker: %u\n",
                        uxTaskGetStackHighWaterMark(xFlashTaskHandle),
                        uxTaskGetStackHighWaterMark(xCheckTaskHandle));
        } else if (strncmp(cmd_buf, "tasklist", 8) == 0) {
            char taskList[256];
            vTaskList(taskList);
            UART_PRINTF("[TASKLIST]\n%s\n", taskList);
        } else if (strncmp(cmd_buf, "flash", 5) == 0) {
            UART_PRINTF("[CLI] Manually triggering flash task.\n");
            xTaskNotifyGive(xFlashTaskHandle);
        } else {
            UART_PRINTF("[CLI] Unknown command\n");
        }
    }
}

void handler_irq_timer(void) {
    configASSERT(rv_timer_reset(&timer_0_1) == kRvTimerOk);
    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
    configASSERT(rv_timer_arm(&timer_0_1, 0, 0, TICK_COUNT) == kRvTimerOk);
    if (xTaskIncrementTick() != 0) vTaskSwitchContext();
    configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);
}

void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    UART_PRINTF("[ERROR] Malloc failed!\n");
    for (;;) __asm volatile("ebreak");
}

void vApplicationIdleHook(void) {}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void)pcTaskName; (void)pxTask;
    taskDISABLE_INTERRUPTS();
    UART_PRINTF("[ERROR] Stack overflow!\n");
    for (;;) __asm volatile("ebreak");
}

void vApplicationTickHook(void) {}

void freertos_risc_v_application_exception_handler(uint32_t mcause) {
    UART_PRINTF("[ISR] App mcause: %d\n", mcause);
}

void freertos_risc_v_application_interrupt_handler(uint32_t mcause) {
    int irq_id = plic_irq_claim(&mcause);
    UART_PRINTF("[ISR] Claimed IRQ: %d\n", mcause);
    if (mcause == GPIO_INTR) {
        gpio_intr_clear_stat(GPIO_INTR);
        gpio_button_isr();
    }
    plic_irq_complete(&mcause);
}

void app_main(void) {
    gpio_config((gpio_cfg_t){ .pin = GPIO_BUTTON, .mode = GpioModeIn, .en_input_sampling = true, .en_intr = true, .intr_type = GpioIntrEdgeRising });
    gpio_config((gpio_cfg_t){ .pin = GPIO_LED, .mode = GpioModeOutPushPull });
    gpio_write(GPIO_LED, false);

    plic_Init();
    plic_irq_set_priority(GPIO_INTR, 1);
    plic_irq_set_enabled(GPIO_INTR, kPlicToggleEnabled);
    gpio_assign_irq_handler(GPIO_INTR, gpio_button_isr);

    mmio_region_t timer_0_1_reg = mmio_region_from_addr(RV_TIMER_AO_START_ADDRESS);
    rv_timer_init(timer_0_1_reg, (rv_timer_config_t){.hart_count = 2, .comparator_count = 1}, &timer_0_1);

    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    CSR_SET_BITS(CSR_REG_MIE, (1 << 7));
    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
    configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);

    xButtonQueue = xQueueCreate(4, sizeof(uint8_t));
    xFlashSem = xSemaphoreCreateMutex();

    xTaskCreate(vTaskCheckCompare, "Checker", TASK_STACK_SIZE, NULL, CHECK_TASK_PRIO, &xCheckTaskHandle);
    xTaskCreate(vTaskFlashHandler, "FlashRW", TASK_STACK_SIZE, (void *)xCheckTaskHandle, FLASH_TASK_PRIO, &xFlashTaskHandle);
    xTaskCreate(vTaskButtonWait, "Button", TASK_STACK_SIZE, (void *)xFlashTaskHandle, BUTTON_TASK_PRIO, NULL);
    xTaskCreate(vTaskLEDHeartBeat, "LED Heartbeat", TASK_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIO, NULL);
    xTaskCreate(vTaskCLIMonitor, "CLI", TASK_STACK_SIZE, NULL, CLI_TASK_PRIO, NULL);

    UART_PRINTF("[BOOT] FreeRTOS Flash Test Ready. Press GPIO %d.\n", GPIO_BUTTON);
    vTaskStartScheduler();
    while (1);
}

int main(void) {
    app_main();
    return 0;
}
