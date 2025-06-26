/*
 * FreeRTOS Lab - Button-triggered SPI Flash Write+Read
 * Includes UART logging and timing benchmarks
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

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
#include "handler.h"

#define GPIO_BUTTON         10
#define GPIO_INTR           GPIO_INTR_10
#define FLASH_LENGTH        1024
#define TASK_STACK_SIZE     512
#define FLASH_TASK_PRIO     (tskIDLE_PRIORITY + 2)
#define BUTTON_TASK_PRIO    (tskIDLE_PRIORITY + 1)
#define CHECK_TASK_PRIO     (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIO (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_DELAY_MS  1000  // 1 second

/* Const value to play with TICK counts within the APP */
#define TICK_COUNT                          ( 50 )

#define GPIO_LD5_R  11
#define GPIO_LD5_B  12
#define GPIO_LD5_G  13

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

static QueueHandle_t xButtonQueue;
static TimerHandle_t xFlashTimer;

/* Timer 0 AO Domain as Tick Counter */
static rv_timer_t timer_0_1;

/* Allocate heap to special section. Note that we have no references in the
 * whole program to this variable (since its just here to allocate space in the
 * section for our heap), so when using LTO it will be removed. We force it to
 * stay with the "used" attribute
 */
__attribute__((section(".heap"), used)) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

// Flash buffers
uint8_t flash_write_data[FLASH_LENGTH];
uint8_t flash_read_data[FLASH_LENGTH];
uint8_t __attribute__((section(".xheep_data_flash_only"))) __attribute__((aligned(16))) flash_write_target[FLASH_LENGTH];

// For logging with UART safely from multiple tasks
#define UART_LOCK()    taskENTER_CRITICAL()
#define UART_UNLOCK()  taskEXIT_CRITICAL()
#define UART_PRINTF(...) do { UART_LOCK(); printf(__VA_ARGS__); UART_UNLOCK(); } while(0)

// Helper to toggle LED for result indication (reuse LD5_R)
static void toggle_led_passfail(bool pass) {
    gpio_write(GPIO_LD5_R, pass);
}

void gpio_button_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t event = 1;
    xQueueSendFromISR(xButtonQueue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vTaskButtonWait(void *pvParams) {
    uint8_t received;
    for (;;) 
    {
        if (xQueueReceive(xButtonQueue, &received, portMAX_DELAY) == pdPASS) 
        {
            UART_PRINTF("[EVENT] Button press received in task.\n");
            xTaskNotifyGive((TaskHandle_t)pvParams); // notify Flash task
        }
    }
}

void vTaskFlashHandler(void *pvParams) {
    spi_host_t *spi = spi_flash;
    soc_ctrl_t soc_ctrl = { .base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS) };
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);

    if (w25q128jw_init(spi) != FLASH_OK) 
    {
        UART_PRINTF("[ERROR ❌] SPI init failed!\n");
        vTaskSuspend(NULL);
    }

    // Fill known data
    for (uint32_t i = 0; i < FLASH_LENGTH; i++) flash_write_data[i] = (uint8_t)(i & 0xFF);

    uintptr_t flash_offset = heep_get_flash_address_offset(flash_write_target);
    UART_PRINTF("[INIT] Flash offset: 0x%08lx\n", flash_offset);

    for (;;) 
    {
        UART_PRINTF("[TASK] Flash task waiting for notification...\n");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for button task
        UART_PRINTF("[TASK] Flash task triggered by button\n");

        memset(flash_read_data, 0, FLASH_LENGTH);
        uint32_t t_start = xTaskGetTickCount();

        w25q_error_codes_t err = w25q128jw_erase_and_write_standard((void *)flash_offset, flash_write_data, FLASH_LENGTH);
        if (err != FLASH_OK) {
            UART_PRINTF("[ERROR ❌] Flash write failed!\n");
            continue;
        }

        err = w25q128jw_read_standard((void *)flash_offset, flash_read_data, FLASH_LENGTH);
        if (err != FLASH_OK) {
            UART_PRINTF("[ERROR ❌] Flash read failed!\n");
            continue;
        }

        uint32_t t_end = xTaskGetTickCount();
        uint32_t t_elapsed = (t_end - t_start);
        UART_PRINTF("[BENCH] Flash R/W took %u ticks.\n", t_elapsed);

        xTaskNotifyGive((TaskHandle_t)pvParams); // Notify checker task
    }
}

void vTaskCheckCompare(void *pvParams) {
    for (;;) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        UART_PRINTF("[TASK] Checker task triggered by flash task\n");
        uint32_t errors = 0;
        for (uint32_t i = 0; i < FLASH_LENGTH; i++) {
            if (flash_write_data[i] != flash_read_data[i]) {
                errors++;
                if (errors < 10) UART_PRINTF("[MISMATCH] @%u: 0x%02X != 0x%02X\n", i, flash_write_data[i], flash_read_data[i]);
            }
        }
        bool pass = (errors == 0);
        UART_PRINTF("[RESULT] %s\n", pass ? "PASS ✅" : "FAIL ❌");
        toggle_led_passfail(pass);
    }
}

void vTaskLEDHeartBeat(void *pvParams) {
    bool state = false;
    for (;;) {
        //gpio_write(GPIO_LD5_R, state);
        //gpio_write(GPIO_LD5_B, state);
        //gpio_write(GPIO_LD5_G, state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_DELAY_MS));
    }
}

static void print_memory_summary(void) {
    UART_PRINTF("\n==== FreeRTOS Memory/Status Summary ====\n");
    UART_PRINTF("[HEAP] Free heap size now: %u bytes\n", xPortGetFreeHeapSize());
    UART_PRINTF("[HEAP] Minimum ever free heap size: %u bytes\n", xPortGetMinimumEverFreeHeapSize());
    UART_PRINTF("[STACK] FlashTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
    UART_PRINTF("[STACK] ButtonTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
    UART_PRINTF("[STACK] CheckerTask watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
    UART_PRINTF("[STACK] Total tasks: %u\n", uxTaskGetNumberOfTasks());
    UART_PRINTF("[STACK] Total queues: %u\n", uxQueueGetNumberOfQueues());
    UART_PRINTF("[STACK] Total timers: %u\n", xTimerGetTimerCount());
    UART_PRINTF("[STACK] Total semaphores: %u\n", uxSemaphoreGetCount(NULL));
    UART_PRINTF("[STACK] Total mutexes: %u\n", uxMutexGetCount(NULL));
    UART_PRINTF("[STACK] Total task notifications: %u\n", uxTaskGetNumberOfNotifications());
    UART_PRINTF("[STACK] Total ISR notifications: %u\n", uxTaskGetNumberOfISRs());
    UART_PRINTF("[STACK] Total event groups: %u\n", uxEventGroupGetNumberOfEventGroups());
    UART_PRINTF("[STACK] Total timers: %u\n", uxTimerGetNumberOfTimers());
    UART_PRINTF("[STACK] Total task states: %u\n", uxTaskGetNumberOfTaskStates());
    UART_PRINTF("[STACK] Total task priorities: %u\n", uxTaskGetNumberOfPriorities());
    UART_PRINTF("[STACK] Total task names: %u\n", uxTaskGetNumberOfTaskNames());
    UART_PRINTF("[STACK] Total task handles: %u\n", uxTaskGetNumberOfTaskHandles());
    UART_PRINTF("=================================\n\n");
}

void app_main(void) 
{
    gpio_cfg_t cfg_btn = {
        .pin = GPIO_BUTTON,
        .mode = GpioModeIn,
        .en_input_sampling = true,
        .en_intr = true,
        .intr_type = GpioIntrEdgeRising
    };
    gpio_config(cfg_btn);

   gpio_result_t gpio_res;
    gpio_cfg_t pin_cfg = {
        .pin= GPIO_LD5_R, 
        .mode= GpioModeOutPushPull
    };
    gpio_res = gpio_config(pin_cfg);
    pin_cfg.pin = GPIO_LD5_B;
    gpio_res |= gpio_config(pin_cfg);
    pin_cfg.pin = GPIO_LD5_G;
	gpio_res |= gpio_config(pin_cfg);
    if (gpio_res != GpioOk) printf("Failed\n;");
    gpio_write(GPIO_LD5_R, false);
    gpio_write(GPIO_LD5_B, false);
    gpio_write(GPIO_LD5_G, false);

    plic_Init();
    plic_irq_set_priority(GPIO_INTR, 1);
    plic_irq_set_enabled(GPIO_INTR, kPlicToggleEnabled);
    gpio_assign_irq_handler(GPIO_INTR, gpio_button_isr);

    // Setup rv_timer_0_1
    mmio_region_t timer_0_1_reg = mmio_region_from_addr(RV_TIMER_AO_START_ADDRESS);
    rv_timer_init(timer_0_1_reg, (rv_timer_config_t){.hart_count = 2, .comparator_count = 1}, &timer_0_1);

    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
    //CSR_SET_BITS(CSR_REG_MIE, 1 << 11);

    // Enable timer interrupt
    uint32_t mask = 1 << 7;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);

    xButtonQueue = xQueueCreate(4, sizeof(uint8_t));
    UART_PRINTF("[MEM] Heap after xQueueCreate: %u bytes\n", xPortGetFreeHeapSize());

    TaskHandle_t xFlashTaskHandle = NULL;
    TaskHandle_t xCheckTaskHandle = NULL;

    xTaskCreate(vTaskCheckCompare, "Checker", TASK_STACK_SIZE, NULL, CHECK_TASK_PRIO, &xCheckTaskHandle);
    UART_PRINTF("[MEM] Heap after Checker task: %u bytes\n", xPortGetFreeHeapSize());

    xTaskCreate(vTaskFlashHandler, "FlashRW", TASK_STACK_SIZE, (void *)xCheckTaskHandle, FLASH_TASK_PRIO, &xFlashTaskHandle);
    UART_PRINTF("[MEM] Heap after FlashRW task: %u bytes\n", xPortGetFreeHeapSize());

    xTaskCreate(vTaskButtonWait, "Button", TASK_STACK_SIZE, (void *)xFlashTaskHandle, BUTTON_TASK_PRIO, NULL);
    UART_PRINTF("[MEM] Heap after Button task: %u bytes\n", xPortGetFreeHeapSize());

    xTaskCreate(vTaskLEDHeartBeat, "LED Heartbeat", TASK_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIO, NULL);
    UART_PRINTF("[MEM] Heap after LED Heartbeat task: %u bytes\n", xPortGetFreeHeapSize());

    UART_PRINTF("[BOOT] FreeRTOS Flash Test Ready. Press GPIO %d.\n", GPIO_BUTTON);

    vTaskStartScheduler();

    // Should never reach here
    for(;;);
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	printf( "[ERROR ❌] Application malloc failed\n\r" );
	__asm volatile( "ebreak" );
	for( ;; );
}

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
	taskENTER_CRITICAL();
	//printf("I\r\n");
	taskEXIT_CRITICAL();
	
}

void freertos_risc_v_application_exception_handler(uint32_t mcause)
{
	printf("[ISR] App mcause:%d\r\n", mcause);
}

void freertos_risc_v_application_interrupt_handler(uint32_t mcause)
{
    int irq_id = plic_irq_claim(&mcause);
    printf("[ISR] Claimed IRQ: %d\r\n", mcause);

    // Dispatch based on IRQ
    if (mcause == GPIO_INTR) {
        gpio_intr_clear_stat(GPIO_INTR);
        gpio_button_isr();
    }
    plic_irq_complete(&mcause);
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	__asm volatile( "ebreak" );
	for( ;; );
}

void vApplicationTickHook( void )
{
	/* Nothing to do here, but this function is required by FreeRTOS. */
    // This function can be used for periodic tasks or debugging.
}

/**
 * Use this function in case you want an ad-hoc MTIME logic.
 */
void handler_irq_timer(void)
{
	configASSERT(rv_timer_reset(&timer_0_1)==kRvTimerOk);
    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_arm(&timer_0_1, 0, 0, TICK_COUNT) == kRvTimerOk);
	
    if (xTaskIncrementTick() != 0) {
		vTaskSwitchContext();
		//intr_flag = 1;
	}
	
	uint32_t out = 0;
	out = xTaskGetTickCountFromISR();
	printf( "I %d\r\n",out);
	
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);
}

int main(void) {
    app_main();
    return 0;
}
