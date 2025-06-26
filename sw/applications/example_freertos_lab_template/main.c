/*
 * FreeRTOS Lab - Template APP 
 * Includes just a Heartbeat LED task
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

// HEARTBEAT PRIORITY and DELAY
#define TASK_STACK_SIZE     512 
#define HEARTBEAT_TASK_PRIO (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_DELAY_MS  1000  // 1 second


#define GPIO_LD5_R  11
#define GPIO_LD5_B  12
#define GPIO_LD5_G  13

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Timer 0 AO Domain as Tick Counter */
static rv_timer_t timer_0_1;

/* Allocate heap to special section. Note that we have no references in the
 * whole program to this variable (since its just here to allocate space in the
 * section for our heap), so when using LTO it will be removed. We force it to
 * stay with the "used" attribute
 */
__attribute__((section(".heap"), used)) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

// For logging with UART safely from multiple tasks
#define TASK_LOCK()    taskENTER_CRITICAL()
#define TASK_UNLOCK()  taskEXIT_CRITICAL()
#define UART_PRINTF(...) printf(__VA_ARGS__)

// Helper to toggle LED for result indication (reuse LD5_R)
static void toggle_led_passfail(bool pass) 
{
    gpio_write(GPIO_LD5_R, pass);
}

void vTaskLEDHeartBeat(void *pvParams) 
{
    uint8_t led_state = 0;
    for (;;) {
        // Turn off all LEDs first
        gpio_write(GPIO_LD5_R, false);
        gpio_write(GPIO_LD5_B, false);
        gpio_write(GPIO_LD5_G, false);

        // Turn on one LED based on led_state
        switch (led_state % 3) {
            case 0:
                gpio_write(GPIO_LD5_R, true);
                break;
            case 1:
                gpio_write(GPIO_LD5_B, true);
                break;
            case 2:
                gpio_write(GPIO_LD5_G, true);
                break;
        }

        led_state++;
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_DELAY_MS));
    }
}

static void print_memory_summary(void) {
    UART_PRINTF("\n==== FreeRTOS Memory/Status Summary ====\n");
    UART_PRINTF("[HEAP] Free heap size now: %u bytes\n", xPortGetFreeHeapSize());
    UART_PRINTF("[HEAP] Minimum ever free heap size: %u bytes\n", xPortGetMinimumEverFreeHeapSize());
    UART_PRINTF("[STACK] HeartBeat watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
    UART_PRINTF("[STACK] Total tasks: %u\n", uxTaskGetNumberOfTasks());
    UART_PRINTF("=================================\n\n");
}

void app_main(void) 
{

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

    xTaskCreate(vTaskLEDHeartBeat, "LED Heartbeat", TASK_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIO, NULL);
    UART_PRINTF("[MEM] Heap after LED Heartbeat task: %u bytes\n", xPortGetFreeHeapSize());

    UART_PRINTF("[BOOT] Test Ready.\n");

    print_memory_summary();

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

    // Dispatch based on IRQ...
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

int main(void) {
    app_main();
    return 0;
}
