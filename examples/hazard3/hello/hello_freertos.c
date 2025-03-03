/**
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "uart_drv.h"
#include "printf.h"
#include "hazard3_csr.h"

#include "FreeRTOS.h"
#include "task.h"

// Priorities of our threads - higher numbers are higher priority
#define HELLO_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
#define HELLO_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE


void blink_task(void *params)
{
    printf("blink_task starts\n");

    while (1) {
        printf("blink\n");
        vTaskDelay(90);
    }
}

void hello_task(void *params)
{
    printf("hello_task starts\n");

    while (1) {
        printf("hello\n");
        vTaskDelay(100);
    }
}

void vLaunch()
{
    TaskHandle_t task;

    xTaskCreate(hello_task, "HelloThread", HELLO_TASK_STACK_SIZE, NULL, HELLO_TASK_PRIORITY, NULL);
    xTaskCreate(blink_task, "BlinkThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);

    /* Start the tasks running. */
    vTaskStartScheduler();
}

int main( void )
{
	uart_init(115200);

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;

    rtos_name = "FreeRTOS";

    printf("Starting %s\n", rtos_name);

    vLaunch();

    printf("Should not go here!!!\n");

    return 0;
}

void vApplicationIdleHook()
{

}

void vApplicationTickHook()
{

}

void printf_exception_info()
{
    printf("mcause=0x%x\n", read_csr(mcause));
    printf("mepc=0x%x\n", read_csr(mepc));
    printf("mstatus=0x%x\n", read_csr(mstatus));
}

void freertos_risc_v_application_exception_handler()
{
    printf("%s\n", __func__);

    printf_exception_info();

    while (1);
}

void freertos_risc_v_application_interrupt_handler()
{
    printf("%s\n", __func__);

    printf_exception_info();

    while (1);
}
