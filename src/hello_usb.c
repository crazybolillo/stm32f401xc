#include <FreeRTOS.h>
#include <tusb.h>

#include "common.h"

static TaskHandle_t tusbTaskHandle;
static TaskHandle_t printTaskHandle;

static const char message[] = "Hello World\r\n";

void setupHardware(void) {
    __disable_irq();
    setup_clock();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    tusb_init();
}

void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

void tusbTask(void *pvParameters) {
    (void)pvParameters;
    while (1) { tud_task(); }
}

void printTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        if (!tud_cdc_connected()) { continue; }
        if (tud_cdc_write_available() < strlen(message)) { continue; }

        tud_cdc_write_str(message);
        tud_cdc_write_flush();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(tusbTask, "tusb", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &tusbTaskHandle);
    configASSERT(&tusbTaskHandle);

    xTaskCreate(printTask, "print", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &printTaskHandle);
    configASSERT(&printTaskHandle);

    vTaskStartScheduler();
}
