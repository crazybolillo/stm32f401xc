#include <FreeRTOS.h>
#include <tusb.h>

#include "common.h"

static TaskHandle_t printTaskHandle;

static const char message[] = "Hello World\r\n";

void setupHardware(void) {
    __disable_irq();
    setup_clock();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 2);
}

void processTask(void *pvParameters) {
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

    xTaskCreate(processTask, "print", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &printTaskHandle);
    configASSERT(&printTaskHandle);

    vTaskStartScheduler();
}
