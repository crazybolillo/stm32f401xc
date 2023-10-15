#include <FreeRTOS.h>
#include <task.h>

#include "common.h"

// Redefine these functions in common.h to do nothing
void OTG_FS_IRQHandler(void) {}
void tusbTask(void *pvParameters) {}

static TaskHandle_t blinkTaskHandle;

void setupHardware(void) {
    __disable_irq();
    setup_clock();
    setup_board_led();
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &blinkTaskHandle);
    configASSERT(&blinkTaskHandle);
    vTaskStartScheduler();

    return 0;
}
