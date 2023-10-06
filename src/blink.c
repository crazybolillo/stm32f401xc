#include <FreeRTOS.h>
#include <task.h>

#include "common.h"

static TaskHandle_t blinkTaskHandle;

void setupHardware(void) {
    __disable_irq();
    setup_clock();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->ODR = 0;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;
}

void blinkLedTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        GPIOC->ODR |= 1 << GPIO_ODR_OD13_Pos;
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOC->ODR = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &blinkTaskHandle);
    configASSERT(&blinkTaskHandle);
    vTaskStartScheduler();

    return 0;
}
