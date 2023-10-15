#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum { BUFFER_SIZE = 64 };

static const uint8_t ADC_CHANNEL_1 = 0x01;

static TaskHandle_t blinkTaskHandle;
static TaskHandle_t readAnalogTaskHandle;

static char stdio_buffer[BUFFER_SIZE];
static volatile float adc_voltage;
static const float adc_res = 0.000805F;

void setupHardware(void) {
    setup_clock();
    setup_board_led();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 2);

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= MODER_ANALOG << GPIO_MODER_MODER1_Pos;

    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SQR3 |= ADC_CHANNEL_1 << ADC_SQR3_SQ1_Pos;

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
}

void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        ADC1->SR &= ~(ADC_SR_EOC);
        vTaskNotifyGiveFromISR(readAnalogTaskHandle, NULL);
    }
}

void readAnalogTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        ADC1->CR2 |= ADC_CR2_SWSTART;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!tud_cdc_connected()) { continue; }

        adc_voltage = adc_res * ADC1->DR;
        snprintf_(stdio_buffer, BUFFER_SIZE, "Voltage: %.3f\r\n", adc_voltage);
        if (tud_cdc_write_available() < strlen(stdio_buffer)) { continue; }
        tud_cdc_write_str(stdio_buffer);
        tud_cdc_write_flush();
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(readAnalogTask, "adc", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &readAnalogTaskHandle);
    configASSERT(&readAnalogTaskHandle);

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &blinkTaskHandle);
    configASSERT(&blinkTaskHandle);

    vTaskStartScheduler();

    return 0;
}
