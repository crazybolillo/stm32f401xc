/**
 * cny70.c
 *
 * Read the analog output of a CNY70 IR proximity sensor
 * and turn on the onboard LED after reaching a predefined
 * threshold voltage.
 */

#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum { BUFFER_SIZE = 64 };

static const float THRESHOLD_VOLTAGE = 0.6F;
static const uint8_t ADC_CHANNEL_1 = 0x01;

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
        vTaskDelay(pdMS_TO_TICKS(250));

        ADC1->CR2 |= ADC_CR2_SWSTART;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_voltage = adc_res * ADC1->DR;
        if (adc_voltage > THRESHOLD_VOLTAGE) {
            GPIOC->ODR &= ~(1 << GPIO_ODR_OD13_Pos);
        } else {
            GPIOC->ODR |= GPIO_ODR_OD13;
        }

        if (!tud_cdc_connected()) { continue; }
        snprintf_(stdio_buffer, BUFFER_SIZE, "Voltage: %.3f\r", adc_voltage);
        tud_cdc_write_str(stdio_buffer);
        tud_cdc_write_flush();
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(readAnalogTask, "adc", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &readAnalogTaskHandle);
    configASSERT(&readAnalogTaskHandle);

    vTaskStartScheduler();

    return 0;
}
