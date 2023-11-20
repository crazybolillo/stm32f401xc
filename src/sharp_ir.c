/**
 * sharp_ir.c
 *
 * Read the analog output of a SHARP GP2Y0A02YK0F IR sensor
 * and calculate the distance measured. Analog voltage and
 * distance are printed over the serial monitor.
 *
 * Polynomial interpolation is used to calculate the distance
 * based on a given voltage. The coefficients were obtained
 * with Matlab's polyfit. To interpolate, the voltage readings
 * from 15 to 150cm in the datasheet were used.
 */

#include <FreeRTOS.h>
#include <math.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum { BUFFER_SIZE = 128 };

static const uint8_t ADC_CHANNEL_1 = 0x01;

static TaskHandle_t readAnalogTaskHandle;

static char stdio_buffer[BUFFER_SIZE];
static volatile float adc_voltage;
static const float adc_res = 0.000805F;

static float distance;
static const float cf1 = 14.6703F, cf2 = -119.5783F, cf3 = 358.9549F, cf4 = -490.6314F, cf5 = 297.9901F;

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

float interpolate_distance(float voltage) {
    return (cf1 * powf(voltage, 4)) + (cf2 * powf(voltage, 3)) + (cf3 * powf(voltage, 2)) + (cf4 * voltage) + cf5;
}

void readAnalogTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));

        ADC1->CR2 |= ADC_CR2_SWSTART;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!tud_cdc_connected()) { continue; }

        adc_voltage = adc_res * ADC1->DR;
        distance = interpolate_distance(adc_voltage);

        snprintf_(stdio_buffer, BUFFER_SIZE, "Voltage: %.3f V\r\nDistance: %06.2f cm\r\033[A", adc_voltage, distance);
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
