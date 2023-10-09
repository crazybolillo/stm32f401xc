#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

#define BUFFER_SIZE 64

static const uint8_t ADC_CHANNEL_1 = 0x01;

static TaskHandle_t tusbTaskHandle;
static TaskHandle_t blinkTaskHandle;
static TaskHandle_t readAnalogTaskHandle;

static char stdio_buffer[BUFFER_SIZE];
static volatile float adc_voltage;
static const float adc_res = 0.000805F;
static float voltage_integer;
static uint8_t voltage_decimal;

void setupHardware(void) {
    setup_clock();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    tusb_init();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->ODR = 0;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= MODER_ANALOG << GPIO_MODER_MODER1_Pos;

    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SQR3 |= ADC_CHANNEL_1 << ADC_SQR3_SQ1_Pos;

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
}

void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        ADC1->SR &= ~(ADC_SR_EOC);
        vTaskNotifyGiveFromISR(readAnalogTaskHandle, NULL);
    }
}

void blinkLedTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        GPIOC->ODR ^= 1 << GPIO_ODR_OD13_Pos;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tusbTask(void *pvParameters) {
    (void)pvParameters;
    while (1) { tud_task(); }
}

void readAnalogTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        ADC1->CR2 |= ADC_CR2_SWSTART;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!tud_cdc_connected()) { continue; }

        adc_voltage = adc_res * ADC1->DR;
        voltage_integer = (uint8_t)adc_voltage;
        voltage_decimal = (uint8_t)((adc_voltage - voltage_integer) * 1000.0F);
        snprintf_(stdio_buffer, BUFFER_SIZE, "Voltage: %d.%03d\r\n", (uint8_t)voltage_integer, voltage_decimal);
        if (tud_cdc_write_available() < strlen(stdio_buffer)) { continue; }
        tud_cdc_write_str(stdio_buffer);
        tud_cdc_write_flush();
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(tusbTask, "tusb", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &tusbTaskHandle);
    configASSERT(&tusbTaskHandle);

    xTaskCreate(readAnalogTask, "adc", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &readAnalogTaskHandle);
    configASSERT(&readAnalogTaskHandle);

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &blinkTaskHandle);
    configASSERT(&blinkTaskHandle);

    vTaskStartScheduler();

    return 0;
}
