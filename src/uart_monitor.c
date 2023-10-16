/**
 * uart_monitor.c
 * Read UART data and print it over a USB CDC connection.
 *
 * UART is configured for 9600 bps, 1 start bit, 8 data bits and 1 stop bit.
 * The serial line must be connected to PA3.
 */

#include <FreeRTOS.h>
#include <tusb.h>

#include "common.h"

static const uint8_t UART_MANTISSA = 234;
static const uint8_t UART_DIV = 6;
static const uint8_t UART_RX_AF = 7;

static volatile int uart_byte;

static TaskHandle_t processTaskHandle;
static TaskHandle_t blinkLedTaskHandle;

void setupHardware(void) {
    __disable_irq();
    setup_clock();
    setup_board_led();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, tskIDLE_PRIORITY + 1);

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= MODER_ALTERNATE << GPIO_MODER_MODER3_Pos;
    GPIOA->AFR[0] |= UART_RX_AF << GPIO_AFRL_AFSEL3_Pos;

    USART2->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE;
    USART2->BRR = (UART_MANTISSA << USART_BRR_DIV_Mantissa_Pos) | (UART_DIV);

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}

void USART2_IRQHandler(void) {
    USART2->SR;
    uart_byte = USART2->DR;
    vTaskNotifyGiveFromISR(processTaskHandle, NULL);
}

void processTask(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (tud_cdc_connected() == false) { continue; }
        tud_cdc_write_char(uart_byte);
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &blinkLedTaskHandle);
    configASSERT(&blinkLedTaskHandle);

    xTaskCreate(processTask, "process", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &processTaskHandle);
    configASSERT(&processTaskHandle);

    vTaskStartScheduler();

    return 0;
}
