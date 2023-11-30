/**
 * max6675.c
 *
 * Use the in-built SPI hardware module to read the temperature from a MAX6675 chip.
 * Print out the readings over USB CDC.
 * PB3: SCK
 * PB4: MISO
 * PA15: CS
 */

#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum { AFR_SPI = 0x05, BR_PRESCALE_64 = 0x05, FORMAT_16 = 0x01, BUFFER_SIZE = 64, DUMMY_WRITE = 0xAAAA };

static char buffer[BUFFER_SIZE];

static TaskHandle_t blinkLedTaskHandle;
static TaskHandle_t spiReadTaskHandle;

void setupHardware(void) {
    setup_clock();
    setup_board_led();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 2);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOB->MODER |= (MODER_ALTERNATE << GPIO_MODER_MODER3_Pos) | (MODER_ALTERNATE << GPIO_MODER_MODER4_Pos);
    GPIOB->AFR[0] |= (AFR_SPI << GPIO_AFRL_AFSEL3_Pos) | (AFR_SPI << GPIO_AFRL_AFSEL4_Pos);

    SPI1->CR1 |= (BR_PRESCALE_64 << SPI_CR1_BR_Pos) | (FORMAT_16 << SPI_CR1_DFF_Pos) | SPI_CR1_MSTR | SPI_CR1_SSM;
    SPI1->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_SSOE;

    GPIOA->BSRR |= GPIO_BSRR_BS15;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE15) | MODER_OUTPUT << GPIO_MODER_MODE15_Pos;

    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
}

inline void spi1_cs_enable(void) { GPIOA->BSRR |= GPIO_BSRR_BR15; }
inline void sp1_cs_disable(void) { GPIOA->BSRR |= GPIO_BSRR_BS15; }

void SPI1_IRQHandler(void) {
    if (SPI1->SR & SPI_SR_RXNE) {
        sp1_cs_disable();
        int spi_data = SPI1->DR;
        xTaskNotifyFromISR(spiReadTaskHandle, spi_data, eSetValueWithOverwrite, NULL);
    }
}

void spiReadTask(void *pvParameters) {
    SPI1->CR1 |= SPI_CR1_SPE;
    while (1) {
        spi1_cs_enable();
        SPI1->DR = DUMMY_WRITE;
        int data = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
        float temp = (float)(data >> 3) / 4;

        snprintf_(buffer, BUFFER_SIZE, "RX: %#06x\r\nTemp: %07.2f C°\r\033[A", data, temp);
        tud_cdc_write_str(buffer);
        tud_cdc_write_flush();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &blinkLedTaskHandle);
    configASSERT(&blinkLedTaskHandle);

    xTaskCreate(spiReadTask, "spi", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &spiReadTaskHandle);
    configASSERT(&spiReadTaskHandle);

    vTaskStartScheduler();
}
