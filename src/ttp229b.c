/**
 * ttp229b.c
 *
 * Read the pressed keys from a capacitive keyboard using
 * a TTP229B sensor. This variant does not support I2C, but
 * instead a custom serial protocol. It is close enough to SPI,
 * so the hardware module is used.
 *
 * Raw data and the smallest value key being pressed are
 * printed out over USB CDC.
 *
 * PB3: SCL
 * PB4: SDO
 */

#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum { BR_PRESCALE_64 = 0x05, BUFFER_SIZE = 64, DUMMY_WRITE = 0xAAAA, TTP_MASK = 0xFFFF, TTP_HIGHEST_BIT = 15 };

static TaskHandle_t spiReadTaskHandle;
static char buffer[BUFFER_SIZE];

void setupHardware(void) {
    setup_clock();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 2);
    setup_board_led();
    setup_spi1(BR_PRESCALE_64, true, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
}

void SPI1_IRQHandler(void) {
    if (SPI1->SR & SPI_SR_RXNE) {
        int spi_data = SPI1->DR;
        xTaskNotifyFromISR(spiReadTaskHandle, spi_data, eSetValueWithOverwrite, NULL);
    }
}

void spiReadTask(void *pvParameters) {
    SPI1->CR1 |= SPI_CR1_SPE;
    while (1) {
        SPI1->DR = DUMMY_WRITE;

        int data = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
        data ^= TTP_MASK;

        int pressedKey = -1;
        for (int key = TTP_HIGHEST_BIT; key >= 0; key--) {
            int mask = 1 << key;
            if (data & mask) {
                pressedKey = TTP_HIGHEST_BIT - key;
                break;
            }
        }

        snprintf_(buffer, BUFFER_SIZE, "Keyboard: 0x%04x\r\nKey: %02d\r\033[A", data, pressedKey);
        tud_cdc_write_str(buffer);
        tud_cdc_write_flush();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    xTaskCreate(spiReadTask, "spi", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &spiReadTaskHandle);
    configASSERT(&spiReadTaskHandle);

    vTaskStartScheduler();
}
