/**
 * usb_echo.c
 *
 * Basic program that offers a shell like interface and echoes back
 * the written characters after a newline (enter key) is sent.
 *
 * Before the host opens a serial monitor, the onboard LED rapidly blinks.
 * Once a serial connection is established, the LED will turn off and toggle
 * upon receiving new characters (key presses).
 *
 * Tested with Minicom.
 */

#include <FreeRTOS.h>
#include <tusb.h>

#include "common.h"

int readChar = -1;

static TaskHandle_t readTaskHandle;
static TaskHandle_t printTaskHandle;

void setupHardware(void) {
    __disable_irq();
    setup_clock();
    setup_board_led();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 2);
}

void printTask(void *pvParameters) {
    while (!tud_cdc_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        led_toggle();
    }
    led_off();
    vTaskDelay(pdMS_TO_TICKS(50));

    while (1) {
        tud_cdc_write_str("% ");
        tud_cdc_write_flush();

        xTaskNotifyGive(readTaskHandle);
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(portMAX_DELAY));

        readChar = -1;
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
}

void readTask(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(portMAX_DELAY));
        do {
            if (tud_cdc_available()) {
                readChar = tud_cdc_read_char();
                if (readChar > -1) {
                    led_toggle();
                    if (readChar != '\r') {
                        tud_cdc_write_char(readChar);
                        tud_cdc_write_flush();
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        } while (readChar != '\r');
        xTaskNotifyGive(printTaskHandle);
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(readTask, "read", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &readTaskHandle);
    configASSERT(&readTaskHandle);

    xTaskCreate(printTask, "print", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &printTaskHandle);
    configASSERT(&printTaskHandle);

    vTaskStartScheduler();

    return 0;
}
