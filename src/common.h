#ifndef STM32F4_COMMON_H
#define STM32F4_COMMON_H

#include <stdbool.h>
#include <stdint.h>

enum GPIO_MODER { MODER_INPUT = 0x00, MODER_OUTPUT = 0x01, MODER_ALTERNATE = 0x02, MODER_ANALOG = 0x03 };

/**
 * Setup the clock tree for USB operation and increase CPU frequency. The internal PLL is used to achieve this.
 * CPU clock set to 72 MHz
 * USB clock set to 48 MHz
 * CPU clock divided by 4 (18 MHz) output on MCO1.
 */
void setup_clock(void);

/**
 * Initialize USB. This includes hardware and software setup. The peripheral is configured and TinyUSB bootstrapped
 * to work with FreeRTOS. Calling this method adds a 'tusb' task to the scheduler.
 * @param irq_priority Priority that will be assigned to the OTG_FS interruption.
 */
void setup_usb(uint8_t irq_priority, uint8_t rtos_priority);

/**
 * Setup the GPIO hardware to control the board's LED found on C13.
 */
void setup_board_led(void);

/**
 * Initialize the SPI1 peripheral and SCK, MISO and CS pins. Interrupts on RXNE are enabled as well.
 * PB3: SCK
 * PB4: MISO
 * PA15: CS
 * @param prescaler Prescaler for the SPI1 clock.
 * @param format16 If true, 16 bit format will be used.
 * @param irq_priority Priority given to the SPI1 IRQ
 */
void setup_spi1(uint8_t prescaler, bool format16, uint8_t irq_priority);

/**
 * Default implementation for FreeRTOS TinyUSB task. Calls tud_task().
 * @param pvParameters
 */
void tusbTask(void *pvParameters);

/**
 * Default implementation for FreeRTOS blinking led task. It toggles C13 every second.
 * @param pvParameters
 */
void blinkLedTask(void *pvParameters);

#endif  // STM32F4_COMMON_H
