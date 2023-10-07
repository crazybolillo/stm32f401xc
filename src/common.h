#ifndef STM32F4_COMMON_H
#define STM32F4_COMMON_H

#include <stdint.h>

enum GPIO_MODER { MODER_INPUT = 0x00, MODER_OUTPUT = 0x01, MODER_ALTERNATE = 0x02, MODER_ANALOG = 0x03 };

/**
 * Setup the clock tree for USB operation and increase CPU frequency. The internal PLL is used to achieve this.
 * CPU clock set to 72 MHz
 * USB clock set to 48 MHz
 * USB clock divided by 4 (12 MHz) output on MCO1.
 */
void setup_clock(void);

/**
 * Initialize the USB peripheral.
 * @param irq_priority Priority that will be assigned to the OTG_FS interruption.
 */
void setup_usb(uint8_t irq_priority);

#endif  // STM32F4_COMMON_H
