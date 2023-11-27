/**
 * gpio.c
 *
 * Use PA0 as an input, read from it and output its inverted value on PC13.
 */

#include <stm32f4xx.h>

#include "common.h"

void OTG_FS_IRQHandler(void) {}

int main(void) {
    __disable_irq();  // TODO: Find out which interrupt is triggered on A0
    setup_clock();
    setup_board_led();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    while (1) {
        volatile int input = GPIOA->IDR & GPIO_IDR_ID0;
        if (!input) {
            GPIOC->ODR |= GPIO_ODR_OD13;
        } else {
            GPIOC->ODR &= ~GPIO_ODR_OD13;
        }
    }
}
