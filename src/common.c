#include "common.h"

#include <FreeRTOSConfig.h>
#include <stm32f4xx.h>

static const uint8_t MCO1PRE_DIV4 = 0x06;
static const uint8_t MCO1_PPL = 0x03;
static const uint8_t PPRE1_DIV2 = 0x04;

static const uint8_t AFR_AF10 = 0x0A;

void setup_clock(void) {
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS) {}

    RCC->CFGR |=
        (MCO1PRE_DIV4 << RCC_CFGR_MCO1PRE_Pos) | (MCO1_PPL << RCC_CFGR_MCO1_Pos) | (PPRE1_DIV2 << RCC_CFGR_PPRE1_Pos);

    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {}

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    RCC->PLLCFGR = (6 << RCC_PLLCFGR_PLLQ_Pos) | (288 << RCC_PLLCFGR_PLLN_Pos) | (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (1 << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) == 0) {}

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / configTICK_RATE_HZ);
}

void setup_usb(uint8_t irq_priority) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |=
        (MODER_ALTERNATE << GPIO_MODER_MODE8_Pos | (MODER_ALTERNATE << GPIO_MODER_MODER11_Pos) |
         (MODER_ALTERNATE << GPIO_MODER_MODER12_Pos));
    GPIOA->OSPEEDR |= (0x02 << GPIO_OSPEEDR_OSPEED11_Pos) | (0x02 << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOA->AFR[1] |= (AFR_AF10 << GPIO_AFRH_AFSEL11_Pos) | (AFR_AF10 << GPIO_AFRH_AFSEL12_Pos);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0) {}

    NVIC_SetPriority(OTG_FS_IRQn, irq_priority);
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
}
