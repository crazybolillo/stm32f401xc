#include "common.h"

#include <FreeRTOSConfig.h>
#include <stm32f4xx.h>
#include <tusb.h>

static const uint8_t MCO1PRE_DIV4 = 0x06;
static const uint8_t MCO1_PPL = 0x03;
static const uint8_t PPRE1_DIV2 = 0x04;

static const uint8_t AFR_AF10 = 0x0A;

static TaskHandle_t tusbTaskHandle;

__attribute__((weak)) void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

__attribute__((weak)) void tusbTask(void *pvParameters) {
    (void)pvParameters;
    while (1) { tud_task(); }
}

__attribute__((weak)) void blinkLedTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        GPIOC->ODR ^= 1 << GPIO_ODR_OD13_Pos;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void setup_usb(uint8_t irq_priority, uint8_t rtos_priority) {
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

    tusb_init();
    xTaskCreate(tusbTask, "tusb", configMINIMAL_STACK_SIZE, NULL, rtos_priority, &tusbTaskHandle);
    configASSERT(&tusbTaskHandle);
}

void setup_board_led(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->ODR |= GPIO_ODR_OD13;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;
}
