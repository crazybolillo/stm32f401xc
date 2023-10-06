#include <FreeRTOS.h>
#include <task.h>

const uint8_t MCO1PRE_DIV4 = 0x06;
const uint8_t MCO1_PPL = 0x03;
const uint8_t PPRE1_DIV2 = 0x04;

static TaskHandle_t blinkTaskHandle;

void setupHardware(void) {
    __disable_irq();
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

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->ODR = 0;
    GPIOC->MODER = 1 << GPIO_MODER_MODER13_Pos;
}

void blinkLedTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        GPIOC->ODR |= 1 << GPIO_ODR_OD13_Pos;
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOC->ODR = 0;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(blinkLedTask, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &blinkTaskHandle);
    configASSERT(&blinkTaskHandle);
    vTaskStartScheduler();

    return 0;
}
