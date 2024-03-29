/**
 * led1642.c
 *
 * Communicate with a ST LED1642GW LED controller and decrease/increase LED brightness repeteadly.
 *
 * This chip uses a custom (?) serial protocol to configure several output characteristics.
 * Since I could not map this protocol to a hardware module, the program bit bangs the output using
 * a timer as clock reference. The PWCLK signal required by this chip is generated by another timer as well.
 *
 * PA1: CLK
 * PA0: LE
 * PB5: TIM3.CH2 -> PWCLK @ ~514 KHz
 * PB7: SDI
 */

#include <FreeRTOS.h>
#include <task.h>

#include "common.h"

enum {
    PWM_MODE_1 = 0x06,
    PRESCALE_4 = 3,
    ARR_500_KHZ = 34,
    ARR_100_KHZ = 90,
    BRIGHTNESS_LATCH = 4,
    BRIGHTNESS_GLOBAL_LATCH = 6,
    WRITE_SWITCH_LATCH = 2,
    WRITE_CR_LATCH = 7,
    ALL_ONES = 0xFFFF,
    TWO_BYTES = 15,
    BRIGHTNESS_REG_COUNT = 16,
    BRIGHTNESS_DELTA = 32,
    MAX_BRIGHTNESS = 4095,
};

struct Message {
    uint32_t data;
    int size;
    int latch;
};

static bool rising = true;
static struct Message message = {0};

static TaskHandle_t led1642TaskHandle;

void inline clock_on(void) { GPIOA->BSRR |= GPIO_BSRR_BS1; }
void inline clock_off(void) { GPIOA->BSRR |= GPIO_BSRR_BR1; }

void inline sdo_on(void) { GPIOB->BSRR |= GPIO_BSRR_BS7; }
void inline sdo_off(void) { GPIOB->BSRR |= GPIO_BSRR_BR7; }

void inline le_on(void) { GPIOA->BSRR |= GPIO_BSRR_BS0; }
void inline le_off(void) { GPIOA->BSRR |= GPIO_BSRR_BR0; }

void setupHardware(void) {
    setup_clock();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, tskIDLE_PRIORITY + 2);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (MODER_OUTPUT << GPIO_MODER_MODER7_Pos) | (MODER_ALTERNATE << GPIO_MODER_MODE5_Pos);
    GPIOB->AFR[0] |= 0x02 << GPIO_AFRL_AFSEL5_Pos;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN;

    TIM3->PSC = PRESCALE_4;
    TIM3->CCMR1 |= PWM_MODE_1 << TIM_CCMR1_OC2M_Pos;
    TIM3->ARR = ARR_500_KHZ;
    TIM3->CCR2 = ARR_500_KHZ / 2;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM3->CCER |= TIM_CCER_CC2E;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (MODER_OUTPUT << GPIO_MODER_MODE0_Pos) | (MODER_OUTPUT << GPIO_MODER_MODE1_Pos);

    TIM2->PSC = PRESCALE_4;
    TIM2->ARR = ARR_100_KHZ;
    TIM2->DIER |= TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    sdo_off();
    le_off();
    clock_off();
}

void led1642_transmit(void) {
    if ((message.data >> message.size) & 0x01) { sdo_on(); }
    if (message.latch > message.size) { le_on(); }
    message.size--;
}

void led1642_set_brightness(const uint32_t brightness) {
    message.data = brightness;
    for (int idx = 0; idx < BRIGHTNESS_REG_COUNT - 1; idx++) {
        message.size = TWO_BYTES;
        message.latch = BRIGHTNESS_LATCH;
        TIM2->CR1 |= TIM_CR1_CEN;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    message.size = TWO_BYTES;
    message.latch = BRIGHTNESS_GLOBAL_LATCH;
    TIM2->CR1 |= TIM_CR1_CEN;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void led1642Task(void *pvParameters) {
    // Set CFG-15=1, enables 12 bit PWM counter
    message.data = 1 << TWO_BYTES;
    message.size = TWO_BYTES;
    message.latch = WRITE_CR_LATCH;
    TIM2->CR1 |= TIM_CR1_CEN;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    led1642_set_brightness(0);

    // Turn all outputs ON
    message.data = ALL_ONES;
    message.size = TWO_BYTES;
    message.latch = WRITE_SWITCH_LATCH;
    TIM2->CR1 |= TIM_CR1_CEN;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t brightness = BRIGHTNESS_DELTA;
    while (1) {
        for (; brightness <= MAX_BRIGHTNESS; brightness += BRIGHTNESS_DELTA) { led1642_set_brightness(brightness); }
        vTaskDelay(pdMS_TO_TICKS(100));

        brightness -= BRIGHTNESS_DELTA;  // Otherwise if you set the same brightness twice, LEDs blink
        for (; brightness > 0 && brightness <= MAX_BRIGHTNESS; brightness -= BRIGHTNESS_DELTA) {
            led1642_set_brightness(brightness);
        }
        led1642_set_brightness(0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_DIER_UIE) {
        if (message.size >= 0) {
            if (rising) {
                led1642_transmit();
                clock_on();
            } else {
                sdo_off();
                clock_off();
            }
            rising = !rising;
        } else {
            sdo_off();
            le_off();
            clock_off();
            TIM2->CR1 &= ~(TIM_CR1_CEN);
            vTaskNotifyGiveFromISR(led1642TaskHandle, NULL);
        }
    }
    TIM2->SR = 0;
}

int main(void) {
    setupHardware();

    xTaskCreate(led1642Task, "led1642", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &led1642TaskHandle);
    configASSERT(&led1642TaskHandle);

    vTaskStartScheduler();
}
