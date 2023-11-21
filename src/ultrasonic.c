/**
 * ultrasonic.c
 *
 * Control an HC-SR04 ultrasonic sensor and calculate distances
 * based on the provided measurements. Output is printed over a
 * USB CDC connection.
 *
 * PA10 - TIM1_CH3 triggers the sensor.
 * PB6 - TIM4_CH1,TIM4_CH2 reads the sensor output (rising and falling edge respectively).
 *
 * Both timers are set to count once each micro second (1 MHz).
 */

#include <FreeRTOS.h>
#include <printf/printf.h>
#include <task.h>
#include <tusb.h>

#include "common.h"

enum {
    BUFFER_SIZE = 128,
    PRESCALER = 71,
    TIM1_PULSE_US_DELAY = 10,
    TIM1_PULSE_US_TIME = 12,
    PWM_MODE_2 = 0x07,
    SLAVE_RESET_MODE = 0x06,
};

TaskHandle_t readSensorTaskHandle;

static char buffer[BUFFER_SIZE];
static const float SENSOR_CONSTANT = 58.0F;
static int start_micros = 0, end_micros = 0;

void setupHardware(void) {
    setup_clock();
    setup_board_led();
    setup_usb(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, tskIDLE_PRIORITY + 1);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= MODER_ALTERNATE << GPIO_MODER_MODER10_Pos;
    GPIOA->AFR[1] |= 0x01 << GPIO_AFRH_AFSEL10_Pos;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_OPM | TIM_CR1_ARPE;
    TIM1->PSC = PRESCALER;
    TIM1->CCMR2 |= (PWM_MODE_2 << TIM_CCMR2_OC3M_Pos) | (TIM_CCMR2_OC3PE);
    TIM1->CCR3 = TIM1_PULSE_US_DELAY;
    TIM1->ARR = TIM1_PULSE_US_DELAY + TIM1_PULSE_US_TIME;
    TIM1->CCER |= TIM_CCER_CC3E;
    TIM1->EGR |= TIM_EGR_UG;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (MODER_ALTERNATE << GPIO_MODER_MODER6_Pos) | (MODER_ALTERNATE << GPIO_MODER_MODER7_Pos);
    GPIOB->AFR[0] |= (0X02 << GPIO_AFRL_AFSEL6_Pos) | (0X02 << GPIO_AFRL_AFSEL7_Pos);

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->CR1 |= TIM_CR1_CEN;
    TIM4->BDTR |= TIM_BDTR_MOE;
    TIM4->PSC |= PRESCALER;
    TIM4->CCMR1 |= (0x01 << TIM_CCMR1_CC1S_Pos) | (0x02 << TIM_CCMR1_CC2S_Pos);
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;
    TIM4->SMCR |= SLAVE_RESET_MODE << TIM_SMCR_SMS_Pos;
    TIM4->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;

    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_CC1IF) {
        TIM4->SR &= ~(0x01 << TIM_SR_CC2IF);
        start_micros = TIM4->CCR1;
    } else if ((TIM4->SR & TIM_SR_CC2IF) && (start_micros != 0)) {
        TIM4->SR &= ~(0x01 << TIM_SR_CC1IF);
        end_micros = TIM4->CCR2;
        vTaskNotifyGiveFromISR(readSensorTaskHandle, NULL);
    }
    TIM4->SR = 0x00;
}

void readSensorTask(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));
        TIM1->CR1 |= TIM_CR1_CEN;
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(portMAX_DELAY));

        int micros = end_micros - start_micros;
        if (micros > 0) {
            float distance_cm = (float)micros / SENSOR_CONSTANT;
            snprintf_(
                buffer, BUFFER_SIZE, "Time to receive: %06d us\r\nDistance: %06.2f cm\r\033[A", micros, distance_cm
            );
            tud_cdc_write_str(buffer);
            tud_cdc_write_flush();
        }

        start_micros = end_micros = 0;
    }
}

int main(void) {
    setupHardware();

    xTaskCreate(readSensorTask, "read", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &readSensorTaskHandle);
    configASSERT(&readSensorTaskHandle);

    vTaskStartScheduler();
}
