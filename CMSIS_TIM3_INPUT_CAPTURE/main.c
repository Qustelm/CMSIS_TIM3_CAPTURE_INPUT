#include "stm32f10x.h"
#include <math.h>
#include <stdint.h>

void USART1_INIT() {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // configure PA9 as TX
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

    USART1->BRR = SystemCoreClock/9600;
    USART1->CR1 |= USART_CR1_UE;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;
}

void USART_SEND(char symb) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = symb;
}

void USART_SEND_NUMBER(int number) {
    if (number == 0) {
        USART_SEND('0');
    }
    char arr[12];
    int step = 0;
    while (number > 0) {
        arr[step] = number%10;
        number /= 10;
        step++;
    }
    for (int i = step-1; i >= 0; i--) {
        USART_SEND(arr[i]+48);
    }
    USART_SEND('\n');
    USART_SEND('\r');
}

void TIM3_INIT() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // configure PA6 (TIM3 CH1) as input
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
    GPIOA->CRL |= GPIO_CRL_CNF6_1;

    TIM3->PSC = 72-1;
    TIM3->ARR = 0xFFFF;

    // configure first channel and map to TI1
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S);
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;

    // trigger on rising edge
    TIM3->CCER &= ~TIM_CCER_CC1P;      
    TIM3->CCER |= TIM_CCER_CC1E;

    // enable interrupts
    TIM3->DIER |= TIM_DIER_CC1IE;
    // start timer
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM2_INIT() {
    // configure PWM on PA0
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->CRL |= GPIO_CRL_MODE; 

    TIM2->PSC = 72 - 1; 
    TIM2->ARR = 10000 - 1;    
    TIM2->CCR1 = 5000-1;

    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2;

    TIM2->CCER |= TIM_CCER_CC1E;
    
    TIM2->CR1 |= TIM_CR1_CEN;
}

int32_t capture;
void TIM3_IRQHandler() {
    if ((TIM3->SR & TIM_SR_CC1IF)) {
        if ((TIM3->CCER & TIM_CCER_CC1P) == 0) {
            capture = TIM3->CCR1;
            TIM3->CCER |= TIM_CCER_CC1P;
        } else {
            if (TIM3->CCR1 < capture) {
                capture = 0xFFFF-capture+TIM3->CCR1;
            } else {
                capture = TIM3->CCR1 - capture;
            }
            USART_SEND_NUMBER(capture/490);
            TIM3->CCER &= ~TIM_CCER_CC1P;
        }
    }
    TIM3->SR &= ~TIM_SR_CC1IF;
}

int main() {
    USART1_INIT();
    TIM3_INIT();
    TIM2_INIT();

    while (1) {

    }
}


//PA0 - PWM
//PA6 - Input Capture