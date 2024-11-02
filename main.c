
#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

void nano_wait(int);
void autotest();

uint16_t xVal = 0;
uint16_t yVal = 0;

void init_dmas(void) {
   RCC -> AHBENR |= RCC_AHBENR_DMA1EN;

   DMA1_Channel1 -> CCR &= ~DMA_CCR_EN; // X
   DMA1_Channel2 -> CCR &= ~DMA_CCR_EN; // Y
   
   DMA1_Channel1 -> CMAR = (uint32_t) &xVal;
   DMA1_Channel1 -> CPAR = (uint32_t) &(ADC1 -> DR);
   DMA1_Channel1 -> CNDTR = 0xf;
   DMA1_Channel1 -> CCR &= ~(DMA_CCR_DIR); // read from peripheral
   DMA1_Channel1 -> CCR |= DMA_CCR_MINC;
   DMA1_Channel1 -> CCR &= ~(DMA_CCR_MSIZE); // 00 - 8b
   DMA1_Channel1 -> CCR &= ~(DMA_CCR_PSIZE); // 00 - 8b
   DMA1_Channel1 -> CCR |= DMA_CCR_MSIZE_0; // 01 - 16b
   DMA1_Channel1 -> CCR |= DMA_CCR_PSIZE_0; // 01 - 16b
   
   DMA1_Channel2 -> CMAR = (uint32_t) &yVal;
   DMA1_Channel2 -> CPAR = (uint32_t) &(ADC1 -> DR); // Look into
   DMA1_Channel2 -> CNDTR = 0xf;
   DMA1_Channel2 -> CCR &= ~(DMA_CCR_DIR);
   DMA1_Channel2 -> CCR |= DMA_CCR_MINC;
   DMA1_Channel2 -> CCR &= ~(DMA_CCR_MSIZE);
   DMA1_Channel2 -> CCR &= ~(DMA_CCR_PSIZE);
   DMA1_Channel2 -> CCR |= DMA_CCR_MSIZE_0;
   DMA1_Channel2 -> CCR |= DMA_CCR_PSIZE_0;

}

void enable_dmas(void) {
   DMA1_Channel1-> CCR |= DMA_CCR_EN;
   DMA1_Channel2-> CCR |= DMA_CCR_EN;
}

void setup_adcs(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER |= (GPIO_MODER_MODER1); //PA1 Analog
    GPIOA -> MODER |= (GPIO_MODER_MODER0); //PA0

    RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable ADC1
    
    RCC -> CR2 |= RCC_CR2_HSI14ON; //Clock?
    while((RCC -> CR2 & RCC_CR2_HSI14RDY) == 0);
    ADC1 -> CR |= ADC_CR_ADEN; //Enable ADC
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
}

void readXY(void) {
    SYSCFG -> CFGR1 &= ~(0b100000000);
    //Use DMA Channel 1 (X)
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR = 0b1;
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
    ADC1 -> CR |= ADC_CR_ADSTART;

   SYSCFG -> CFGR1 |= 0b100000000;
   //Use DMA Channel 2 (Y)
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR = 0b10;
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
}

void init_tim6(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    //Set prescaler to 2 - 1
    TIM6->PSC = 2 - 1;
    //Calculate ARR for 30 Hz interrupt rate
    TIM6->ARR = (24000000 / 30) - 1;
    //Enable update interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 1 << 17;
    //Enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
}

void init_tim6(void) { 
    //Enable RCC clock for TIM6 
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; 
    //Set prescaler to 2 - 1 
    TIM6->PSC = 2 - 1; 
    //Calculate ARR for 30 Hz interrupt rate 
    TIM6->ARR = (24000000 / 30) - 1; 
    //Enable update interrupt 
    TIM6->DIER |= TIM_DIER_UIE; 
    //Unmask the interrupt in the 
    NVIC NVIC->ISER[0] = 1 << 17; 
    //Enable counter 
    TIM6->CR1 |= TIM_CR1_CEN; TIM6->CR2 |= TIM_CR2_MMS_1; 
}
