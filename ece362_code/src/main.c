/**
  ******************************************************************************
  * @file    main.c
  * @author  Ashwin Thampi, Dylan Manning, Jack Porter, Nicole Lueck
  * @date    Nov 1, 2024
  * @brief   ECE 362 Mini-Project "Spin-Masters" main file
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "lcd.h"
#include "spi.h"

void nano_wait(int);
void internal_clock();

uint16_t xyVals[2] = {0, 0}; //analog xvalue [0] and yvalue [1]
//uint16_t xVal = 0; //analog xvalue
//uint16_t yVal = 0; //analog yvalue

uint32_t xCurr = 0; //x coordinate
uint32_t yCurr = 0; //y coordinate

uint16_t x_0 = 0; //zero x-acceleration
uint16_t y_0 = 0; //zero y-acceleration
uint16_t z_0 = 0; //zero z-acceleration

uint16_t screen = 1; //current screen

uint16_t xhist[30]; //x value history
uint16_t yhist[30]; //y value history
uint16_t zhist[30]; //z value history

//some type of grid to represent the pixels

static uint8_t pgrid[40][240]; //(represent with a 0/1)

int led_curr = 0;


void init_dmas(void) {
   RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
   //DMA1 -> CSELR |= DMA1_CSELR_CH1_ADC;// Channel Select ADC on 1
   //DMA1 -> CSELR |= DMA1_CSELR_CH2_ADC;// Channel Select ADC on 2

   DMA1_Channel1 -> CCR &= ~DMA_CCR_EN; // X
   DMA1_Channel2 -> CCR &= ~DMA_CCR_EN; // Y
   
   DMA1_Channel1 -> CMAR = (uint32_t) xyVals;
   DMA1_Channel1 -> CPAR = (uint32_t) &(ADC1 -> DR);
   DMA1_Channel1 -> CNDTR = 0x2;
   DMA1_Channel1 -> CCR &= ~(DMA_CCR_DIR); // read from peripheral
   DMA1_Channel1 -> CCR |= DMA_CCR_MINC;
   DMA1_Channel1 -> CCR |= DMA_CCR_CIRC; // may not need circ?
   DMA1_Channel1 -> CCR |= DMA_CCR_TEIE;
   //DMA1_Channel1 -> CCR &= ~(DMA_CCR_MSIZE); // 00 - 8b
   //DMA1_Channel1 -> CCR &= ~(DMA_CCR_PSIZE); // 00 - 8b
   DMA1_Channel1 -> CCR |= DMA_CCR_MSIZE_0; // 01 - 16b
   DMA1_Channel1 -> CCR |= DMA_CCR_PSIZE_0; // 01 - 16b

   /* Do we still need DMA for pgrid?
   DMA1_Channel3 -> CMAR = (uint32_t) &pgrid;
   DMA1_Channel3 -> CPAR = (uint32_t) &(SPI1 -> DR); // Look into
   DMA1_Channel3 -> CNDTR = 0x4b00;
   DMA1_Channel3 -> CCR |= DMA_CCR_DIR;
   DMA1_Channel3 -> CCR |= DMA_CCR_MINC;
   DMA1_Channel3 -> CCR |= DMA_CCR_PINC;
   DMA1_Channel3 -> CCR &= ~(DMA_CCR_MSIZE);
   DMA1_Channel3 -> CCR &= ~(DMA_CCR_PSIZE);
   DMA1_Channel3 -> CCR |= DMA_CCR_MSIZE_0;
   DMA1_Channel3 -> CCR |= DMA_CCR_PSIZE_0;
   */

}

void enable_dmas(void) {
    DMA1_Channel1-> CCR |= DMA_CCR_EN;
    DMA1_Channel2-> CCR |= DMA_CCR_EN;
    //DMA1_Channel2-> CCR |= DMA_CCR_EN;
}

void setup_adcs(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER |= (GPIO_MODER_MODER1); //PA1 Analog
    GPIOA -> MODER |= (GPIO_MODER_MODER0); //PA0

    RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable ADC1
    ADC1 -> CFGR1 |= ADC_CFGR1_DMAEN;
    ADC1 -> CFGR1 |= ADC_CFGR1_DMACFG;
    
    RCC -> CR2 |= RCC_CR2_HSI14ON; //Clock?
    while((RCC -> CR2 & RCC_CR2_HSI14RDY) == 0);
    ADC1 -> CR |= ADC_CR_ADEN; //Enable ADC
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
}

void readXY(void) {
    //SYSCFG -> CFGR1 &= ~(0b100000000);
    //Use DMA Channel 1 (X)
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR = 0b1;
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
    ADC1 -> CR |= ADC_CR_ADSTART;
    //while((ADC1 -> ISR & ADC_ISR_EOC) == 0);
    //xVal = ADC1->DR; //replace with triggering a DMA transfer
    //DMA1 -> IFCR |= 0xff;

    //SYSCFG -> CFGR1 |= 0b100000000;
    //Use DMA Channel 2 (Y)
    ADC1 -> CHSELR = 0;
    ADC1 -> CHSELR = 0b10;
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0);
    ADC1 -> CR |= ADC_CR_ADSTART;
    //while((ADC1 -> ISR & ADC_ISR_EOC) == 0);
    //yVal = ADC1->DR; //replace with triggering a DMA transfer
    //DMA1 -> IFCR |= 0xff;
}

//used as an interrupt to refresh the LCD display & read the acceleration at 20Hz
void init_tim6(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    //Set prescaler to 10,000
    TIM6->PSC = 10-1;
    //Calculate ARR for 20 Hz interrupt rate
    TIM6->ARR = 4800-1;
    //Enable update interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 1 << 17;
    //Set TIM6 a lower priority than TIM7
    NVIC_SetPriority(TIM6_DAC_IRQn, 2); //Priority 2
    //Enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    //TIM6->CR2 |= TIM_CR2_MMS_1; do not need to do this since this is for timer synchronization
}

//used to triger an interrupt to save the display every half second
void init_tim7(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    //Set prescaler to 10,000 - 1
    TIM7->PSC = 100 - 1;
    //Calculate ARR for 0.5 Hz interrupt rate
    TIM6->ARR = (9600) - 1;
    //Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 0x01 << TIM7_IRQn;
    //Set TIM7 a higher priority than TIM6
    NVIC_SetPriority(TIM7_IRQn, 1); // Priority 1
    //Enable counter
    TIM7->CR1 |= TIM_CR1_CEN;
    //TIM7->CR2 |= TIM_CR2_MMS_1; do not need to do this since this is for timer synchronization

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  
    GPIOC->MODER |= GPIO_MODER_MODER6_0;
}

// read the accelerometers, check for shaking/tilt
// check x-y position, refresh the screen 
void TIM6_IRQHandler(){
    TIM6->SR &= ~TIM_SR_UIF; //acknowledge the interrupt'
    //read X and Y
    readXY();
    xCurr = (xyVals[0])*240/4095;
    yCurr = (xyVals[1])*320/4096; // Should these denomenators be the same?
    LCD_DrawPoint(xCurr, yCurr, BLACK);
    //check for shaking
    //if shaking clear screen, save, wait till acelerometer is restored to flat
    //if tilting save screen and switch
    //update screen
}

void TIM7_IRQHandler(){
    TIM7->SR &= ~TIM_SR_UIF; //acknowledge the interrupt'
    
    if (led_curr == 1) {
        GPIOC->ODR |= 1 << 6;
        led_curr = 0;
    }
    else {
        GPIOC->ODR &= ~(1 << 6);
        led_curr = 1;   
    }
    //save current screen to SD card
}

int main(void) {
    internal_clock();
    //call setup functions
    setup_adcs();
    init_dmas();
    enable_dmas();
    
    //init_I2C();
    init_spi1();
    LCD_Setup();
    LCD_Clear(WHITE);
    
    init_tim6();
    init_tim7();

    for(;;);
}