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

void nano_wait(int);
void internal_clock();

uint16_t xVal = 0; //analog xvalue
uint16_t yVal = 0; //analog yvalue

uint16_t xCurr = 0; //x coordinate
uint16_t yCurr = 0; //y coordinate

uint16_t x_0 = 0; //zero x-acceleration
uint16_t y_0 = 0; //zero y-acceleration
uint16_t z_0 = 0; //zero z-acceleration

uint16_t screen = 1; //current screen

uint16_t xhist[30]; //x value history
uint16_t yhist[30]; //y value history
uint16_t zhist[30]; //z value history

//some type of grid to represent the pixels

//uint8_t pgrid[320][240] (represent with a 0/1)


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

//used as an interrupt to refresh the LCD display & read the acceleration at 30Hz
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
    //TIM6->CR2 |= TIM_CR2_MMS_1; do not need to do this since this is for timer synchronization
}

//used to triger an interrupt to save the display every half second
void init_tim7(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    //Set prescaler to 10,000 - 1
    TIM7->PSC = 10000 - 1;
    //Calculate ARR for 0.5 Hz interrupt rate
    TIM6->ARR = (9600) - 1;
    //Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 0x01 << TIM7_IRQn;
    //Enable counter
    TIM7->CR1 |= TIM_CR1_CEN;
    //TIM7->CR2 |= TIM_CR2_MMS_1; do not need to do this since this is for timer synchronization
}

// read the accelerometers, check for shaking/tilt
// check x-y position, refresh the screen 
void TIM6_IRQHandler(){
    TIM6->SR &= ~TIM_SR_UIF; //acknowledge the interrupt'
    //read X
    //read Y
    //check for shaking
    //if shaking clear screen, save, wait till acelerometer is restored to flat
    //if tilting save screen and switch
    //update screen
}

void TIM7_IRQHandler(){
    TIM7->SR &= ~TIM_SR_UIF; //acknowledge the interrupt'
    //save current screen to SD card
}

//setup spi1 peripheral to control TFT LCD display and SD card
void init_spi1() {
    //initalize GPIOB peripheral
    
    //PB2 SD chip select
    //PB3 SCK
    //PB4 MISO
    //PB5 MOSI
    //PB8 TFT chip select
    //PB11 TFT Reset
    //PB14 D/C (data/command) select signal, high = write data, low = write command

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable peripheral clock for PortB
    GPIOB->MODER &= ~(0x30C30FF0); //clear MODER for PB2-5, 8, 11, 14
    GPIOB->MODER |= 0x80008800; // set PA5, 7, 15 to alternate function
    
    GPIOA->AFR[0] &= ~(0xF0F00000); //set AFR for PA5, 7, to AF0
    GPIOA->AFR[1] &= ~(0xF0000000); //set AFR for PA15 to AF0

    //setup SPI2 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enable clock to spi2
    SPI1->CR1 &= ~(SPI_CR1_SPE); //disable the SPI channel
    SPI1->CR1 |= SPI_CR1_BR; //set the baud rate to the minimum baud rate ck/256
    SPI1->CR1 |= SPI_CR1_MSTR; //set to master mode
    SPI1->CR2 |= SPI_CR2_DS; //set data size to 10 bit
    SPI1->CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2);
    SPI1->CR2 |= SPI_CR2_SSOE; //set the SS output enable bit
    SPI1->CR2 |= SPI_CR2_NSSP; //enable NSSP mode
    SPI1->CR2 |= SPI_CR2_TXDMAEN; //enable DMA transfer request upon transmit buffer empty
    SPI1->CR1 |= SPI_CR1_SPE; //enable the SPI channel
}

int main(void) {
    internal_clock();
    //call setup functions
    //setup_adcs();
    //init_dmas();
    
    //init_I2C();
    init_spi();
    
    init_tim6();
    init_tim7();

    for(;;);
}
