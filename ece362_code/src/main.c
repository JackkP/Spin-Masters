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

/*void init_tim7(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    //Set prescaler to 10,000 - 1
    TIM7->PSC = 10000 - 1;
    //Calculate ARR for 0.5 Hz interrupt rate
    TIM6->ARR = (9600) - 1;
    //Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = ISER_TIM7;
    //Enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
}*/

void TIM7_IRQHandler(){

}

//===========================================================================
// Configure SDA and SCL.
//===========================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // initialize clock for I2C

    // configure PA9->I2C1_SCL (GPIOA-AF4), PA10->I2C1_SDA (GPIOA-AF4)
    GPIOA->MODER &= ~0x3c0000; // reset PA9&PA10
    GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1; // configure PA9&PA10 to AF mode
    GPIOA->AFR[1] |= GPIO_AFRH_AFRH2; // AFR to AF4
}


//===========================================================================
// Configure I2C for GPIOA
//===========================================================================
void init_i2c(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // configure clock for I2C1
    I2C1->CR1 &= ~I2C_CR1_PE; // disable I2C peripheral for setting programming

    /* control register settings 1:
     * -turn off analog noise filter
     * -enable error interrupts 
     * -disable clock stretching
     */
    I2C1->CR1 |= I2C_CR1_ANFOFF | I2C_CR1_ERRIE | I2C_CR1_NOSTRETCH; 
    
    // set the I2C to "fast mode" @400kHz
    I2C1->TIMINGR = (u_int32_t) 0x00B01A4B; // hex number came from example don't ask me about it
    
    I2C1->CR2 &= ~I2C_CR2_ADD10; // 7-bit addressing, not 10-bit (for master mode)
    I2C1->CR2 |= I2C_CR2_STOP; // send STOP condiciton after last byte of transmission

    I2C1->CR1 |= I2C_CR1_PE; // enable peripheral after programming
}

//===========================================================================
// Necessary functions for communication with I2C device:
// i2c_(start, stop, waitidle, senddata, recvdata, clearnack, checknack)
//===========================================================================


//===========================================================================
// Send a START bit.
//===========================================================================
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir) {
    // 0. Take current contents of CR2 register. 
    uint32_t tmpreg = I2C1->CR2;

    // 1. Clear the following bits in the tmpreg: SADD, NBYTES, RD_WRN, START, STOP
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);

    // 2. Set read/write direction in tmpreg.
    tmpreg |= ((dir << 9) & I2C_CR2_RD_WRN);

    // 3. Set the target's address in SADD (shift targadr left by 1 bit) and the data size.
    tmpreg |= ((targadr<<1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);

    // 4. Set the START bit.
    tmpreg |= I2C_CR2_START;

    // 5. Start the conversion by writing the modified value back to the CR2 register.
    I2C1->CR2 = tmpreg;
}

//===========================================================================
// Send a STOP bit.
//===========================================================================
void i2c_stop(void) {
    // 0. If a STOP bit has already been sent, return from the function.
    // Check the I2C1 ISR register for the corresponding bit.    
    if (I2C_ISR_STOPF) {
        return;
    }

    // 1. Set the STOP bit in the CR2 register.
    I2C1->CR2 |= I2C_CR2_STOP;

    // 2. Wait until STOPF flag is reset by checking the same flag in ISR.
    while ((I2C1->ISR & I2C_ISR_STOPF) == 0) {    }

    // 3. Clear the STOPF flag by writing 0 to the corresponding bit in the ICR.
    I2C1->ICR |= 1 << 5; // resets the STOP flag clear bit
}

//===========================================================================
// Wait until the I2C bus is not busy. (One-liner!)
//===========================================================================
void i2c_waitidle(void) {
    while ((I2C1->ISR & I2C_ISR_BUSY) == 0) {}
}

//===========================================================================
// Send each char in data[size] to the I2C bus at targadr.
//===========================================================================
int8_t i2c_senddata(uint8_t targadr, uint8_t data[], uint8_t size) {
    // wait until I2C idle 
    i2c_waitidle();

    // send a start condition to the target address with the write bit set (dir=0)
    i2c_start(targadr, size, 0);

    int count = 0;
    while ((I2C1->ISR & I2C_ISR_TXIS) == 0) {
        count += 1;
        if (count > 1000000)
            return -1;
        if (i2c_checknack()) {
            i2c_clearnack();
            i2c_stop();
            return -1;
        }
        // mask data[i] with I2C_TXDR_TXDATA to make sure only 8 bits long, write to TXDR
        I2C1->TXDR |= I2C_TXDR_TXDATA_Msk & data[];
    }

    // wait until transmission complete or not acknowledge are set
    while ((I2C1->ISR & I2C_ISR_TC & I2C_ISR_NACKF) == 0) {}

    // if end reached without acknowledging data, unsuccessful
    if (i2c_checknack()) { return -1; }

    // successful
    i2c_stop();
    return 0;
}

//===========================================================================
// Receive size chars from the I2C bus at targadr and store in data[size].
//===========================================================================
int i2c_recvdata(uint8_t targadr, void *data, uint8_t size) {
    // wait until I2C idle 
    i2c_waitidle();

    // send a start condition to the target address with the read bit set (dir=1)
    i2c_start(targadr, size, 1);

    int count = 0;
    while ((I2C1->ISR & I2C_ISR_RXNE) == 0) {
        count += 1;
        if (count > 1000000)
            return -1;
        if (i2c_checknack()) {
            i2c_clearnack();
            i2c_stop();
            return -1;
        }

    }
}

//===========================================================================
// Clear the NACK bit. (One-liner!)
//===========================================================================
void i2c_clearnack(void) {
    // just clear the NACK flag
    I2C1->ICR |= I2C_ICR_NACKCF;
}

//===========================================================================
// Check the NACK bit. (One-liner!)
//===========================================================================
int i2c_checknack(void) {
    // check to make sure the NACK flag is cleared
    if (!I2C_ISR_NACKF) {return 1;}
}



int main(void) {
    internal_clock();
    //call setup functions
    //setup_adcs();
    //init_dmas()
    for(;;);
}
