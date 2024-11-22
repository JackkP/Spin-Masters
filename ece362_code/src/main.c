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
#include "sdcard.h"
#include "commands.h"
#include "i2c.h"

void nano_wait(int);
void internal_clock();
// // i2c stuff
void enable_ports();
void init_i2c();
void accel_write(uint16_t loc, uint8_t* data, uint8_t len);
void accel_read(uint16_t loc, uint8_t data[], uint8_t len);
void i2c_stop();

void update_accel_XYZ(); // for some reason it prefers it like this

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

static uint8_t pgrid[9600]; //(represent with a 0/1)

// acceleration variables from i2c
uint8_t accel_dataX[1]; // x-axis accelerometer data
uint8_t accel_dataY[1]; // y-axis accelerometer data
uint8_t accel_dataZ[1]; // z-axis accelerometer data

int led_curr = 0;


void init_dmas(void) {
   RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
   //DMA1 -> CSELR |= DMA1_CSELR_CH1_ADC;// Channel Select ADC on 1
   //DMA1 -> CSELR |= DMA1_CSELR_CH2_ADC;// Channel Select ADC on 2

   DMA1_Channel1 -> CCR &= ~DMA_CCR_EN; //ignore -> // X
   //DMA1_Channel2 -> CCR &= ~DMA_CCR_EN; // Y
   
   DMA1_Channel1 -> CMAR = (uint32_t) xyVals;
   DMA1_Channel1 -> CPAR = (uint32_t) &(ADC1 -> DR);
   DMA1_Channel1 -> CNDTR = 0x2;
   DMA1_Channel1 -> CCR &= ~(DMA_CCR_DIR); // read from peripheral
   DMA1_Channel1 -> CCR |= DMA_CCR_MINC; // memory increment
   DMA1_Channel1 -> CCR |= DMA_CCR_CIRC; // Circular mode
   // DMA1_Channel1 -> CCR |= DMA_CCR_TEIE; // do not want a transfier error interrupt
   //DMA1_Channel1 -> CCR &= ~(DMA_CCR_MSIZE); // 00 - 8b
   //DMA1_Channel1 -> CCR &= ~(DMA_CCR_PSIZE); // 00 - 8b
   DMA1_Channel1 -> CCR |= DMA_CCR_MSIZE_0; // memory size 01 -> 16b since using 16 bit integers
   DMA1_Channel1 -> CCR |= DMA_CCR_PSIZE_0; //set peripheral value size to 16b

   DMA1_Channel1 -> CCR |= DMA_CCR_EN; //enable the DMA

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
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock to port A

    GPIOA -> MODER |= (GPIO_MODER_MODER1); //set PA1 to Analog mode
    GPIOA -> MODER |= (GPIO_MODER_MODER0); //set PA0 to Analog mode

    RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable ADC1
    
    RCC -> CR2 |= RCC_CR2_HSI14ON; //turn on the internal 14MHz clock
    while((RCC -> CR2 & RCC_CR2_HSI14RDY) == 0); //wait for 14MHz clock to be ready
    ADC1 -> CR |= ADC_CR_ADEN; //Enable ADC
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0); //wait for the ADC to be ready

    ADC1->CHSELR = 0x1 << 1; //select ADC channel 1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //wait for the ADC to be ready

    //these steps must happen after ADC calibration per pg251 of FRM (we don't do ADC calibration here so doesn't matter)
    ADC1 -> CFGR1 |= ADC_CFGR1_DMAEN; //enable DMA transfer request upon ADC conversion completion
    ADC1 -> CFGR1 |= ADC_CFGR1_DMACFG; //set DMA transfer request to circular mode

}

void readXY(void) {
    //Use DMA Channel 1
    
    ADC1 -> CHSELR = 0b1; // set channel selection register to channel 1
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0); //wait for ADC to be ready
    ADC1 -> CR |= ADC_CR_ADSTART; //start ADC conversion
    while((ADC1 -> CR & ADC_CR_ADSTART) != 0); //wait until the conversion is done

    ADC1 -> CHSELR = 0b10; // set channel selection register to channel 2
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0); //wait for previous transfer to finish
    ADC1 -> CR |= ADC_CR_ADSTART;
    while((ADC1 -> CR & ADC_CR_ADSTART) != 0); //wait until the conversion is done
}

//used as an interrupt to refresh the LCD display & read the acceleration at 20Hz
void init_tim6(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    //Set prescaler to 500
    TIM6->PSC = 500-1;
    //Calculate ARR for 200 Hz interrupt rate
    TIM6->ARR = 480-1;
    //Enable update interrupt
    TIM6->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 1 << 17;
    // changing this so tim6 and tim7 have same interrupt priority so one must complete before the other can //Set TIM6 a lower priority than TIM7
    // NVIC_SetPriority(TIM6_DAC_IRQn, 2); //Priority 2
    //Enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    //TIM6->CR2 |= TIM_CR2_MMS_1; do not need to do this since this is for timer synchronization
}

//used to triger an interrupt to save the display every half second
void init_tim7(void) {
    //Enable RCC clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    //Set prescaler to 5,000 - 1
    TIM7->PSC = 5000 - 1;
    //Calculate ARR for 0.2 Hz interrupt rate
    TIM7->ARR = 48000 - 1;
    //Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;
    //Unmask the interrupt in the NVIC
    NVIC->ISER[0] = 0x01 << TIM7_IRQn;
    // changing this so tim6 and tim7 have same interrupt priority so one must complete before the other can //Set TIM7 a higher priority than TIM6
    // NVIC_SetPriority(TIM7_IRQn, 1); // Priority 1
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
    xCurr = (xyVals[0])*239/4095;
    yCurr = (xyVals[1])*319/4095;
    //draw point
    LCD_DrawPoint(xCurr, yCurr, BLACK);
    //update this point in the pgrid
    int p_curr = xCurr*320 + yCurr;
    pgrid[p_curr/8] |= 1 << p_curr % 8;

    //check for shaking
    //if shaking clear screen, save, wait till acelerometer is restored to flat
    //if tilting save screen and switch
}

void TIM7_IRQHandler(){
    TIM7->SR &= ~TIM_SR_UIF; //acknowledge the interrupt'
    
    //toggle LED on and off when savings
    GPIOC->ODR |= 1 << 6;
    
    //save current screen to SD card
    save_screen(screen, pgrid, 9600);

    GPIOC->ODR &= ~(1 << 6);

}

// chaos commences

void config_accel() {
    uint8_t config[1];
    config[0] = 0x1;
    accel_write(0x2A, config, 1);
}

void config_int_pins() {
    // Configuration occurs in the CTRL_REG5 (0x2E)
    // identifies TRANS, LNDPRT, DRDY
    uint8_t intList[1];
    intList[0] = 0x31; // 0011 0001
    accel_write(0x2E, intList, 1);
}

void set_motion_limits() {
    // set the threshold limits for the transient interrupt
    uint8_t force[1];
    force[0] = 0x30; // accel of roughly 3g's
    // write the acceleration threshold limit to the transient threshold register
    accel_write(0x1F, force, 1);
}

//omit setting tilt limits, default already at +/-75 degrees
// void set_tilt_limits() {
//     // set the back/front trip angle threshold to be above 75 degrees
//     char tilt[1] = 
// }

void init_exti() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; 

    GPIOA->MODER |= GPIO_MODER_MODER7;
    
    SYSCFG->EXTICR[1] &= ~(0xF000); // cover EXTI7 for PA

    // trigger on the rising edge
    EXTI->RTSR |= 1<<7; // only need pin PA7
    EXTI->IMR |= 1<<7; // also only need pin PA7

    NVIC->ISER[0] |= 1<<EXTI4_15_IRQn; // call interrupt handler
}

void EXTI4_15_IRQHandler() {
    EXTI->PR |= EXTI_PR_PR7;
    // check whether interrupt has been raised
    uint8_t int_stat[1];
    // read from int_source (0x0C)
    // int_stat = [SRC_ASLP, SRC_FIFO, SRC_TRANS, SRC_LNDPRT, SRC_PULSE, SRC_FF_MT, --, SRC_DRDY]
    accel_read(0x0C, int_stat, 1); 
    GPIOC->ODR &= ~(1 << 6);
    // if (led_curr == 1) {
    //     GPIOC->ODR |= 1 << 6;
    //     led_curr = 0;
    // }
    // else {
    //     GPIOC->ODR &= ~(1 << 6);
    //     led_curr = 1;   
    // }

    // check interrupt status in order of relative priority
    if ((int_stat[0] && (1<<5)) == 1)
    {
        // acknowledge transient interrupt - disable correct bit, set to CTRL_REG4 
        int_stat[0] &= (~(1<<5));
        accel_write(0x2D, int_stat, 1);

        // clear the screen
        LCD_Clear(WHITE);
    }

    if ((int_stat[0] && (1<<4)) == 1)
    {
        // acknowledge orientation interrupt - disable correct bit, set to CTRL_REG4 
        int_stat[0] &= (~(1<<4));
        accel_write(0x2D, int_stat, 1);

        // clear screen
        //LCD_Clear(WHITE);
    }

    if ((int_stat[0] && (1)) == 1) {
        // acknowledge data ready interrupt - disable correct bit, set to CTRL_REG4 
        int_stat[0] &= (~(1<<0));
        accel_write(0x2D, int_stat, 1);

        // // call new values for X,Y,Z
        // update_accel_XYZ();
    }
}

void update_accel_XYZ() {
    // read data from the status register of accelerometer
    // only getting MSB because i don't think 14 bits is necessary
    // uint8_t targadr = 0x1D;
    // uint8_t data[1] = {0x1}; 
    // uint8_t size = 1;
    // i2c_start(targadr, size, 0);

    // for (int i = 0; i <= size - 1; i++) {
    //     int count = 0;
    //     while ((I2C1->ISR & I2C_ISR_TXIS) == 0) {
    //         count += 1;
    //         if (count > 1000000)
    //             return;
    //         if (i2c_checknack()) {
    //             i2c_clearnack();
    //             i2c_stop();
    //             return;
    //         }
    //     }
    //     // mask data[i] with I2C_TXDR_TXDATA to make sure only 8 bits long, write to TXDR
    //     I2C1->TXDR = I2C_TXDR_TXDATA & data[i];
    // }
    
    // // wait until transmission complete and not acknowledge are set
    // while ((I2C1->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) == 0) {}

    // // if end reached without acknowledging data, unsuccessful
    // if (i2c_checknack()) { return; }

    // // now receiving
    // i2c_start(targadr, size, 1);

    // // start a loop from 0 to size-1 and do the following for each iteration
    // for (int i = 0; i <= size - 1; i++) {
    //     // wait until the RXNE flag is set in the ISR, and quit if it takes too long
    //     int count = 0;
    //     while ((I2C1->ISR & I2C_ISR_RXNE) == 0) {
    //         count += 1;
    //         if (count > 1000000)
    //             return;
    //         if (i2c_checknack()) {
    //             i2c_clearnack();
    //             i2c_stop();
    //             return;
    //         }
    //     }
    //     // mask data in the RXDR register with I2C_RXDR_RXDATA to make sure only 8 bits, store in data[i]
    //     // data += sizeof(*data);
    //     data[i] = (I2C1->RXDR & I2C_RXDR_RXDATA);
    // }

    // while ((I2C1->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) == 0) {}

    // if (!(I2C1->ISR & I2C_ISR_NACKF) == 0) { return -1; }

    // i2c_stop();
    

    accel_read(0x01, accel_dataX, 1);
    accel_read(0x03, accel_dataY, 1);
    accel_read(0x05, accel_dataZ, 1);
}


int main(void) {
    internal_clock();
    //call setup functions
    setup_adcs();
    init_dmas();
    enable_dmas();
    
    enable_ports();
    init_i2c();
    init_exti();
    config_int_pins();
    config_accel();
    // i2c_stop();
    set_motion_limits(); // should be sending data

    // uint8_t int_stat[1];
    // // read from int_source (0x0C)
    // // int_stat = [SRC_ASLP, SRC_FIFO, SRC_TRANS, SRC_LNDPRT, SRC_PULSE, SRC_FF_MT, --, SRC_DRDY]
    // accel_read(0x0C, int_stat, 1); 
    while (1)
       update_accel_XYZ();

    init_spi1();
    //mount();

    LCD_Setup();

    LCD_Clear(WHITE);
    load_screen(screen, pgrid, 9600);
    LCD_setDisp(pgrid);
    
    init_tim6();
    init_tim7();

    for(;;);
}