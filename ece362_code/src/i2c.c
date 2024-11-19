#include "stm32f0xx.h"
#include <stdio.h>
#include "i2c.h"

void internal_clock();
void enable_ports();
void init_i2c();
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir);
void i2c_stop();
void i2c_waitidle();
void i2c_clearnack();
int i2c_checknack();


//===========================================================================
// Configure SDA and SCL.
//===========================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // initialize clock for GPIOA pins

    // configure PA9->I2C1_SCL (GPIOA-AF4), PA10->I2C1_SDA (GPIOA-AF4)
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10); // reset PA9&PA10
    GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1; // configure PA9&PA10 to AF mode
    GPIOA->AFR[1] |= (3 << GPIO_AFRH_AFRH2_Pos) | (3 << GPIO_AFRH_AFRH1_Pos); // AFR to AF4
}

//===========================================================================
// Configure I2C1.
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
    // I2C1->CR2 |= I2C_CR2_STOP; // send STOP conditon after last byte of transmission
    //** check here ^ for potential issues */

    I2C1->CR1 |= I2C_CR1_PE; // enable peripheral after programming
}

//===========================================================================
// Send a START bit.
//===========================================================================
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir) {
    // 0. Take current contents of CR2 register. 
    uint32_t tmpreg = I2C1->CR2;

    // 1. Clear the following bits in the tmpreg: SADD, NBYTES, RD_WRN, START, STOP
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);

    // 2. Set read/write direction in tmpreg. ** check here for potential issues
    tmpreg &= ~(tmpreg & (dir << I2C_CR2_RD_WRN_Pos));

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
    while (!(I2C1->ISR & I2C_ISR_STOPF) == 0) {    }

    // 3. Clear the STOPF flag by writing 0 to the corresponding bit in the ICR.
    I2C1->ICR |= 1 << I2C_ICR_STOPCF_Pos; // resets the STOP flag clear bit
}

//===========================================================================
// Wait until the I2C bus is not busy. (One-liner!)
//===========================================================================
void i2c_waitidle(void) {
    while (!(I2C1->ISR & I2C_ISR_BUSY) == 0) {}
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
    for (int i = 0; i <= size - 1; i++) {
        while ((I2C1->ISR & I2C_ISR_TXIS) == 0) {
            count += 1;
            if (count > 1000000)
                return -1;
            if (i2c_checknack()) {
                i2c_clearnack();
                i2c_stop();
                return -1;
            }
        }
        // mask data[i] with I2C_TXDR_TXDATA to make sure only 8 bits long, write to TXDR
        I2C1->TXDR |= I2C_TXDR_TXDATA_Msk & data[i];
    }
    
    // wait until transmission complete and not acknowledge are set
    while (!(I2C1->ISR & (I2C_ISR_TC & I2C_ISR_NACKF)) == 0) {}

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
    // start a loop from 0 to size-1 and do the following for each iteration
    for (int i = 0; i <= size - 1; i++) {
        // wait until the RXNE flag is set in the ISR, and quit if it takes too long
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
        // mask data in the RXDR register with I2C_RXDR_RXDATA to make sure only 8 bits, store in data[i]
        data++;
        data = (I2C1->RXDR & I2C_RXDR_RXDATA_Msk);
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
    if (!I2C_ISR_NACKF) {
        return 1;
    } else {
        return 0;
    }
}

//===========================================================================
// EEPROM functions
// We'll give these so you don't have to figure out how to write to the EEPROM.
// These can differ by device.

#define EEPROM_ADDR 0x57

void accel_write(uint16_t loc, const char* data, uint8_t len) {
    uint8_t bytes[34];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    for(int i = 0; i<len; i++){
        bytes[i+2] = data[i];
    }
    i2c_senddata(EEPROM_ADDR, bytes, len+2);
}

void accel_read(uint16_t loc, char data[], uint8_t len) {
    // ... your code here
    uint8_t bytes[2];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    i2c_senddata(EEPROM_ADDR, bytes, 2);
    i2c_recvdata(EEPROM_ADDR, data, len);
}