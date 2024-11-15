#include "spi.h"
#include "stm32f091xc.h"

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
    GPIOB->MODER |= 0x00000A80; // set PB3-5, to alternate function 0 (spi things)
    GPIOA->AFR[0] &= ~(0x00000FC0); //set AFR for PB3-5, to AF0
        
    GPIOB->MODER |= 0x10410010; //set PB2, 8, 11, 14 to P/P output
    //GPIOB->OTYPER |= 0x4904;
    
    //setup SPI1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //enable clock to spi1
    SPI1->CR1 &= ~(SPI_CR1_SPE); //disable the SPI channel
    SPI1->CR1 &= ~(SPI_CR1_BR); //set the baud rate to 12 MHz
    
    SPI1->CR1 |= SPI_CR1_BR_0;

    SPI1->CR1 |= SPI_CR1_MSTR; //set to master mode
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; //enable software slave management and internal slave select 
    SPI1->CR2 &= ~(SPI_CR2_DS); //set data size to 8 bit (* by writing zeros the register is reset to 8 bit default)

    SPI1->CR2 |= SPI_CR2_SSOE; //set the SS output enable bit
    SPI1->CR2 |= SPI_CR2_NSSP; //enable NSSP mode
    SPI1->CR1 |= SPI_CR1_SPE; //enable the SPI channel
}