// file with all of the necessesary SDcard functions for main

#include <stdio.h>
#include <string.h>
#include "sdcard.h"
#include "stm32f0xx.h"
#include "spi.h"

void write(char* pg, char* str){
    //rm file
    //fopen new file
    //fputs string
    return;
}

uint8_t* read(char* pg) { //returns a string of 
    //fopen file
    //fgetc for 9600 characters
    return NULL;
}

void enable_sdcard() {
    GPIOB->ODR &= ~(0x01 << 2); //set PB2 low to enable the SD card
}

void disable_sdcard(){
    GPIOB->ODR |= 0x01 << 2; //set PB2 high to enable the SD card
}

void init_sdcard_io() {
    init_spi1_slow();
    GPIOB->MODER &= ~(0x00000030); //clear MODER for PB2
    GPIOB->MODER |= 0x00000010; //set MODER to P/P output for PB2
    disable_sdcard();
}

void sdcard_io_high_speed() {
    SPI1->CR1 &= ~(SPI_CR1_SPE); //disable the SPI channel
    SPI1->CR1 &= ~(SPI_CR1_BR); //set the baud rate to 12 MHz
    SPI1->CR1 |= SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_SPE; //enable the SPI channel
}
