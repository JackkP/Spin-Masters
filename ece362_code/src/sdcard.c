// file with all of the necessesary SDcard functions for main

#include <stdio.h>
#include <string.h>
#include "sdcard.h"

void write(char* pg, char* str){
    //rm file
    //fopen new file
    //fputs string
}

uint8_t* read(char* pg) { //returns a string of 
    //fopen file
    //fgetc for 9600 characters
}

void enable_sdcard();
void disable_sdcard();
void init_sdcard_io();
void sdcard_io_high_speed();
