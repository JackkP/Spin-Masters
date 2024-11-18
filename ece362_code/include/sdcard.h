
#include <stdio.h>

#ifndef SDCARD_H
#define SDCARD_H


void write(char* pg, char* str);
uint8_t* read(char* pg);

void enable_sdcard();
void disable_sdcard();
void init_sdcard_io();
void sdcard_io_high_speed();

#endif //SDCARD_H