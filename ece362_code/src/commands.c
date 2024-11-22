// support functions for SD card as used in sdcard.c

#include "stm32f0xx.h"
#include "ff.h"
#include "lcd.h"
#include "commands.h"
#include <string.h>
#include <stdio.h>

// Data structure for the mounted file system.
FATFS fs_storage;

typedef union {
    struct {
        unsigned int bisecond:5; // seconds divided by 2
        unsigned int minute:6;
        unsigned int hour:5;
        unsigned int day:5;
        unsigned int month:4;
        unsigned int year:7;
    };
} fattime_t;

// Current time in the FAT file system format.
static fattime_t fattime;

void set_fattime(int year, int month, int day, int hour, int minute, int second)
{
    fattime_t newtime;
    newtime.year = year - 1980;
    newtime.month = month;
    newtime.day = day;
    newtime.hour = hour;
    newtime.minute = minute;
    newtime.bisecond = second/2;
    int len = sizeof newtime;
    memcpy(&fattime, &newtime, len);
}

void advance_fattime(void)
{
    fattime_t newtime = fattime;
    newtime.bisecond += 1;
    if (newtime.bisecond == 30) {
        newtime.bisecond = 0;
        newtime.minute += 1;
    }
    if (newtime.minute == 60) {
        newtime.minute = 0;
        newtime.hour += 1;
    }
    if (newtime.hour == 24) {
        newtime.hour = 0;
        newtime.day += 1;
    }
    if (newtime.month == 2) {
        if (newtime.day >= 29) {
            int year = newtime.year + 1980;
            if ((year % 1000) == 0) { // we have a leap day in 2000
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            } else if ((year % 100) == 0) { // no leap day in 2100
                if (newtime.day > 28)
                newtime.day -= 27;
                newtime.month = 3;
            } else if ((year % 4) == 0) { // leap day for other mod 4 years
                if (newtime.day > 29) {
                    newtime.day -= 28;
                    newtime.month = 3;
                }
            }
        }
    } else if (newtime.month == 9 || newtime.month == 4 || newtime.month == 6 || newtime.month == 10) {
        if (newtime.day == 31) {
            newtime.day -= 30;
            newtime.month += 1;
        }
    } else {
        if (newtime.day == 0) { // cannot advance to 32
            newtime.day = 1;
            newtime.month += 1;
        }
    }
    if (newtime.month == 13) {
        newtime.month = 1;
        newtime.year += 1;
    }

    fattime = newtime;
}

uint32_t get_fattime(void)
{
    union FattimeUnion {
        fattime_t time;
        uint32_t value;
    };

    union FattimeUnion u;
    u.time = fattime;
    return u.value;
}

//check if a file exists and read a long array of bytes from it
int load_screen(int pagenum, uint8_t* buf, int datasize) {
    FATFS *fs = &fs_storage;
    FRESULT res = f_mount(fs, "", 1);

    FIL fil;
    //set filename to be page number (null terminated string)
    char fname[2] = {" \0"};
    fname[0] = pagenum + 48;
    FRESULT fr;
    //open the file to read
    fr = f_open(&fil, fname, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK) {
        fr = f_close(&fil);
        res = f_mount(fs, "", 1);
        return 0;
    }
    //read every byte from the file into the buffer
    int br;
    f_read(&fil, buf, datasize, &br);
    //close file
    fr = f_close(&fil);
    res = f_mount(fs, "", 1);
    return br;
}

// useful function to convert string to integer
int to_int(char *start, char *end, int base)
{
    int n = 0;
    for( ; start != end; start++)
        n = n * base + (*start - '0');
    return n;
}

//make a new file and write a long array of bytes to it
void save_screen(int pagenum, uint8_t* data, int datasize) {
    FATFS *fs = &fs_storage;
    FRESULT res = f_mount(fs, "", 1);
    
    FIL fil;
    //set filename to be page number (null terminated string)
    char fname[2] = {" \0"};
    fname[0] = pagenum + 48;
    res = f_unlink(fname);
    //open the file to write or create new file
    res = f_open(&fil, fname, FA_WRITE|FA_CREATE_NEW);
    //write every byte to the file
    int bw;
    res = f_write(&fil, data, datasize, &bw);
    res = f_sync(&fil);
    //close file
    res = f_close(&fil);
    res = f_mount(fs, "", 1);
}

// mount file system
void mount()
{
    FATFS *fs = &fs_storage;
    if (fs->id != 0) {
        // error, system already mounted
        return;
    }
    FRESULT res = f_mount(fs, "", 1);
    if (res != FR_OK)
        return; //error occurred while mounting
}

//set display pixels based on bit-array
void LCD_setDisp(uint8_t* data){
    int x, y;
    for (x = 0; x < 240; x++){
        for (y=0; y<320; y++){
            int p_curr = x*320 + y;
            if ((data[p_curr/8] & (1 << (p_curr % 8))) != 0){
                LCD_DrawPoint(x, y, BLACK);
            }

        }
    }
}
