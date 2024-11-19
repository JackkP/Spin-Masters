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

/*
//todo: remake into read function
void cat(int argc, char *argv[])
{
    for(int i=1; i<argc; i++) {
        FIL fil;        // File object
        char line[100]; // Line buffer
        FRESULT fr;     // FatFs return code

        // Open a text file
        fr = f_open(&fil, argv[i], FA_READ);
        if (fr) {
            print_error(fr, argv[i]);
            return;
        }

        // Read every line and display it
        while(f_gets(line, sizeof line, &fil))
            printf(line);
        // Close the file
        f_close(&fil);
    }
} */

// useful function to convert string to integer
int to_int(char *start, char *end, int base)
{
    int n = 0;
    for( ; start != end; start++)
        n = n * base + (*start - '0');
    return n;
}

//can leave this lmao
static const char *month_name[] = {
        [1] = "Jan",
        [2] = "Feb",
        [3] = "Mar",
        [4] = "Apr",
        [5] = "May",
        [6] = "Jun",
        [7] = "Jul",
        [8] = "Aug",
        [9] = "Sep",
        [10] = "Oct",
        [11] = "Nov",
        [12] = "Dec",
};

//setting date to a random date close to the due date lmao
void date()
{
    set_fattime(2024, 11, 18, 10, 30, 0);
}

/*//todo modify into make new file and write entire char array to it
void input(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Specify only one file name to create.");
        return;
    }
    FIL fil;        //File object
    char line[100]; //Line buffer
    FRESULT fr;     //FatFs return code
    fr = f_open(&fil, argv[1], FA_WRITE|FA_CREATE_NEW);
    if (fr) {
        print_error(fr, argv[1]);
        return;
    }
    printf("To end input, enter a line with a single '.'\n");
    for(;;) {
        fgets(line, sizeof(line)-1, stdin);
        if (line[0] == '.' && line[1] == '\n')
            break;
        int len = strlen(line);
        if (line[len-1] == '\004')
            len -= 1;
        UINT wlen;
        fr = f_write(&fil, (BYTE*)line, len, &wlen);
        if (fr)
            print_error(fr, argv[1]);
    }
    f_close(&fil);
}*/

void makefile(){
    FIL fil;
    char fname[9] = {"holyfuck\0"};
    FRESULT fr;
    fr = f_open(&fil, fname, FA_WRITE|FA_CREATE_NEW);
    f_close(&fil);
}

// mount file system
void mount()
{
    FATFS *fs = &fs_storage;
    if (fs->id != 0) {
        // error, system already mounted
        return;
    }
    int res = f_mount(fs, "", 1);
    if (res != FR_OK)
        return; //error occurred while mounting
}

// remove file by filename
void rm(int argc, char *filename)
{
    FRESULT res;
    for(int i=1; i<argc; i++) {
        res = f_unlink(filename);
        if (res != FR_OK) ;
            // error, unable to remove filename but we don't really care
    }
}
