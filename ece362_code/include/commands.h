#ifndef __COMMANDS_H__
#define __COMMANDS_H__

struct commands_t {
    const char *cmd;
    void      (*fn)(int argc, char *argv[]);
};

void mount();
void write_screen(int pagenum, uint8_t* data, int datasize);
void LCD_setDisp(uint8_t* data);

#endif /* __COMMANDS_H_ */
