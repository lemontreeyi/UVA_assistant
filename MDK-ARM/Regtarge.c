#include "stdio.h"

#pragma import(__use_no_semihosting_swi)
extern int SendChar(int ch); // �����ⲿ��������main�ļ��ж���
extern int GetKey(void);

struct __FILE
{
    int handle; // Add whatever you need here
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
    return (SendChar(ch));
}

int fgetc(FILE *f)
{
    return (SendChar(GetKey()));
}

void _ttywrch(int ch)
{
    SendChar (ch);
}

int ferror(FILE *f)
{
    // Your implementation of ferror
    return EOF;
}

void _sys_exit(int return_code)
{
    label: goto label; // endless loop
}