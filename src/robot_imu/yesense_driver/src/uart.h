#ifndef _UART_HPP
#define _UART_HPP

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>


#define OPENFAILED -1
#define SETUPFAILED -2
#define NONEPARITY 0
#define ODDPARITY  1
#define EVENPARITY 2
#define SETUPSUCCEED 3

class UART
{

public:
    int fd;

public:
    UART(){};
    ~UART(){};

    int OpenUart(const char *pDev, int baudrate, int bits = 8,int parity = NONEPARITY,int stopbits = 1);
    int ReadUart(unsigned char* buf, int size);
    int WriteUart(unsigned char * buf, int size);
    int CloseUart();
};

#endif //_UART_HPP