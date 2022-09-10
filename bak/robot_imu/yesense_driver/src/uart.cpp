#include "uart.h"

int UART::OpenUart(const char* pDev, int baudrate, int bits,int parity,int stopbits)
{

    //O_NDELAY标志告诉UNIX不关心端口另一端是否已经连接。如果不指定这个标志的话，除非DCD信号线上有space电压否则这个程序会一直睡眠
    /*O_NOCTTY标志告诉UNIX这个程序不会成为这个端口上的“控制终端”。
    **如果不这样做的话，所有的输入，比如键盘上过来的Ctrl+C中止信号等等，会影响到你的进程
    **但是通常情况下，用户程序不会使用这个行为
    */
    this->fd = open(pDev,O_RDWR | O_NDELAY | O_NOCTTY);

    if(this->fd == -1)
    {
        perror(pDev);
        return OPENFAILED;
    }

    struct termios PortParam;
    if(tcgetattr(this->fd,&PortParam) != 0)
    {
        perror("Read serialport attr failed");
        return SETUPFAILED;
    }

/********************* 设置波特率 (默认9600)************************/
    switch(baudrate)
    {
        case 460800:cfsetispeed(&PortParam,B460800);
                    cfsetospeed(&PortParam,B460800);
                    break;
        case 115200:cfsetispeed(&PortParam,B115200);
                    cfsetospeed(&PortParam,B115200);
                    break;
        case 57600:cfsetispeed(&PortParam,B57600);
                    cfsetospeed(&PortParam,B57600);
                    break;
        case 38400:cfsetispeed(&PortParam,B38400);
                    cfsetospeed(&PortParam,B38400);
                    break;
        case 19200:cfsetispeed(&PortParam,B19200);
                   cfsetospeed(&PortParam,B19200);
                   break;      
        case 9600:
          default:
                   cfsetispeed(&PortParam,B9600);
                   cfsetospeed(&PortParam,B9600);
                   break;
    }

/********************* 设置数据位 (默认8位)************************/
    PortParam.c_cflag &= ~CSIZE;
    switch(bits)
    {
        case 5: PortParam.c_cflag |= CS5;
                break;
        case 6: PortParam.c_cflag |= CS6;
                break;
        case 7: PortParam.c_cflag |= CS7;
                break;
        case 8: 
       default: PortParam.c_cflag |= CS8;
                break;

    }
    
/********************* 设置校验位 （默认无校验）************************/
    switch(parity)
    {
        case ODDPARITY: PortParam.c_cflag |= PARENB;
                        PortParam.c_cflag |= PARODD;
                        PortParam.c_iflag |= (INPCK | ISTRIP);
                        break;
        case EVENPARITY:PortParam.c_cflag |= PARENB;
                        PortParam.c_cflag &= ~PARODD;
                        PortParam.c_iflag |= (INPCK | ISTRIP);
                        break;
        case NONEPARITY:
        default        :PortParam.c_cflag &= ~PARENB;
                        PortParam.c_iflag &= ~INPCK;
                        PortParam.c_iflag &= ~(ICRNL | IGNCR);

                        break;
    }

/********************* 设置停止位 （默认2位）************************/
    switch(stopbits)
    {
        case 1: PortParam.c_cflag &= ~CSTOPB;
                break;
        case 2:
       default: PortParam.c_cflag |= CSTOPB; 
                break;
    }

    PortParam.c_cflag |= (CLOCAL | CREAD); //控制模式,保证程序不会成为端口的占有者,使能端口读取输入的数据
    PortParam.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);;//原始输入
    PortParam.c_iflag &= ~(IXON | IXOFF | IXANY);
    PortParam.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    PortParam.c_oflag &= ~OPOST ;//原始输出
    PortParam.c_oflag &= ~(ONLCR | OCRNL);
    PortParam.c_cc[VTIME]  = 10;   //单位100ms
    PortParam.c_cc[VMIN] = 196;

    // fcntl(this->fd,F_SETFL,FNDELAY);//原数据模式下设置read()系统调用的模式（第三个参数为0时为阻塞模式,FNDELAY时为非阻塞模式）
    fcntl(this->fd,F_SETFL,0);//原数据模式下设置read()系统调用的模式（第三个参数为0时为阻塞模式,FNDELAY时为非阻塞模式)
    tcflush(this->fd,TCIOFLUSH);
    if(tcsetattr(this->fd,TCSANOW,&PortParam) != 0)//更新串口设置
    {
        perror("Setup serialport");
        return SETUPFAILED;
    }
    
    return SETUPSUCCEED;
}


int UART::ReadUart(unsigned char* buf, int size)
{
    int iCount = 0;

    iCount = read(this->fd, buf, size);

    return iCount;
}


int UART::WriteUart(unsigned char *buf, int size)
{
    int iCount = 0;

    iCount = write(this->fd, buf, size);

    return iCount ;
}


int UART::CloseUart()
{
    int ret = 0;
    ret = close(this->fd);
    if(ret < 0)
    {
        perror("Close uart error");
        return -1;
    }
    else
    {
        return 0;
    }
}
