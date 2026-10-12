#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <termios.h>
#include <stdint.h>
#include <stdarg.h>
#include <iostream>
#define UART1        0X01
#define UART2        0X02
#define UART3        0X04
#define UART_DBG     0X08
#define UART_UDP     0X10

#define UART_ALL (UART1 | UART2 | UART3 | UART_DBG | UART_UDP)

inline void ThreadDelay(const long msecond)
{
    timeval timeOut;
    timeOut.tv_sec = 0;
    timeOut.tv_usec = msecond * 1000;
    select(0, NULL, NULL, NULL, &timeOut);
}

inline void QPrint(const char *format,...)
{
    char buffer[1024];
    va_list p;
    va_start(p, format);
    int count = vsnprintf(buffer,sizeof(buffer) - 1,format,p);
    va_end(p);

    if(count < 1) return;

    std::cout << buffer << std::flush;
}
class SerialPort
{
public:
    SerialPort();
    ~SerialPort();
    int GetFid(void);
    bool Close(void);
    bool Open(const char* port,int baud);
    bool IsOpened(void);
    int Read(uint8_t* buffer,int size,int timeout);
    int Write(const uint8_t* buffer,int size);
    int Read(const char* buffer);
    int Write(const char* buffer);
    void WriteSync(void);
    void WriteFlush(void);
    void ReadFlush(void);
    bool ResetBaud(int baud);
private:
    int fid;
    bool is_open;
    char path[128];
    int baud;
};

#endif // SERIALPORT_H
