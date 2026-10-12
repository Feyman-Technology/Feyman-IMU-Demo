#include "serialport.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdio.h>
#include <iostream>

SerialPort::SerialPort()
{
    is_open = false;
    memset(path,0,sizeof(path));
}

SerialPort::~SerialPort()
{
    Close();
}

int SerialPort::GetFid()
{
    if(is_open == true) return fid;
    else return -1;
}

bool SerialPort::Close()
{
    is_open = false;
    if(close(fid) < 0) return false;
    else return true;
}

bool SerialPort::IsOpened()
{
    return is_open;
}

bool SerialPort::Open(const char *port,int baud_set)
{
    if(strlen(port) > sizeof(path) - 1) return false;
    memcpy(path,port,strlen(port));
    baud = baud_set;
    uint32_t baud_enum;

    switch (baud) {
    case 9600:
        baud_enum = B9600;
        break;
    case 19200:
        baud_enum = B19200;
        break;
    case 38400:
        baud_enum = B38400;
        break;
    case 57600:
        baud_enum = B57600;
        break;
    case 115200:
        baud_enum = B115200;
        break;
    case 230400:
        baud_enum = B230400;
        break;
    case 460800:
        baud_enum = B460800;
        break;
    case 921600:
        baud_enum = B921600;
        break;
    default:
        return false;
    }

    if((fid = open(port,  O_RDWR|O_NOCTTY)) < 0) return false;

    struct termios ios;
    tcgetattr(fid, &ios);

    cfsetispeed(&ios, baud_enum); /* Set Read  Speed as 9600                       */
    cfsetospeed(&ios, baud_enum); /* Set Write Speed as 9600                       */

    ios.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    ios.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    ios.c_cflag &= ~CSIZE;	  /* Clears the mask for setting the data size             */
    ios.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    ios.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    ios.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

    ios.c_iflag &= ~(IXON | IXOFF | IXANY);
    ios.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    ios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Disable XON/XOFF flow control both i/p and o/p */
    ios.c_oflag &= ~OPOST;                           /*No Output Processing   raw  format  output*/

    if((tcsetattr(fid,TCSANOW,&ios)) != 0)
    {
        close(fid);
        return false;
    }

    tcflush(fid,TCIOFLUSH);
    is_open = true;
    return true;
}

bool SerialPort::ResetBaud(int baud_set)
{
    if(baud == baud_set) return is_open;

    char buffer[sizeof(path)];
    memcpy(buffer,path,sizeof(path));

    if(strlen(buffer) <= 0) return false;

    Close();

    return Open(buffer,baud_set);
}

void SerialPort::ReadFlush()
{
    if(is_open == false) return;
    tcflush(fid,TCIFLUSH);
}

void SerialPort::WriteFlush()
{
    if(is_open == false) return;
    tcflush(fid,TCOFLUSH);
}

int SerialPort::Read(uint8_t *buffer, int size, int timeout)
{
    if(is_open == false) return -1;
    if(size <= 0) return -1;

    fd_set readfds;
    struct timeval tv;
    int count = 0;

    while(1)
    {
        tv.tv_sec = 0;
        tv.tv_usec = timeout * 1000;
        FD_ZERO(&readfds);
        FD_SET(fid, &readfds);

        select(fid + 1, &readfds, NULL, NULL, &tv);
        if(FD_ISSET(fid, &readfds) == 0) return count;

        int tmp = read(fid,buffer + count,size - count);
        if(tmp <= 0) break;

        count += tmp;
        if(count >= size) break;
    }

    return count;
}

int SerialPort::Write(const uint8_t *buffer, int size)
{
    if(is_open == false) return -1;
    return write(fid,buffer,size);
}

void SerialPort::WriteSync()
{
    if(is_open == false) return;
    tcdrain(fid);
}

int SerialPort::Read(const char *buffer)
{
    if(is_open == false) return -1;
    return read(fid,(uint8_t*)buffer,strlen(buffer));
}

int SerialPort::Write(const char *buffer)
{
    if(is_open == false) return -1;
    return write(fid,(uint8_t*)buffer,strlen(buffer));
}
