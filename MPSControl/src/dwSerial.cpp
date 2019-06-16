extern "C" {
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
}
#include <iostream>
using namespace std;

#include "../include/dwSerial.h"

dwSerial::dwSerial()
{
}

dwSerial::dwSerial(string deviceName, int baud)
{
    handle=-1;
    Open(deviceName,baud);
}

dwSerial::~dwSerial()
{
    if(handle >=0)
        Close();
}

void dwSerial::Close(void)
{
    if(handle >=0)
        close(handle);
    handle = -1;
}


bool dwSerial::Open(string deviceName, int baud)
{
    struct termios tio;
    struct termios2 tio2;
    this->deviceName=deviceName;
    this->baud=baud;
    handle  = open(this->deviceName.c_str(),O_RDWR | O_NOCTTY /* | O_NONBLOCK */);

    if(handle <0)
        return false;
    tio.c_cflag =  CS8 | CLOCAL | CREAD;
    tio.c_oflag = 0;
    tio.c_lflag = 0;       //ICANON;
    tio.c_cc[VMIN]=32;
    tio.c_cc[VTIME]=1;     // time out every .1 sec
    ioctl(handle,TCSETS,&tio);

    ioctl(handle,TCGETS2,&tio2);
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;
    tio2.c_ispeed = baud;
    tio2.c_ospeed = baud;
    ioctl(handle,TCSETS2,&tio2);

//   flush buffer
    ioctl(handle,TCFLSH,TCIOFLUSH);

    return true;
}

bool dwSerial::IsOpen(void)
{
    return( handle >=0);
}

bool dwSerial::Send( unsigned char  * data,int len)
{
    if(!IsOpen())
        return false;
    int rlen= write(handle,data,len);
    return(rlen == len);
}

bool dwSerial::Send( unsigned char value)
{
    if(!IsOpen())
        return false;
    int rlen= write(handle,&value,1);
    return(rlen == 1);
}

bool dwSerial::Send(std::string value)
{
    if(!IsOpen())
        return false;
    unsigned int rlen= write(handle,value.c_str(),value.size());
    return(rlen == value.size());
}

bool dwSerial::SenddwCommand(std::string cmd)
{
    if(!IsOpen())
        return false;
    unsigned int rlen= write(handle,cmd.c_str(),cmd.size());
    printf("send %s\n", cmd.c_str());
    return(rlen == cmd.size());
}

int  dwSerial::Receive( unsigned char  * data, int len)
{
    if(!IsOpen())
        return -1;

    // this is a blocking receives
    int lenRCV=0;
    while(lenRCV < len)
    {
        int rlen = read(handle,&data[lenRCV],len - lenRCV);
        lenRCV+=rlen;
        //printf("%s", data);
        break;
    }
    return  lenRCV;
}


unsigned int dwSerial::ReceiveLocation(unsigned char * line, int maxLen)
{
    int ret = 0;
    if(!IsOpen())
        return -1;

    // this is a blocking receives
    int lenRCV=0;
    bool done = false;
    int bytelen;
    this->NumberByteRcv(bytelen);
    if (bytelen > maxLen)
    {
        printf("flush\n");

        ioctl(handle,TCFLSH,TCIOFLUSH);

        return maxLen + 1; // indicate it was flushed
    }
    else if(bytelen == 0)
    {
        pthread_yield();
        return 0;
    }
    // printf("\nrcvd: %d\n", bytelen);
    int state = 0;
    while (!done)
    {
        lenRCV = read(handle,&line[ret],1);
        switch (lenRCV)
        {
        case -1:
            printf ("error in readline\n");
            done = true;
            break;
        case 0:
            printf("EOF in read line\n");
            done = true;
            break;
        default:
            switch(line[ret])
            {
            case 'P':
                if (state == 3)
                {
                    state = 4;
                }
                state = 1;
                break;
            case 'O':
                state = 2;
                break;
            case 'S':
                state = 3;
                break;
            }
            if (line[ret] == '\n' || line[ret] == '\r') done = true;
            ret++;
            if (ret == maxLen)
            {
                printf("filled to the end of the buffer state:%d\n", state);
                done = true;
                //ret = 0;
            }
            else
            {
                //pthread_yield();
            }
        }

    }
    ioctl(handle,TCFLSH,TCIOFLUSH);

     return ret;
}

bool dwSerial::NumberByteRcv(int &bytelen)
{
    if(!IsOpen())
        return false;
    ioctl(handle, FIONREAD, &bytelen);
    return true;
}
