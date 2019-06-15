
#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <string>


class  dwSerial
{

public:

  int handle;
  std::string  deviceName;
  int baud;

  dwSerial();
  dwSerial(std::string deviceName, int baud);
  ~dwSerial();

  bool Send( unsigned char  * data,int len);
  bool Send(unsigned char value);
  bool Send( std::string value);
  bool SenddwCommand(std::string cmd);
  int Receive( unsigned char  * data, int len);
  int ReadLine( unsigned char * line, int maxLen);
  int ReceiveLocation(unsigned char * location, int maxLen);
  bool IsOpen(void);
  void Close(void);
  bool Open(std::string deviceName, int baud);
  bool NumberByteRcv(int &bytelen);
};

#endif //_SERIAL_H_
