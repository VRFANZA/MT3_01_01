#pragma once
#include <windows.h>
#include <string>

class SerialReader{
public:
    SerialReader(const std::string& portName, int baud = 115200);
    ~SerialReader();
    bool readLine(std::string& outLine);

private:
    HANDLE hSerial;

};