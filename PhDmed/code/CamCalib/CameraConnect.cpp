//Camera Calibration project
//Auther Sophie M Greene
#include "CameraConnect.h"

// Configuration header file

CameraConnect::CameraConnect(int deviceNumber)
{
    this->deviceNumber=deviceNumber;
}

int CameraConnect::getDeviceNumber()
{
    return deviceNumber;
} // getDeviceNumber()
void setDeviceNumber(int deviceNumber)
{
    this->deviceNumber=deviceNumber;
}
