#include "capture.h"

Capture::Capture()
{
    this->setWidth(320);
    this->setHeight(240);
    this->setBrightness(224);
    this->setContrast(196);
    this->setSaturation(64);
    this->setGain(128);
}
bool openCap(){
    return true;
}

bool closeCap(){
    return true;
}

Mat captureFrame(){
    //return 0;
}

int getWidth(){
    //return width;
    return 0;
}

int getHeight(){
    return 0;

}

int getGain(){
    return 0;
}

int getBrightness(){
return 0;
}

int getContrast(){
return 0;
}

int getSaturation(){
return 0;
}

void setWidth(int){

}

void setHeight(int){

}

void setGain(int){

}

void setBrightness(int){

}

void setContrast(int){

}

void setSaturation(int){

}
