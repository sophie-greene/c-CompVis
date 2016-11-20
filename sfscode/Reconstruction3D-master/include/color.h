#ifndef COLOR_H
#define COLOR_H

#include <stdlib.h>
#include <iostream>
#include <sstream>


/*std::string color(std::string str, int color, bool bold = false)
{
  std::stringstream ss;
  ss << "\033[";
  if( bold ) ss << "1";
  else ss << "0";
  ss << ";" << color+30 << "m" << str << "\033[0m";
  return ss.str();
}

std::string color(double nb, int color, bool bold = false)
{
  std::stringstream ss;
  ss << "\033[";
  if( bold ) ss << "1";
  else ss << "0";
  ss << ";" << color+30 << "m" << nb << "\033[0m";
  return ss.str();
}*/

namespace color{

  enum {BLACK=0,RED,GREEN,YELLOW,BLUE,PURPLE,CYAN,GRAY};

  inline std::string color(int colorID, bool bold = false){
    std::stringstream ss;
    ss << "\033[";
    ss << bold?"1":"0";
    ss << ";" << colorID+30<< "m";
    return ss.str();
  }

  inline std::string red(){ return color(RED); }
  inline std::string Red(){ return color(RED,true); }
  inline std::string green(){ return color(GREEN); }
  inline std::string Green(){ return color(GREEN,true); }
  inline std::string yellow(){ return color(YELLOW); }
  inline std::string Yellow(){ return color(YELLOW,true); }
  inline std::string blue(){ return color(BLUE); }
  inline std::string Blue(){ return color(BLUE,true); }
  inline std::string purple(){ return color(PURPLE); }
  inline std::string Purple(){ return color(PURPLE,true); }
  inline std::string cyan(){ return color(CYAN); }
  inline std::string Cyan(){ return color(CYAN,true); }
  inline std::string gray(){ return color(GRAY); }
  inline std::string Gray(){ return color(GRAY,true); }
  inline std::string black(){ return color(BLACK);}
  inline std::string Black(){ return color(BLACK,true);}
  inline std::string reset(){
    return "\033[0m";
  }
}

#endif // COLOR_H
