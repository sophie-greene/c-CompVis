/**
* @brief GLFW Bit
* @file visual_app.cpp
* @date 07/08/2012
*
*/

#include "visualapp.hpp"

using namespace s9;
using namespace boost;
using namespace std;


void WindowApp::fireEvent(Event e){
  BOOST_FOREACH(shared_ptr<WindowResponder> p, _listeners){
    p->processEvent(e);
  }
}

void WindowApp::fireEvent(MouseEvent e){
   BOOST_FOREACH(shared_ptr<WindowResponder> p, _listeners){
    p->processEvent(e);
  }
}

void WindowApp::fireEvent(ResizeEvent e){
   BOOST_FOREACH(shared_ptr<WindowResponder> p, _listeners){
    p->processEvent(e);
  }
}

void WindowApp::fireEvent(KeyboardEvent e){
   BOOST_FOREACH(shared_ptr<WindowResponder> p, _listeners){
    p->processEvent(e);
  }

}
