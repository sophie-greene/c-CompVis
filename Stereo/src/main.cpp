/**
* @brief Leeds main function
* @file main.cpp
* @date 11/05/2012
*
*/

#include "mainwindow.h"
#include <QtGui/QApplication>
#include <QtGui/QMessageBox>
#include <QtOpenGL/QGLFormat>
#include <QtOpenGL/QGLFramebufferObject>


int main(int argc, char *argv[]) {

    QApplication a(argc, argv);
    
    if (!QGLFormat::hasOpenGL() || !QGLFramebufferObject::hasOpenGLFramebufferObjects() || !QGLFormat::OpenGL_Version_4_0) {
		QMessageBox::information(0, "Leeds", "OpenGL Support too low");
		return -1;
    }    
    
 
    MainWindow w;
    w.show();
  
    QObject::connect(&a, SIGNAL(lastWindowClosed()), &w, SLOT(handleExit()));	
    
    return a.exec();
}
