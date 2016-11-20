/**
* @brief Leeds Main Window header
* @file mainwindow.h
* @date 11/06/2012
*
*/

#ifndef MAINAPP_H
#define MAINAPP_H


#include "leeds.hpp"
#include "mainwindow.h"
#include <QtGui/QApplication>
#include <QtGui/QMessageBox>
#include <QtOpenGL/QGLFormat>
#include <QtOpenGL/QGLFramebufferObject>


class MainApp : public QApplication {

Q_OBJECT

public:
	MainApp (int argc, char *argv[]);
	~MainApp(){};
};


#endif
