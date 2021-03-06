/*
 * Copyright (c) 2006-2007, Johan Thelin
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright notice, 
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice,  
 *       this list of conditions and the following disclaimer in the documentation 
 *       and/or other materials provided with the distribution.
 *     * Neither the name of APress nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MDIWINDOW_H
#define MDIWINDOW_H

#include <QMainWindow>

class QAction;
class QWorkspace;
class QSignalMapper;
class QMenu;

class DocumentWindow;

class MdiWindow : public QMainWindow
{
  Q_OBJECT
  
public:
  MdiWindow( QWidget *parent = 0 );
  
protected:
  void closeEvent( QCloseEvent *event );
  
private slots:
  void fileNew();
  
  void editCut();
  void editCopy();
  void editPaste();
  
  void helpAbout();
  
  void enableActions();
  void updateWindowList();

private:
  void createActions();
  void createMenus();
  void createToolbars();

  DocumentWindow *activeDocument();

  QWorkspace *workspace;
  QSignalMapper *mapper;
  
  QAction *newAction;
  QAction *closeAction;
  QAction *exitAction;
  
  QAction *cutAction;
  QAction *copyAction;
  QAction *pasteAction;
  
  QAction *tileAction;
  QAction *cascadeAction;
  QAction *nextAction;
  QAction *previousAction;
  QAction *separatorAction;
  
  QAction *aboutAction;
  QAction *aboutQtAction;
  
  QMenu *windowMenu;
};
  
#endif // MDIWINDOW_H
