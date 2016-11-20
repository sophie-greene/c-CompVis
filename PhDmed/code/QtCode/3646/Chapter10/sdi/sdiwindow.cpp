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

#include <QApplication>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QCloseEvent>
#include <QMessageBox>

#include <QTextEdit>

#include <QFile>
#include <QTextStream>
#include <QFileDialog>

#include "sdiwindow.h"

SdiWindow::SdiWindow( QWidget *parent ) : QMainWindow( parent )
{
  setAttribute( Qt::WA_DeleteOnClose );
  setWindowTitle( tr("%1[*] - %2" ).arg(tr("unnamed")).arg(tr("SDI")) );

  docWidget = new QTextEdit( this );
  setCentralWidget( docWidget );

  connect( docWidget->document(), SIGNAL(modificationChanged(bool)), this, SLOT(setWindowModified(bool)) );
  
  createActions();
  createMenus();
  createToolbars();
  statusBar()->showMessage( tr("Done") );
}
  
void SdiWindow::closeEvent( QCloseEvent *event )
{
  if( isSafeToClose() )
    event->accept();
  else
    event->ignore();
}
  
void SdiWindow::fileNew()
{
  (new SdiWindow())->show();
}

void SdiWindow::helpAbout()
{
  QMessageBox::about( this, tr("About SDI"), tr("A single document interface application.") );
}

void SdiWindow::createActions()
{
  newAction = new QAction( QIcon(":/images/new.png"), tr("&New"), this );
  newAction->setShortcut( tr("Ctrl+N") );
  newAction->setStatusTip( tr("Create a new document") );
  connect( newAction, SIGNAL(triggered()), this, SLOT(fileNew()) );

  openAction = new QAction( tr("&Open"), this );
  openAction->setShortcut( tr("Ctrl+O") );
  openAction->setStatusTip( tr("Open a document") );
  connect( openAction, SIGNAL(triggered()), this, SLOT(fileOpen()) );
 
  saveAction = new QAction( tr("&Save"), this );
  saveAction->setShortcut( tr("Ctrl+S") );
  saveAction->setStatusTip( tr("Save the document") );
  connect( saveAction, SIGNAL(triggered()), this, SLOT(fileSave()) );
 
  saveAsAction = new QAction( tr("Save &As"), this );
  saveAsAction->setStatusTip( tr("Save the document as") );
  connect( saveAsAction, SIGNAL(triggered()), this, SLOT(fileSaveAs()) );
 
  closeAction = new QAction( tr("&Close"), this );
  closeAction->setShortcut( tr("Ctrl+W") );
  closeAction->setStatusTip( tr("Close this document") );
  connect( closeAction, SIGNAL(triggered()), this, SLOT(close()) );

  exitAction = new QAction( tr("E&xit"), this );
  exitAction->setShortcut( tr("Ctrl+Q") );
  exitAction->setStatusTip( tr("Quit the application") );
  connect( exitAction, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()) );
  

  cutAction = new QAction( QIcon(":/images/cut.png"), tr("Cu&t"), this );
  cutAction->setShortcut( tr("Ctrl+X") );
  cutAction->setStatusTip( tr("Cut") );
  cutAction->setEnabled(false);
  connect( docWidget, SIGNAL(copyAvailable(bool)), cutAction, SLOT(setEnabled(bool)) );
  connect( cutAction, SIGNAL(triggered()), docWidget, SLOT(cut()) );
  
  copyAction = new QAction( QIcon(":/images/copy.png"), tr("&Copy"), this );
  copyAction->setShortcut( tr("Ctrl+C") );
  copyAction->setStatusTip( tr("Copy") );
  copyAction->setEnabled(false);
  connect( docWidget, SIGNAL(copyAvailable(bool)), copyAction, SLOT(setEnabled(bool)) );
  connect( copyAction, SIGNAL(triggered()), docWidget, SLOT(copy()) );


  pasteAction = new QAction( QIcon(":/images/paste.png"), tr("&Paste"), this );
  pasteAction->setShortcut( tr("Ctrl+V") );
  pasteAction->setStatusTip( tr("Paste") );
  connect( pasteAction, SIGNAL(triggered()), docWidget, SLOT(paste()) );
  

  aboutAction = new QAction( tr("&About"), this );
  aboutAction->setStatusTip( tr("About this application") );
  connect( aboutAction, SIGNAL(triggered()), this, SLOT(helpAbout()) );

  aboutQtAction = new QAction( tr("About &Qt"), this );
  aboutQtAction->setStatusTip( tr("About the Qt toolkit") );
  connect( aboutQtAction, SIGNAL(triggered()), qApp, SLOT(aboutQt()) );
}

void SdiWindow::createMenus()
{
  QMenu *menu;
  
  menu = menuBar()->addMenu( tr("&File") );
  menu->addAction( newAction );
  menu->addSeparator();
  menu->addAction( openAction );
  menu->addAction( saveAction );
  menu->addAction( saveAsAction );
  menu->addSeparator();
  menu->addAction( closeAction );
  menu->addSeparator();
  menu->addAction( exitAction );
  
  menu = menuBar()->addMenu( tr("&Edit") );
  menu->addAction( cutAction );
  menu->addAction( copyAction );
  menu->addAction( pasteAction );
  
  menu = menuBar()->addMenu( tr("&Help") );
  menu->addAction( aboutAction );
  menu->addAction( aboutQtAction );
}

void SdiWindow::createToolbars()
{
  QToolBar *toolbar;
  
  toolbar = addToolBar( tr("File") );
  toolbar->addAction( newAction );
  
  toolbar = addToolBar( tr("Edit") );
  toolbar->addAction( cutAction );
  toolbar->addAction( copyAction );
  toolbar->addAction( pasteAction );
}



bool SdiWindow::isSafeToClose()
{
  if( isWindowModified() ) 
  {
    switch( QMessageBox::warning( this, tr("SDI"), 
      tr("The document has unsaved changes.\n"
         "Do you want to save it before it is closed?"), 
          QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel ) )
    {
    case QMessageBox::Cancel:
      return false;
    case QMessageBox::Save:
      return fileSave();
    default:
      return true;
    }
  }

  return true;
}

void SdiWindow::fileOpen()
{
  QString filename = QFileDialog::getOpenFileName( this );
  if( filename.isEmpty() )
    return;

  if( currentFilename.isEmpty() && !docWidget->document()->isModified() )
    loadFile( filename );
  else
  {    
    SdiWindow *window = new SdiWindow();
    window->loadFile( filename );
    window->show();
  }
}

bool SdiWindow::fileSave()
{
  if( currentFilename.isEmpty() )
    return fileSaveAs();
  else
    return saveFile( currentFilename );  
}

bool SdiWindow::fileSaveAs()
{
  QString filename = QFileDialog::getSaveFileName( this, tr("Save As"), currentFilename );
  if( filename.isEmpty() )
    return false;
  
  return saveFile( filename );
}

bool SdiWindow::saveFile( QString filename )
{
  QFile file( filename );
  if( !file.open( QIODevice::WriteOnly | QIODevice::Text ) )
  {
    QMessageBox::warning( this, tr("SDI"), tr("Failed to save file.") );
    return false;
  }
  
  QTextStream stream( &file );
  stream << docWidget->toPlainText();
  
  currentFilename = filename;
  docWidget->document()->setModified( false );
  setWindowTitle( tr("%1[*] - %2" ).arg(filename).arg(tr("SDI")) );
  
  return true;
}

void SdiWindow::loadFile( QString filename )
{
  QFile file( filename );
  if( !file.open( QIODevice::ReadOnly | QIODevice::Text ) )
  {
    QMessageBox::warning( this, tr("SDI"), tr("Failed to open file.") );
    return;
  }
  
  QTextStream stream( &file );
  docWidget->setPlainText( stream.readAll() );

  currentFilename = filename;
  docWidget->document()->setModified( false );  
  setWindowTitle( tr("%1[*] - %2" ).arg(filename).arg(tr("SDI")) );
}
