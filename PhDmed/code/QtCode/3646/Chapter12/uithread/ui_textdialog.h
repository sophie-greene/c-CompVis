/*

   Copyright (c) 2006-2007, Johan Thelin
   
   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
       * Redistributions of source code must retain the above copyright notice, 
         this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright notice,  
         this list of conditions and the following disclaimer in the documentation 
         and/or other materials provided with the distribution.
       * Neither the name of APress nor the names of its contributors 
         may be used to endorse or promote products derived from this software 
         without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
*/

/********************************************************************************
** Form generated from reading UI file 'textdialog.ui'
**
** Created: Tue May 21 11:28:27 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TEXTDIALOG_H
#define UI_TEXTDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QListWidget>

QT_BEGIN_NAMESPACE

class Ui_TextDialog
{
public:
    QGridLayout *gridLayout;
    QListWidget *listWidget;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *TextDialog)
    {
        if (TextDialog->objectName().isEmpty())
            TextDialog->setObjectName(QString::fromUtf8("TextDialog"));
        TextDialog->resize(400, 300);
        gridLayout = new QGridLayout(TextDialog);
#ifndef Q_OS_MAC
        gridLayout->setSpacing(6);
#endif
#ifndef Q_OS_MAC
        gridLayout->setContentsMargins(9, 9, 9, 9);
#endif
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        listWidget = new QListWidget(TextDialog);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));

        gridLayout->addWidget(listWidget, 0, 0, 1, 1);

        buttonBox = new QDialogButtonBox(TextDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close|QDialogButtonBox::NoButton|QDialogButtonBox::Reset);

        gridLayout->addWidget(buttonBox, 1, 0, 1, 1);


        retranslateUi(TextDialog);
        QObject::connect(buttonBox, SIGNAL(rejected()), TextDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(TextDialog);
    } // setupUi

    void retranslateUi(QDialog *TextDialog)
    {
        TextDialog->setWindowTitle(QApplication::translate("TextDialog", "Threading Demonstration", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TextDialog: public Ui_TextDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TEXTDIALOG_H
