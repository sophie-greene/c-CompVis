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
** Form generated from reading UI file 'imagedialog.ui'
**
** Created: Tue May 21 11:30:06 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGEDIALOG_H
#define UI_IMAGEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ImageDialog
{
public:
    QGridLayout *gridLayout;
    QLabel *imageLabel;
    QVBoxLayout *vboxLayout;
    QListWidget *tagList;
    QHBoxLayout *hboxLayout;
    QLabel *label;
    QLabel *imagesLabel;
    QHBoxLayout *hboxLayout1;
    QPushButton *previousButton;
    QPushButton *nextButton;
    QSpacerItem *spacerItem;
    QPushButton *addTagButton;
    QPushButton *addImageButton;

    void setupUi(QDialog *ImageDialog)
    {
        if (ImageDialog->objectName().isEmpty())
            ImageDialog->setObjectName(QString::fromUtf8("ImageDialog"));
        ImageDialog->resize(730, 537);
        gridLayout = new QGridLayout(ImageDialog);
#ifndef Q_OS_MAC
        gridLayout->setSpacing(6);
#endif
#ifndef Q_OS_MAC
        gridLayout->setContentsMargins(9, 9, 9, 9);
#endif
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        imageLabel = new QLabel(ImageDialog);
        imageLabel->setObjectName(QString::fromUtf8("imageLabel"));
        imageLabel->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(imageLabel, 0, 0, 1, 1);

        vboxLayout = new QVBoxLayout();
#ifndef Q_OS_MAC
        vboxLayout->setSpacing(6);
#endif
        vboxLayout->setContentsMargins(0, 0, 0, 0);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        tagList = new QListWidget(ImageDialog);
        tagList->setObjectName(QString::fromUtf8("tagList"));
        tagList->setMaximumSize(QSize(150, 16777215));
        tagList->setSelectionMode(QAbstractItemView::MultiSelection);

        vboxLayout->addWidget(tagList);

        hboxLayout = new QHBoxLayout();
#ifndef Q_OS_MAC
        hboxLayout->setSpacing(6);
#endif
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        label = new QLabel(ImageDialog);
        label->setObjectName(QString::fromUtf8("label"));

        hboxLayout->addWidget(label);

        imagesLabel = new QLabel(ImageDialog);
        imagesLabel->setObjectName(QString::fromUtf8("imagesLabel"));
        imagesLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        hboxLayout->addWidget(imagesLabel);


        vboxLayout->addLayout(hboxLayout);


        gridLayout->addLayout(vboxLayout, 0, 1, 2, 1);

        hboxLayout1 = new QHBoxLayout();
#ifndef Q_OS_MAC
        hboxLayout1->setSpacing(6);
#endif
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
        previousButton = new QPushButton(ImageDialog);
        previousButton->setObjectName(QString::fromUtf8("previousButton"));

        hboxLayout1->addWidget(previousButton);

        nextButton = new QPushButton(ImageDialog);
        nextButton->setObjectName(QString::fromUtf8("nextButton"));

        hboxLayout1->addWidget(nextButton);

        spacerItem = new QSpacerItem(161, 27, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacerItem);

        addTagButton = new QPushButton(ImageDialog);
        addTagButton->setObjectName(QString::fromUtf8("addTagButton"));

        hboxLayout1->addWidget(addTagButton);

        addImageButton = new QPushButton(ImageDialog);
        addImageButton->setObjectName(QString::fromUtf8("addImageButton"));

        hboxLayout1->addWidget(addImageButton);


        gridLayout->addLayout(hboxLayout1, 1, 0, 1, 1);


        retranslateUi(ImageDialog);

        QMetaObject::connectSlotsByName(ImageDialog);
    } // setupUi

    void retranslateUi(QDialog *ImageDialog)
    {
        ImageDialog->setWindowTitle(QApplication::translate("ImageDialog", "Image Book", 0, QApplication::UnicodeUTF8));
        imageLabel->setText(QApplication::translate("ImageDialog", "No Image", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ImageDialog", "Images:", 0, QApplication::UnicodeUTF8));
        imagesLabel->setText(QApplication::translate("ImageDialog", "#", 0, QApplication::UnicodeUTF8));
        previousButton->setText(QApplication::translate("ImageDialog", "Previous", 0, QApplication::UnicodeUTF8));
        nextButton->setText(QApplication::translate("ImageDialog", "Next", 0, QApplication::UnicodeUTF8));
        addTagButton->setText(QApplication::translate("ImageDialog", "Add Tag", 0, QApplication::UnicodeUTF8));
        addImageButton->setText(QApplication::translate("ImageDialog", "Add Image", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ImageDialog: public Ui_ImageDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGEDIALOG_H
