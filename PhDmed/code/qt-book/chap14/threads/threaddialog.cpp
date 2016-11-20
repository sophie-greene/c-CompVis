#include <QtGui>

#include "threaddialog.h"

ThreadDialog::ThreadDialog(QWidget *parent)
    : QDialog(parent)
{
    threadA.setMessage("A");
    threadB.setMessage("B");

    threadAButton = new QPushButton(tr("Start A"));
    threadBButton = new QPushButton(tr("Start B"));
    quitButton = new QPushButton(tr("Quit"));
    quitButton->setDefault(true);

    connect(threadAButton, SIGNAL(clicked()),
            this, SLOT(startOrStopThreadA()));
    connect(threadBButton, SIGNAL(clicked()),
            this, SLOT(startOrStopThreadB()));
    connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));

    QGridLayout *mainLayout = new QGridLayout;
    int row=0;
    int col=0;
    for(int i=0;i<8;i++){
        if(i%3==0 && i!=0){
           row=row+2;
           col=0;
        }
        lblCam[i]=new QLabel;
        lbl[i]=new QLabel;

        QString s;
        s="Camera "+s.number(i);
        lbl[i]->setText(s);
        lblCam[i]->setText("dusihfiksdjhfkjsa\hnfkjsanfkfkbsdkjgfbdskjgbsdjkgbdkjsbgkjdsbgjkdsbn");
        mainLayout->addWidget(lbl[i],row,col);
        mainLayout->addWidget(lblCam[i],row+1,col);
        col=col+1;
        qDebug()<<"i="<<i<<"("<<row<<","<<col<<")";



    }
QVBoxLayout* s=new QVBoxLayout();
    s->addWidget(threadAButton);
    s->addWidget(threadBButton);
    mainLayout->addLayout(s,6,6);


    // mainLayout->addRow(lblCam[0],lblCam[1]);
    // mainLayout->addRow(lbl[3],lbl[4]);

    //  mainLayout->addWidget(threadAButton);
    // mainLayout->addWidget(threadBButton);
    // mainLayout->addWidget(quitButton);
    setLayout(mainLayout);

    setWindowTitle(tr("Threads"));
}

void ThreadDialog::startOrStopThreadA()
{
    if (threadA.isRunning()) {
        threadA.stop();
        threadAButton->setText(tr("Start A"));
    } else {
        threadA.start();
        threadAButton->setText(tr("Stop A"));
    }
}

void ThreadDialog::startOrStopThreadB()
{
    if (threadB.isRunning()) {
        threadB.stop();
        threadBButton->setText(tr("Start B"));
    } else {
        threadB.start();
        threadBButton->setText(tr("Stop B"));
    }
}

void ThreadDialog::closeEvent(QCloseEvent *event)
{
    threadA.stop();
    threadB.stop();
    threadA.wait();
    threadB.wait();
    event->accept();
}
