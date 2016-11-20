#include "calibratedialog.h"
#include "ui_calibratedialog.h"

calibrateDialog::calibrateDialog(QWidget *parent) :
        QDialog(parent),
        ui(new Ui::calibrateDialog)
{
    ui->setupUi(this);
    cThread=new CalibrateThread();
    connect(cThread,SIGNAL(calibrationSuccess(bool)),this,SLOT(onCalibrationSuccess(bool)));
    cThread->start();
}

calibrateDialog::~calibrateDialog()
{
    delete ui;
}

void calibrateDialog::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void calibrateDialog::on_cmdCancel_clicked()
{
    this->close();
}
void calibrateDialog::onCalibrationSuccess(bool){
    ui->lblMsg->setText("Calibration performed successfully");
}
