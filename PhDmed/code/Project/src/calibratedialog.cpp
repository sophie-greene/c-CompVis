#include "calibratedialog.h"
#include "ui_calibratedialog.h"

calibrateDialog::calibrateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::calibrateDialog)
{
    ui->setupUi(this);
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
