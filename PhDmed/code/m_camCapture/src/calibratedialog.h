#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#include <QDialog>
#include "src/calibratethread.h"
namespace Ui {
    class calibrateDialog;
}

class calibrateDialog : public QDialog {
    Q_OBJECT
public:
    calibrateDialog(QWidget *parent = 0);
    ~calibrateDialog();
   CalibrateThread *cThread;

protected:
    void changeEvent(QEvent *e);

private:
    Ui::calibrateDialog *ui;

private slots:
    void on_cmdCancel_clicked();
public slots:
    void onCalibrationSuccess(bool);
};

#endif // CALIBRATEDIALOG_H
