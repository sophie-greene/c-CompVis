#ifndef CALIBRATEDIALOG_H
#define CALIBRATEDIALOG_H

#include <QDialog>

namespace Ui {
    class calibrateDialog;
}

class calibrateDialog : public QDialog {
    Q_OBJECT
public:
    calibrateDialog(QWidget *parent = 0);
    ~calibrateDialog();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::calibrateDialog *ui;
};

#endif // CALIBRATEDIALOG_H
