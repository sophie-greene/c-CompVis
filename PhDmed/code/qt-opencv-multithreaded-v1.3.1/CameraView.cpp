/************************************************************************/
/* qt-opencv-multithreaded:                                             */
/* A multithreaded OpenCV application using the Qt framework.           */
/*                                                                      */
/* CameraView.cpp                                                       */
/*                                                                      */
/* Nick D'Ademo <nickdademo@gmail.com>                                  */
/*                                                                      */
/* Copyright (c) 2012-2013 Nick D'Ademo                                 */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files (the "Software"), to deal in the Software without restriction, */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#include "CameraView.h"
#include "ui_CameraView.h"

CameraView::CameraView(QWidget *parent, int deviceNumber, SharedImageBuffer *sharedImageBuffer) :
    QMainWindow(parent),
    ui(new Ui::CameraView),
    sharedImageBuffer(sharedImageBuffer)
{
    // Setup UI
    ui->setupUi(this);
    // Save Device Number
    this->deviceNumber=deviceNumber;
    // Initialize internal flag
    isCameraConnected=false;
    // Set initial GUI state
    // Menu (disabled)
    ui->imageProcessingMenu->setDisabled(true);
    QList<QAction*> actions = ui->imageProcessingMenu->actions();
    for(int i=0; i<actions.size(); i++)
        actions.at(i)->setChecked(false);
    // Window
    ui->frameLabel->setText("No camera connected.");
    ui->imageBufferBar->setValue(0);
    ui->imageBufferLabel->setText("[000/000]");
    ui->captureRateLabel->setText("");
    ui->processingRateLabel->setText("");
    ui->deviceNumberLabel->setText("");
    ui->cameraResolutionLabel->setText("");
    ui->roiLabel->setText("");
    ui->mouseCursorPosLabel->setText("");
    ui->clearImageBufferButton->setDisabled(true);
    // Initialize ImageProcessingFlags structure
    imageProcessingFlags.grayscaleOn=false;
    imageProcessingFlags.smoothOn=false;
    imageProcessingFlags.dilateOn=false;
    imageProcessingFlags.erodeOn=false;
    imageProcessingFlags.flipOn=false;
    imageProcessingFlags.cannyOn=false;
    // Connect signals/slots
    connect(ui->frameLabel, SIGNAL(onMouseMoveEvent()), this, SLOT(updateMouseCursorPosLabel()));
    connect(ui->actionGrayscale, SIGNAL(toggled(bool)), this, SLOT(setGrayscale(bool)));
    connect(ui->actionSmooth, SIGNAL(toggled(bool)), this, SLOT(setSmooth(bool)));
    connect(ui->actionDilate, SIGNAL(toggled(bool)), this, SLOT(setDilate(bool)));
    connect(ui->actionErode, SIGNAL(toggled(bool)), this, SLOT(setErode(bool)));
    connect(ui->actionFlip, SIGNAL(toggled(bool)), this, SLOT(setFlip(bool)));
    connect(ui->actionCanny, SIGNAL(toggled(bool)), this, SLOT(setCanny(bool)));
    connect(ui->imageProcessingSettingsAction, SIGNAL(triggered()), this, SLOT(setImageProcessingSettings()));
    connect(ui->clearImageBufferButton, SIGNAL(released()), this, SLOT(clearImageBuffer()));
    connect(ui->actionScaleToFitFrame, SIGNAL(toggled(bool)), this, SLOT(setScaledContents(bool)));
    // Register
    qRegisterMetaType<struct ThreadStatisticsData>("ThreadStatisticsData");
}

CameraView::~CameraView()
{
    if(isCameraConnected)
    {
        // Stop processing thread
        if(processingThread->isRunning())
            stopProcessingThread();
        // Stop capture thread
        if(captureThread->isRunning())
            stopCaptureThread();
        // Remove from shared buffer
        sharedImageBuffer->removeByDeviceNumber(deviceNumber);
        // Disconnect camera
        if(captureThread->disconnectCamera())
            qDebug() << "[" << deviceNumber << "] Camera successfully disconnected.";
        else
            qDebug() << "[" << deviceNumber << "] WARNING: Camera already disconnected.";
    }
    // Delete UI
    delete ui;
}

bool CameraView::connectToCamera(bool dropFrameIfBufferFull, int capThreadPrio, int procThreadPrio, bool enableFrameProcessing)
{
    // Set frame label text
    if(sharedImageBuffer->isSyncEnabledForDeviceNumber(deviceNumber))
        ui->frameLabel->setText("Camera connected. Waiting...");
    else
        ui->frameLabel->setText("Connecting to camera...");

    // Create capture thread
    captureThread = new CaptureThread(sharedImageBuffer, deviceNumber, dropFrameIfBufferFull);
    // Attempt to connect to camera
    if(captureThread->connectToCamera())
    {
        // Create processing thread
        processingThread = new ProcessingThread(sharedImageBuffer, deviceNumber);
        // Create image processing settings dialog
        imageProcessingSettingsDialog = new ImageProcessingSettingsDialog(this);
        // Setup signal/slot connections
        connect(processingThread, SIGNAL(newFrame(QImage)), this, SLOT(updateFrame(QImage)));
        connect(processingThread, SIGNAL(updateStatisticsInGUI(struct ThreadStatisticsData)), this, SLOT(updateProcessingThreadStats(struct ThreadStatisticsData)));
        connect(captureThread, SIGNAL(updateStatisticsInGUI(struct ThreadStatisticsData)), this, SLOT(updateCaptureThreadStats(struct ThreadStatisticsData)));
        connect(imageProcessingSettingsDialog, SIGNAL(newImageProcessingSettings(struct ImageProcessingSettings)), processingThread, SLOT(updateImageProcessingSettings(struct ImageProcessingSettings)));
        connect(this, SIGNAL(newImageProcessingFlags(struct ImageProcessingFlags)), processingThread, SLOT(updateImageProcessingFlags(struct ImageProcessingFlags)));
        connect(this, SIGNAL(setROI(QRect)), processingThread, SLOT(setROI(QRect)));
        // Only enable ROI setting/resetting if frame processing is enabled
        if(enableFrameProcessing)
            connect(ui->frameLabel, SIGNAL(newMouseData(struct MouseData)), this, SLOT(newMouseData(struct MouseData)));
        // Set initial data in processing thread
        emit setROI(QRect(0, 0, captureThread->getInputSourceWidth(), captureThread->getInputSourceHeight()));
        emit newImageProcessingFlags(imageProcessingFlags);
        imageProcessingSettingsDialog->updateStoredSettingsFromDialog();

        // Start capturing frames from camera
        captureThread->start((QThread::Priority)capThreadPrio);
        // Start processing captured frames (if enabled)
        if(enableFrameProcessing)
            processingThread->start((QThread::Priority)procThreadPrio);

        // Setup imageBufferBar with minimum and maximum values
        ui->imageBufferBar->setMinimum(0);
        ui->imageBufferBar->setMaximum(sharedImageBuffer->getByDeviceNumber(deviceNumber)->getMaxSize());
        // Enable/disable appropriate GUI items
        ui->imageProcessingMenu->setEnabled(enableFrameProcessing);
        ui->imageProcessingSettingsAction->setEnabled(enableFrameProcessing);
        // Enable "Clear Image Buffer" push button
        ui->clearImageBufferButton->setEnabled(true);
        // Set text in labels
        ui->deviceNumberLabel->setNum(deviceNumber);
        ui->cameraResolutionLabel->setText(QString::number(captureThread->getInputSourceWidth())+QString("x")+QString::number(captureThread->getInputSourceHeight()));
        // Set internal flag and return
        isCameraConnected=true;
        // Set frame label text
        if(!enableFrameProcessing)
            ui->frameLabel->setText("Frame processing disabled.");
        return true;
    }
    // Failed to connect to camera
    else
        return false;
}

void CameraView::stopCaptureThread()
{
    qDebug() << "[" << deviceNumber << "] About to stop capture thread...";
    captureThread->stop();
    sharedImageBuffer->wakeAll(); // This allows the thread to be stopped if it is in a wait-state
    // Take one frame off a FULL queue to allow the capture thread to finish
    if(sharedImageBuffer->getByDeviceNumber(deviceNumber)->isFull())
        sharedImageBuffer->getByDeviceNumber(deviceNumber)->getFrame();
    captureThread->wait();
    qDebug() << "[" << deviceNumber << "] Capture thread successfully stopped.";
}

void CameraView::stopProcessingThread()
{
    qDebug() << "[" << deviceNumber << "] About to stop processing thread...";
    processingThread->stop();
    processingThread->wait();
    qDebug() << "[" << deviceNumber << "] Processing thread successfully stopped.";
}

void CameraView::updateCaptureThreadStats(struct ThreadStatisticsData statData)
{
    // Show [number of images in buffer / image buffer size] in imageBufferLabel
    ui->imageBufferLabel->setText(QString("[")+QString::number(sharedImageBuffer->getByDeviceNumber(deviceNumber)->getSize())+
                                  QString("/")+QString::number(sharedImageBuffer->getByDeviceNumber(deviceNumber)->getMaxSize())+QString("]"));
    // Show percentage of image bufffer full in imageBufferBar
    ui->imageBufferBar->setValue(sharedImageBuffer->getByDeviceNumber(deviceNumber)->getSize());

    // Show processing rate in captureRateLabel
    ui->captureRateLabel->setText(QString::number(statData.averageFPS)+" fps");
    // Show number of frames captured in nFramesCapturedLabel
    ui->nFramesCapturedLabel->setText(QString("[") + QString::number(statData.nFramesProcessed) + QString("]"));
}

void CameraView::updateProcessingThreadStats(struct ThreadStatisticsData statData)
{
    // Show processing rate in processingRateLabel
    ui->processingRateLabel->setText(QString::number(statData.averageFPS)+" fps");
    // Show ROI information in roiLabel
    ui->roiLabel->setText(QString("(")+QString::number(processingThread->getCurrentROI().x())+QString(",")+
                          QString::number(processingThread->getCurrentROI().y())+QString(") ")+
                          QString::number(processingThread->getCurrentROI().width())+
                          QString("x")+QString::number(processingThread->getCurrentROI().height()));
    // Show number of frames processed in nFramesProcessedLabel
    ui->nFramesProcessedLabel->setText(QString("[") + QString::number(statData.nFramesProcessed) + QString("]"));
}

void CameraView::updateFrame(const QImage &frame)
{
    // Display frame
    ui->frameLabel->setPixmap(QPixmap::fromImage(frame));
}

void CameraView::clearImageBuffer()
{
    if(sharedImageBuffer->getByDeviceNumber(deviceNumber)->clear())
        qDebug() << "[" << deviceNumber << "] Image buffer successfully cleared.";
    else
        qDebug() << "[" << deviceNumber << "] WARNING: Could not clear image buffer.";
}

void CameraView::setImageProcessingSettings()
{
    // Prompt user:
    // If user presses OK button on dialog, update image processing settings
    if(imageProcessingSettingsDialog->exec()==QDialog::Accepted)
        imageProcessingSettingsDialog->updateStoredSettingsFromDialog();
    // Else, restore dialog state
    else
        imageProcessingSettingsDialog->updateDialogSettingsFromStored();
}

void CameraView::updateMouseCursorPosLabel()
{
    // Update mouse cursor position in mouseCursorPosLabel
    ui->mouseCursorPosLabel->setText(QString("(")+QString::number(ui->frameLabel->getMouseCursorPos().x())+
                                     QString(",")+QString::number(ui->frameLabel->getMouseCursorPos().y())+
                                     QString(")"));

    // Show ROI-adjusted cursor position if camera is connected
    if(isCameraConnected)
        ui->mouseCursorPosLabel->setText(ui->mouseCursorPosLabel->text()+
                                         QString(" [")+QString::number(ui->frameLabel->getMouseCursorPos().x()-(ui->frameLabel->width()-processingThread->getCurrentROI().width())/2)+
                                         QString(",")+QString::number(ui->frameLabel->getMouseCursorPos().y()-(ui->frameLabel->height()-processingThread->getCurrentROI().height())/2)+
                                         QString("]"));

}

void CameraView::newMouseData(struct MouseData mouseData)
{
    // Local variable(s)
    int x_temp, y_temp, width_temp, height_temp;
    QRect selectionBox;

    // Set ROI
    if(mouseData.leftButtonRelease)
    {
        // Copy box dimensions from mouseData to taskData
        selectionBox.setX(mouseData.selectionBox.x()-((ui->frameLabel->width()-captureThread->getInputSourceWidth()))/2);
        selectionBox.setY(mouseData.selectionBox.y()-((ui->frameLabel->height()-captureThread->getInputSourceHeight()))/2);
        selectionBox.setWidth(mouseData.selectionBox.width());
        selectionBox.setHeight(mouseData.selectionBox.height());
        // Check if selection box has NON-ZERO dimensions
        if((selectionBox.width()!=0)&&((selectionBox.height())!=0))
        {
            // Selection box can also be drawn from bottom-right to top-left corner
            if(selectionBox.width()<0)
            {
                x_temp=selectionBox.x();
                width_temp=selectionBox.width();
                selectionBox.setX(x_temp+selectionBox.width());
                selectionBox.setWidth(width_temp*-1);
            }
            if(selectionBox.height()<0)
            {
                y_temp=selectionBox.y();
                height_temp=selectionBox.height();
                selectionBox.setY(y_temp+selectionBox.height());
                selectionBox.setHeight(height_temp*-1);
            }

            // Check if selection box is not outside window
            if((selectionBox.x()<0)||(selectionBox.y()<0)||
               ((selectionBox.x()+selectionBox.width())>captureThread->getInputSourceWidth())||
               ((selectionBox.y()+selectionBox.height())>captureThread->getInputSourceHeight()))
            {
                // Display error message
                QMessageBox::warning(this,"ERROR:","Selection box outside range. Please try again.");
            }
            // Set ROI
            else
                emit setROI(selectionBox);
        }
    }
    // Reset ROI
    else if(mouseData.rightButtonRelease)
        emit setROI(QRect(0, 0, captureThread->getInputSourceWidth(), captureThread->getInputSourceHeight()));
}

void CameraView::setGrayscale(bool input)
{
    imageProcessingFlags.grayscaleOn=input;
    // Update image processing flags in processing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setSmooth(bool input)
{
    imageProcessingFlags.smoothOn=input;
    // Update image processing flags in pprocessing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setDilate(bool input)
{
    imageProcessingFlags.dilateOn=input;
    // Update image processing flags in processing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setErode(bool input)
{
    imageProcessingFlags.erodeOn=input;
    // Update image processing flags in processing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setFlip(bool input)
{
    imageProcessingFlags.flipOn=input;
    // Update image processing flags in processing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setCanny(bool input)
{
    imageProcessingFlags.cannyOn=input;
    // Update image processing flags in processing thread
    emit newImageProcessingFlags(imageProcessingFlags);
}

void CameraView::setScaledContents(bool input)
{
    ui->frameLabel->setScaledContents(input);
}
