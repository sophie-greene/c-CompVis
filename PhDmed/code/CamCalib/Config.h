#ifndef CONFIG_H
#define CONFIG_H

// OpenCV header files
#include <opencv/cv.h>
#include <opencv/highgui.h>
// Qt header files
#include <QtGui>

// FPS statistics queue lengths
#define DEFAULT_PROCESSING_FPS_STAT_QUEUE_LENGTH 32
#define DEFAULT_CAPTURE_FPS_STAT_QUEUE_LENGTH 32

// Camera device number
#define DEFAULT_CAMERA_DEV_NO -1
// Image buffer size
#define DEFAULT_IMAGE_BUFFER_SIZE 1
// Drop frame if image/frame buffer is full
#define DEFAULT_DROP_FRAMES false
// Thread priorities
#define DEFAULT_CAP_THREAD_PRIO QThread::HighPriority
#endif // CONFIG_H
