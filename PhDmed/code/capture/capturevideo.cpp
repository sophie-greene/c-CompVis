#include "capturevideo.h"

CaptureVideo::CaptureVideo()
{

}

Mat CaptureVideo::getFrame()
{
    if (uvcGrab (videoIn) < 0) {
        fprintf (stderr, "Error grabbing\n");
        close_v4l2 (videoIn);
        free (videoIn);
        exit (1);
    }
    Mat imgbuf(Size(videoIn->width, videoIn->height), CV_8UC3, videoIn->tmpbuffer);
    Mat image = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
    return image;
}

bool CaptureVideo::startCapture (int device,int width, int height,int brightness, int contrast, int saturation,int gain)
{
    this->device=device;
    char buff[20];
    sprintf(buff, "/dev/video%d", this->device);
    char *videodevice = buff;
    int format = V4L2_PIX_FMT_MJPEG;
    int grabmethod = 1;


    this->videoIn = (struct vdIn *) calloc (1, sizeof (struct vdIn));
    if (init_videoIn
        (videoIn, (char *) videodevice, width, height, format, grabmethod) < 0){
        return false;
        exit (1);
    }


    //Reset all camera controls
    v4l2ResetControl (videoIn, V4L2_CID_BRIGHTNESS);
    v4l2ResetControl (videoIn, V4L2_CID_CONTRAST);
    v4l2ResetControl (videoIn, V4L2_CID_SATURATION);
    v4l2ResetControl (videoIn, V4L2_CID_GAIN);

    //Setup Camera Parameters
    if (brightness != 0) {
        v4l2SetControl (videoIn, V4L2_CID_BRIGHTNESS, brightness);
    }
    if (contrast != 0) {
        v4l2SetControl (videoIn, V4L2_CID_CONTRAST, contrast);
    }
    if (saturation != 0) {
        v4l2SetControl (videoIn, V4L2_CID_SATURATION, saturation);
    }
    if (gain != 0) {
        v4l2SetControl (videoIn, V4L2_CID_GAIN, gain);
    }
    this->width=width;
    this->heigh=height;
    this->saturation=saturation;
    this->contrast=contrast;
    this->gain=gain;

    return true;
}

void CaptureVideo::endCapture()
{
    videoIn->getPict = 0;
    close_v4l2 (videoIn);
    free (videoIn);

}
