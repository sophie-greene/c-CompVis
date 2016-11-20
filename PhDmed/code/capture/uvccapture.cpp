


#include "uvccapture.h"

#include "v4l2uvc.h"
Mat capture (int device,   int width, int height ,int brightness  , int contrast , int saturation, int gain )
{

    char buff[20];
    sprintf(buff, "/dev/video%d", device);
    char *videodevice = buff;
    int format = V4L2_PIX_FMT_MJPEG;
    int grabmethod = 1;

    struct vdIn *videoIn;
    videoIn = (struct vdIn *) calloc (1, sizeof (struct vdIn));
    if (init_videoIn
        (videoIn, (char *) videodevice, width, height, format, grabmethod) < 0)
        exit (1);

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

    if (uvcGrab (videoIn) < 0) {
        fprintf (stderr, "Error grabbing\n");
        close_v4l2 (videoIn);
        free (videoIn);
        exit (1);
    }
    Mat imgbuf(Size(videoIn->width, videoIn->height), CV_8UC3, videoIn->tmpbuffer);
    Mat image = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);

    videoIn->getPict = 0;   
    close_v4l2 (videoIn);
    free (videoIn);
    return image;
}
void closeCap(int device){

}

