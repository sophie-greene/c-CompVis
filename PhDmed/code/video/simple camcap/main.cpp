#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include <QDebug>
#include <boost/lexical_cast.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoCapture cap[8];
    string name;
    Mat image;
    int i;
    for (i=0;i<8;i++)
    {
        cap[i].open(i);

        if(!cap[i].isOpened() )  // check if we succeeded
            return -1;

        cap[i].set(CV_CAP_PROP_FRAME_WIDTH, 160);
        cap[i].set(CV_CAP_PROP_FRAME_HEIGHT, 120);
        cap[i].set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
        name=boost::lexical_cast<string>(i);
        name="camera"+name;
        namedWindow(name.c_str(),i);


    }
    while(1)
    {
        for(i=0;i<8;i++){
            cap[i] >> image;
            name=boost::lexical_cast<string>(i);
            name="camera"+name;
            imshow(name, image);
        }

    if(waitKey(10) == 99 ) break;
}
return 0 ;
}
