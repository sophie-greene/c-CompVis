#include "uvccapture.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;
int main(){

//  while(1){
    Mat s=capture("/dev/video0","sna0.jpg",320,240,0,0,0,0);
   imshow("win",s);
//}
//    capture("/dev/video1","sna1.jpg",320,240,0,0,0,0);
//    capture("/dev/video2","sna2.jpg",320,240,0,0,0,0);
//    capture("/dev/video3","sna3.jpg",320,240,0,0,0,0);
//    capture("/dev/video4","sna4.jpg",320,240,0,0,0,0);
//    capture("/dev/video5","sna5.jpg",320,240,0,0,0,0);
//    capture("/dev/video6","sna6.jpg",320,240,0,0,0,0);
//    capture("/dev/video7","sna7.jpg",320,240,0,0,0,0);
//    Mat x=imread("sna2.jpg");
//    imshow("hello",x);
    return 0;
}
