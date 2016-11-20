
#include <iostream>
using namespace cv;
using namespace std;

int main(int, char**)
{
    cout<<"hello";
//    VideoCapture cap(0); // open the default camera
//    if(!cap.isOpened())  // check if we succeeded
//       {
//     cout << "Error opening camera!";
//     waitKey(0);
//     return -1;
// }else{
//        cout<<"cam there";
//    }

//    Mat edges;
//    namedWindow("webcam",2);
//    namedWindow("webcam edges",1);
//    for(;;)
//    {
//        Mat frame;
//        cap >> frame; // get a new frame from camera
        
//        cvtColor(frame, edges, CV_BGR2GRAY);
//        imshow("webcam", edges);
//        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
//        Canny(edges, edges, 0, 30, 3);
//        imshow("webcam edges", edges);
//        if(waitKey(30) >= 0) break;
//    }
//    // the camera will be deinitialized automatically in VideoCapture destructor
//    return 0;
}
