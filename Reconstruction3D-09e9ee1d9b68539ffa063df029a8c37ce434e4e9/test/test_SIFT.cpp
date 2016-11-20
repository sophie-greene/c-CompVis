
#include "../include/detector3.h"
#include "imageprocessing.h"
#include "../include/tools.h"
#include "../include/color.h"

int val1 = 0;
int max1 = 16;
int val2 = 3;
int max2 = 32;
int val3 = 40;
int max3 = 10000;
int val4 = 300;
int max4 = 10000;
int val5 = 800;
int max5 = 10000;
bool force = false;

Detector3 detector;

// _nfeatures=0; _nOctaveLayersSIFT=3; _contrastThreshold=0.04; _edgeThreshold=10; _sigma=1.6;

void onChange(int value)
{
  printf("%d %d %f %f %f",val1,val2,(double)val3/1000.0,(double)val4/100.0,(double)val5/1000.0);
  if(val2<1) return;
  if(val3<1) return;
  if(val4<1) return;
  if(val5<1) return;
  detector.setParamsSIFT(val1,val2,(double)val3/1000.0,(double)val4/100.0,(double)val5/1000.0);
  detector.computeSIFT(force);
  cv::Mat I = detector.getResult(2048);
  printf(" - %d\n",detector.getNbPairs());
  cv::imshow("Display",I);
}

void displayHelp(std::string command)
{
  std::cout << "Usage :" <<std::endl
            << command << " [options]" << std::endl;
}

int main(int argc, char* argv[])
{
  std::string filepath1 = "";
  std::string filepath2 = "";
  std::string filepath3 = "";

  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-i1")) {filepath1 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i2")) {filepath2 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i3")) {filepath3 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-force")) {force=true; continue;}
    std::cout << color::red() << "Unknown option: " << color::Red() << argv[i] << color::reset() << std::endl;
    displayHelp(argv[0]);
    return 0;
  }

  cv::Mat I1,I2,I3;
  I1 = cv::imread(filepath1);
  I2 = cv::imread(filepath2);
  I3 = cv::imread(filepath3);
  //cv::Mat I = concatImages(I1,I2);
  //        I = concatImages(I,I3);

  detector.setImages(I1,I2,I3);
  detector.setParamsSIFT(1,4,0.06,4.0,0.8);
  detector.computeSIFT(force);

  std::cout << detector.getNbPairs() << " pairs" << std::endl;

  cvNamedWindow("Display");
  cvCreateTrackbar("nfeatures", "Display", &val1, max1, NULL );
  cvCreateTrackbar("nOctaveLayersSIFT", "Display", &val2, max2, NULL );
  cvCreateTrackbar("contrastThreshold", "Display", &val3, max3, NULL );
  cvCreateTrackbar("edgeThreshold", "Display", &val4, max4, NULL );
  cvCreateTrackbar("sigma", "Display", &val5, max5, NULL );
  int valid = 0;
  cvCreateTrackbar("Apply", "Display", &valid, 1, onChange );

  cv::Mat I = detector.getResult(2048);
  cv::imshow("Display",I);

  while(cv::waitKey(250)!='q');

  return 0;
}
