#include "../include/mutualinformation.h"
#include "../include/tools.h"

#include "../include/detector.h"

#include "../include/imageprocessing.h"
#include "../include/fileprocessing.h"
#include "../include/color.h"
#include "../include/gnuplot.h"

#include <omp.h>

#include <time.h>

#include <QRunnable>
#include <QThreadPool>
#include <QTime>

using namespace FileProcessing;
using namespace ImageProcessing;

#define CLOCKS_PER_MSEC (CLOCKS_PER_SEC/1000.0)

class WorkingClass : public QRunnable
{
    int id_;
    bool useSSD_;
    cv::Mat Ip_,Io_;
    double *val_;
    double theta_;
    double scale_;
    int nbBins_;
    cv::Point2d center_;
    int w_, h_;
public:
    WorkingClass(int id, cv::Point2d center, int w, int h, bool useSSD, cv::Mat Ip, cv::Mat Io, double theta, double scale, int nbBins, double* val) :
            id_(id), center_(center), w_(w), h_(h), useSSD_(useSSD), Ip_(Ip), Io_(Io), theta_(theta), scale_(scale), nbBins_(nbBins), val_(val) {
        setAutoDelete(true);
    }
    void run()
    {
        MutualInformation mi;
        //std::cout << "Running Thread" << id_ << std::endl;

        //cv::Mat I = Tools::unwarp(Io,cv::Point2d(668,459)*scale, 82*scale, 458*scale, theta, 1.75, 1024*scale, 768*scale,Tools::HORIZONTAL_AXIS);
        cv::Mat I = Tools::unwarp(Io_, center_, 82, 458, theta_, 1.75, w_*scale_, h_*scale_,Tools::HORIZONTAL_AXIS);

        if(useSSD_)
          *val_ = mi.testSSD(Ip_,I);
        else{
          cv::Mat I2 = Ip_.clone();
          *val_ = mi.compute(I2,I,nbBins_);
        }

        /*Detector detector(Ip_,I);
        detector.setDefaultParameters(Detector::HARRIS);
        detector.compute(Detector::HARRIS,Detector::NO_DESCRIPTOR_TYPE,Detector::NO_MATCHER_TYPE);
        *val_ = detector.getNbPairs();*/

        //std::cout << "Ending Thread" << id_ << std::endl;
    }
};

struct Result{
  int nImage;
  double score;
  double theta;
};

void displayHelp(std::string command)
{
  std::cout << "Usage:\n" << command << " -i <filename> -n <nb_bins>" << std::endl;
}

int main(int argc, char* argv[])
{

#if 1

  std::string i1 = argv[1];
  std::string i2 = argv[2];
  cv::Mat I1 = cv::imread(i1,cv::IMREAD_GRAYSCALE);
  cv::Mat I2 = cv::imread(i2,cv::IMREAD_GRAYSCALE);
  MutualInformation mi(I1,I2,255);

#else
  std::string input, input1, input2, output, folder, base, ext;
  int nbBins = 256;
  int step = 1;
  int start = 0;
  double scale = 1.0;
  bool useSSD = false;
  int end = 6;
  input = "images/image00000.jpg";
  folder = "imagesMI/sequence/";
  base = "image";
  ext = "pgm";

  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-i")) {input = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-d")) {folder = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-b")) {base = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-e")) {ext = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-o")) {output = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i1")) {input1 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i2")) {input2 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-n")) {nbBins = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-ssd")) {useSSD = true; continue;}
    if(!strcmp(argv[i],"-step")) {step = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-start")) {start = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-end")) {end = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-scale")) {scale = atof(argv[i+1]); i++; continue;}
    std::cout << color::red() << "Unknown option: " << color::Red() << argv[i] << color::reset() << std::endl;
    displayHelp(argv[0]);
    return 1;
  }

  if(input=="")
    return 1;

  if(end<=start){
    std::cout << "end <= start" << std::endl;
    return 1;
  }

  cv::Point2d center(668,459);
  int w = 1024;
  int h = 768;

  assert(fexists(input));
  cv::Mat Ip = cv::imread(input,cv::IMREAD_GRAYSCALE);


  w = Ip.cols;
  h = Ip.rows;

  Ip = Tools::rescale(Ip,scale);

  std::vector<Result> vals;

  int nbZero = nbNumber(folder+"/"+base ,ext,start);


  QTime timeGlobal;
  timeGlobal.start();
  for(int k=start;k<end;k+=step){
    QTime time;
    time.start();
    std::string filename = makefilename(folder+"/"+base ,ext,k,nbZero);
    assert(fexists(filename));
    cv::Mat Io = cv::imread(filename,cv::IMREAD_GRAYSCALE);
    double val[62];
    int theta_int = 0;
    {
      for(theta_int=0;theta_int<62;theta_int++){
        double theta = 0.1*(double)theta_int;

        WorkingClass *runnable = new WorkingClass(theta_int,center,w,h,useSSD,Ip,Io,theta,scale,nbBins,&val[theta_int]);
        QThreadPool::globalInstance()->start(runnable);

        /*//cv::Mat I = Tools::unwarp(Io,cv::Point2d(668,459)*scale, 82*scale, 458*scale, theta, 1.75, 1024*scale, 768*scale,Tools::HORIZONTAL_AXIS);
        cv::Mat I = Tools::unwarp(Io,cv::Point2d(668,459), 82, 458, theta, 1.75, 1024*scale, 768*scale,Tools::HORIZONTAL_AXIS);
        if(useSSD)
            val[theta_int] = mi.testSSD(Ip,I);
        else{
          cv::Mat I2 = Ip.clone();
          val[theta_int] = mi.compute(I2,I,nbBins);
        }*/
      }
    }
    QThreadPool::globalInstance()->waitForDone();

    double best_result = val[0];
    double best_match = 0.0;

    gnuplot::openFile(makefilename("MI_result_","txt",k,1));
    for(theta_int=0;theta_int<62;theta_int++){
      fprintf(gnuplot::fgnuplot,"%f %f\n",0.1*(double)theta_int,val[theta_int]);
      if( (useSSD && (val[theta_int]<best_result)) || (!useSSD && (val[theta_int]>best_result))){
        best_result = val[theta_int];
        best_match = 0.1*theta_int;
      }
    }
    gnuplot::closeFile();

    Result r;
    r.nImage = k;
    r.score = best_result;
    r.theta = best_match;
    vals.push_back( r );

    //cv::Mat I = Tools::unwarp(Io,cv::Point2d(668,459)*scale, 82*scale, 458*scale, best_match, 1.75, 1024*scale, 768*scale,Tools::HORIZONTAL_AXIS);
    cv::Mat Iu = Tools::unwarp(Io,center, 82, 458, best_match, 1.75, w*scale, h*scale,Tools::HORIZONTAL_AXIS);
    cv::Mat res;
    cv::absdiff(Ip,Iu,res);

    cv::Mat I;
    I = concatImages(Ip,Iu,0.5,false);
    I = concatImages(I,res);
    cv::putText(I, Tools::toStr(r.nImage),cv::Point2d(50,50),cv::FONT_HERSHEY_SIMPLEX,1.2,CV_RGB(255,255,255),2);
    cv::putText(I, Tools::toStr(r.score),cv::Point2d(50,100),cv::FONT_HERSHEY_SIMPLEX,1.2,CV_RGB(255,255,255),2);
    cv::imwrite(makefilename("MI/test","jpg",k,1),I);
    std::cout << k << " done in " << time.elapsed() << "ms" << std::endl;
  }

  double best_result = vals.at(0).score;
  double best_theta = vals.at(0).theta;
  int best_match = vals.at(0).nImage;
  gnuplot::openFile("result_match.txt");
  for(int i=0;i<vals.size();i++){
    fprintf(gnuplot::fgnuplot,"%d %f\n",vals.at(i).nImage, vals.at(i).score);
    if( (useSSD && (vals.at(i).score<best_result)) || (!useSSD && (vals.at(i).score>best_result))){
      best_result = vals.at(i).score;
      best_theta = vals.at(i).theta;
      best_match = vals.at(i).nImage;
    }
  }
  gnuplot::closeFile();
  std::cout << "Best image: "<< best_match << std::endl;
  std::cout << "Best angle: "<< best_theta << std::endl;

  cv::Mat Io = cv::imread(makefilename(folder+"/"+base,ext,best_match,nbZero),cv::IMREAD_GRAYSCALE);
  Io = Tools::rescale(Io,scale);
  cv::Mat Iu = Tools::unwarp(Io,cv::Point2d(668,459)*scale, 82*scale, 458*scale, best_theta, 1.75, w*scale, h*scale,Tools::HORIZONTAL_AXIS);
  cv::Mat res;
  cv::absdiff(Ip,Iu,res);

  cv::Mat I;
  I = concatImages(Ip,Iu,0.5,false);
  I = concatImages(I,res);
  cv::putText(I, Tools::toStr(best_match),cv::Point2d(50,50),cv::FONT_HERSHEY_SIMPLEX,1.2,CV_RGB(255,255,255),2);
  cv::putText(I, Tools::toStr(best_result),cv::Point2d(50,100),cv::FONT_HERSHEY_SIMPLEX,1.2,CV_RGB(255,255,255),2);
  cv::imwrite("MI/test_best.jpg",I);


  std::cout << "Done in " << (int)(timeGlobal.elapsed()/1000) << "s" << std::endl;

  return 0;
#endif
}

