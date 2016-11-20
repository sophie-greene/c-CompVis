#include "../include/loopclosure.h"

LoopClosure::LoopClosure(DataManager *dm) : dm_(dm), matcher(new cv::BFMatcher(cv::NORM_L2,true)), extractor(new cv::SiftDescriptorExtractor)
{
}

void LoopClosure::createInvertedFileFromDB()
{
  std::vector<PointData> corres = dm_->getPointData();
  std::vector<PoseData> images = dm_->getPosesData();

  Timer timer;
  invertedFile.clear();
  //for(int i=0;i<images.size();i++){
  std::vector<PoseData>::iterator it;
  for(it = images.begin(); it != images.end(); it++){
    cv::Mat img = cv::imread(it->getPath());
    //std::cout << img.rows << "x" << img.cols << std::endl;


    std::vector<PointData> ptsd;
    std::vector<cv::Point2d> pts;
    for(int i=0;i<it->getNbPoint();i++){
      int id = it->getPointID(i);
      //cv::Mat d = getDescriptor(img,corres[id].getPoint());
      //addPoint(d,corres[id]);
      pts.push_back( corres[id].getPoint() );
      ptsd.push_back( corres[id] );
    }



    std::vector<cv::Mat> descriptors = getDescriptors(img,pts);
    for(int i=0;i<pts.size();i++){
      addPoint(descriptors[i].clone(),ptsd[i]);
      //cv::circle(img,pts.at(i),2,CV_RGB(255,0,0),-1);
    }

    //cv::imshow("Display",img);
    //cv::waitKey(2);

    std::cout << "[" << timer.elapsed() << "] Add " << it->getNbPoint() << " points of image " << it->getID() << std::endl;
  }
  std::cout << "Create InvertedFile : " << invertedFile.size() << " nodes" << std::endl;

  for(int i=0;i<invertedFile.size();i++){
    std::cout << " - Node " << i << ": " << invertedFile.at(i).data.size() << " points" << std::endl;
  }
}

cv::Mat LoopClosure::getDescriptor(const cv::Mat &img, const cv::Point2d &pt)
{
  cv::Mat descriptor;
  cv::KeyPoint key(pt,5.0);
  std::vector<cv::KeyPoint> keypoint(1,key);
  extractor->compute(img, keypoint, descriptor);
  return descriptor;
}

std::vector<cv::Mat> LoopClosure::getDescriptors(const cv::Mat &img, const std::vector<cv::Point2d> &pts)
{
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoint;
  for(int i=0;i<pts.size();i++){
    cv::KeyPoint key(pts.at(i),1.0);
    keypoint.push_back( key );
  }
  extractor->compute(img, keypoint, descriptors);
  std::vector<cv::Mat> ret;
  for(int i=0;i<descriptors.rows;i++)
    ret.push_back( descriptors.row(i).clone() );

  return ret;
}

void LoopClosure::addPoint(const cv::Mat &pointDescriptor, const PointData &point)
{
  std::vector<Node>::iterator it;
  for(it = invertedFile.begin(); it != invertedFile.end(); it++){
    if(isMatching(pointDescriptor,it->descriptor)){
      it->data.push_back(point);
      return;
    }
  }
  Node n;
  n.id = invertedFile.size();
  n.descriptor = pointDescriptor.clone();
  n.data.push_back( point );
  invertedFile.push_back( n );
}

std::vector<int> LoopClosure::getNodeID(const cv::Mat &currentDescriptor)
{
  std::vector<int> res;
  std::vector<Node>::iterator it;
  for(it = invertedFile.begin(); it != invertedFile.end(); it++){
    if(isMatching(currentDescriptor,it->descriptor)){
      res.push_back(it->id);
    }
  }
}

bool LoopClosure::isMatching(const cv::Mat &descriptor1, const cv::Mat &descriptor2)
{
  //std::cout << descriptor1.rows << "x" << descriptor1.cols << std::endl;
  //cv::imwrite("des1.png",descriptor1);
  //cv::imwrite("des2.png",descriptor2);
  std::vector< cv::DMatch > matches;
  matcher->clear();
  matcher->match(descriptor1, descriptor2, matches);
  //std::cout << (matches.size()>0?"true":"false") << std::endl;
  return matches.size()>0;
}
