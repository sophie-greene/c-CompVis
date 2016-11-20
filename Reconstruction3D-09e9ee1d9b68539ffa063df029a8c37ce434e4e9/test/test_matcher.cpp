#include "../include/detector.h"
#include "../include/color.h"

void displayHelp(std::string command)
{
  std::cout << "Usage:\n" << command << " -i1 <filename> -i2 <filename> -o <filename>" << std::endl;
  std::cout<< " [-e <feature_type> <descriptor_type> <matcher_type>] [-param <nb_params> <param1> ... ] [-flip <flip_mode>]" << std::endl;
}

int main(int argc, char* argv[])
{
  std::string  input1, input2, output;
  output = "";

  Detector detector;
  Detector::FeatureType featureType = Detector::SIFT;
  Detector::DescriptorType descriptorType = Detector::SIFT_EXTRACTOR;
  Detector::MatcherType matcherType = Detector::L2;
  std::vector<double> params;
  int flipMode = 0;

  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-o")) {output = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i1")) {input1 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i2")) {input2 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-e")) {
      featureType = detector.getFeatureType(argv[i+1]); i++;
      if(featureType!=Detector::HARRIS && featureType!=Detector::OLDSURF){
        descriptorType = detector.getDescriptorType(argv[i+1]); i++;
        matcherType = detector.getMatcherType(argv[i+1]); i++;
      }
      continue;
    }
    if(!strcmp(argv[i],"-param")) {
      int nbParam = atoi(argv[i+1]); i++;
      params.clear();
      for(int j=0;j<nbParam;j++){
        params.push_back( atof(argv[i+1]) );
        i++;
      }
      continue;
    }
    if(!strcmp(argv[i],"-flip")) {flipMode = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-h") || !strcmp(argv[i],"--help") || !strcmp(argv[i],"-?")) {displayHelp(argv[0]); detector.displayParametersHelp(); return 1;}
    std::cout << color::red() << "Unknown option: " << color::Red() << argv[i] << color::reset() << std::endl;
    displayHelp(argv[0]);
    detector.displayParametersHelp();
    return 1;
  }

  assert(input1!="" && input2!="");

  detector.setImages(input1,input2);
  detector.flipImages(flipMode);
  detector.setType(featureType,descriptorType,matcherType);
  detector.setDefaultParameters(featureType);
  detector.setParam(params);
  detector.compute();

  if(output==""){
    detector.displayResult(1920);
  }else{
    cv::Mat out = detector.getResult(2048);
    cv::imwrite(output,out);
  }

  return 0;
}
