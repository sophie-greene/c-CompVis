#include "../include/mutualinformation.h"

/* MatLab function

function z = mutualInformation(x, y)
% Compute mutual information I(x,y) of two discrete variables x and y.
% Written by Mo Chen (mochen80@gmail.com).

    assert(numel(x) == numel(y));
    n = numel(x);
    x = reshape(x,1,n);
    y = reshape(y,1,n);

    l = min(min(x),min(y));
    x = x-l+1;
    y = y-l+1;
    k = max(max(x),max(y));

    idx = 1:n;
    Mx = sparse(idx,x,1,n,k,n);
    My = sparse(idx,y,1,n,k,n);
    Pxy = nonzeros(Mx'*My/n); %joint distribution of x and y
    Hxy = -dot(Pxy,log2(Pxy+eps));

    Px = mean(Mx,1);
    Py = mean(My,1);

    % entropy of Py and Px
    Hx = -dot(Px,log2(Px+eps));
    Hy = -dot(Py,log2(Py+eps));

    % mutual information
    z = Hx + Hy - Hxy;
endfunction

  */

MutualInformation::MutualInformation(int nbBins) : nbBins_(nbBins)
{

}

MutualInformation::MutualInformation(cv::Mat image1, cv::Mat image2, int nbBins) : nbBins_(nbBins)
{
  assert(image1.rows == image2.rows);
  assert(image1.cols == image2.cols);

  int mi[nbBins_][nbBins_];

  for(int i=0;i<nbBins_;i++){
    for(int j=0;j<nbBins_;j++){
      mi[i][j] = 0;
    }
  }

  for(int i=0;i<image1.rows;i++){
    for(int j=0;j<image1.cols;j++){
      int i1 = std::max(0,std::min(image1.at<uchar>(i,j) * nbBins_ / 255,nbBins_-1));
      int i2 = std::max(0,std::min(image2.at<uchar>(i,j) * nbBins_ / 255,nbBins_-1));
      //std::cout << i1 << " " << i2 << std::endl;
      mi[i1][i2]++;
    }
  }
  std::cout << image1.type() << std::endl;

  int max = 0;
  for(int i=0;i<nbBins_;i++){
    for(int j=0;j<nbBins_;j++){
      max = std::max(max,mi[i][j]);
    }
  }

  cv::Mat out(nbBins_,nbBins_,CV_8U);
  for(int i=0;i<nbBins_;i++){
    for(int j=0;j<nbBins_;j++){
      //std::cout << std::max(0,std::min(255*mi[i][j]/max,255)) << std::endl;
      out.at<uchar>(i,j) = std::max(0,std::min(255*mi[i][j]/max,255));
    }
  }

  cv::imwrite("MItest.jpg",out);
}

double MutualInformation::compute(cv::Mat image1, cv::Mat image2, int nbBins)
{
  assert(image1.rows==image2.rows && image1.cols==image2.cols);
  assert(image1.rows!=0 && image1.cols!=0);
  if(nbBins>0) nbBins_ = nbBins;

  int size = image1.rows*image1.cols;

  std::vector<cv::Mat> im;
  cv::Mat hist1,hist2;
  std::vector<int> channels(1,0);
  std::vector<int> histSize(1,nbBins_);
  std::vector<float> histRange(0,nbBins_);
  im.clear();
  im.push_back(image1);
  cv::calcHist(im,channels,cv::Mat(),hist1,histSize,histRange);
  im.clear();
  im.push_back(image2);
  cv::calcHist(im,channels,cv::Mat(),hist2,histSize,histRange);

  hist1*=1.0/(double)size;
  hist2*=1.0/(double)size;

  if(nbBins_!=256){
    scaleBins(image1,nbBins_,256);
    scaleBins(image2,nbBins_,256);
  }

  double result = 0.0;

  for(int i=0;i<nbBins_;i++){
    for(int j=0;j<nbBins_;j++){
      double pI1 = hist1.at<float>(i,0);
      //double pI1 = probability(hist1,i,nb);
      if(pI1==0.0) continue;
      double pI2 = hist2.at<float>(j,0);
      //double pI2 = probability(hist2,j,nb);
      if(pI2==0.0) continue;

      double pI1I2 = joinProbability(image1,image2,i,j);
      if(pI1I2==0.0) continue;
      result += pI1I2 * log( pI1I2/(pI1*pI2) );
    }
  }
  //std::cout << "result: " << result << std::endl;
  return result;
}

// pI(i) , DamePhD p26
double MutualInformation::probability(cv::Mat &I, int t)
{
  double Nx = I.rows*I.cols;
  double result = 0.0;
  for(int i=0;i<I.rows;i++){
    for(int j=0;j<I.cols;j++){
      result += d(t-(int)(I.at<uchar>(i,j)));
    }
  }
  return result / Nx;
}

// pI(i) , DamePhD p26
double MutualInformation::joinProbability(cv::Mat &I1, cv::Mat &I2, int i, int j)
{
  double Nx = I1.rows*I1.cols;

  cv::Mat o1,o2;
  cv::compare(I1,i,o1,cv::CMP_EQ);
  cv::compare(I2,j,o2,cv::CMP_EQ);

  cv::Mat o;
  cv::multiply(o1,o2,o);

  return (double)(cv::sum(o).val[0]) / Nx;


  /*for(int m=0;m<I1.rows;m++){
    for(int n=0;n<I1.cols;n++){
      result += d(i-(int)I1.at<uchar>(m,n))*d(j-(int)I2.at<uchar>(m,n));
    }
  }
  return result / Nx;*/
}

// pI(i) , DamePhD p26
double MutualInformation::probability(std::vector<int> &I, int t, int nb)
{
  double result = I.at(t);
  return result / (double)nb;
}


// pI(i) , DamePhD p26
double MutualInformation::joinProbability(std::vector<int> &I12, int i, int j)
{
  double Nx = 0;
  for(int i=0;i<I12.size();i++)
    Nx += I12.at(i);


  double result = 0.0;
  //TODO
  return result / Nx;
}

// pI(i) , DamePhD p26
double MutualInformation::probability(int I[], int t, int nb)
{
  double result = I[t];
  return result / (double)nb;
}

// pI(i) , DamePhD p26
double MutualInformation::joinProbability(int I12[], int i, int j, int nb)
{
    double result = 0.0;
}

//H(I) , DamePhD p26
double MutualInformation::entropy(cv::Mat I)
{
  double result = 0.0;
  for(int i=0;i<nbBins_;i++){
      double pI = probability(I,i);
      result += pI * log(pI);
  }
  return -result;
  //TODO
}

double MutualInformation::joinEntropy(cv::Mat &I1, cv::Mat &I2, int i, int j)
{

  return 0.0;
}

void MutualInformation::scaleBins(cv::Mat &I, int nbBins, int oldNbBins)
{
  I *= ((double)(nbBins-1))/((double)(oldNbBins-1));
}

double MutualInformation::probability(cv::Mat I, int t, cv::Mat p)
{
  cv::Mat Is = w(I,p);
  int Nx = Is.rows*Is.cols;
  double result = 0.0;
  for(int i=0;i<Is.rows;i++){
    for(int j=0;j<Is.cols;j++){
      result += d(t-(int)(Is.at<int>(i,j)));
    }
  }
  return result / (double)Nx;
}

cv::Mat MutualInformation::w(cv::Mat I, cv::Mat p)
{
  //TODO
  return I;
}

//d(x) , DamePhD p26
double MutualInformation::d(int p)
{
  if(p==0) return 1.0;
  else return 0.0;
}

cv::Mat MutualInformation::H(cv::Mat I1, cv::Mat I2)
{
  cv::Mat res;
  int r_max,t_max;
  for(int r=0;r<r_max;r++){
    for(int t=0;t<t_max;t++){
      //TODO
      cv::Mat temp;
      temp = dpii().t() * dpii() * ((1.0/probability(I1,t/* PII* */)) - (1.0/probability(I1,t/* PI* */)))
             + d2pii() * (1.0 + log( probability(I1,t/* PII* */)/probability(I1,t/* PI* */) ));
      res += temp;
    }
  }
  return res;
}

cv::Mat MutualInformation::G(cv::Mat I1, cv::Mat I2)
{
  cv::Mat res;
  int r_max,t_max;
  for(int r=0;r<r_max;r++){
    for(int t=0;t<t_max;t++){
      //TODO
      cv::Mat temp;
      temp = dpii() * (1.0 + log( probability(I1,t/* PII* */)/probability(I1,t/* PI* */) ));
      res += temp;
    }
  }
  return res;
}

cv::Mat MutualInformation::computePose(cv::Mat I1, cv::Mat I2)
{
  cv::Mat deltaP = -H(I1,I2).inv()*G(I1,I2).t();
  return deltaP;
}

// DamePhD , page 32
double MutualInformation::testSSD(cv::Mat image1, cv::Mat image2)
{
    double size = image1.rows*image1.cols;
    //image1.convertTo(image1,CV_32F);
    //image2.convertTo(image2,CV_32F);
    cv::Mat out;
    cv::absdiff(image1,image2,out);
    return (double) (cv::sum(out).val[0]) / size;
}
