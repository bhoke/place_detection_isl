#include "bubbleprocess.h"
#include <QStringList>
#include <QTextStream>
#include <math.h>
#include <QDebug>
double bubbleProcess::round(double r) {
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

static vector<int> imagePanAngles;
static vector<int> imageTiltAngles;

bubbleProcess::bubbleProcess()
{
}

bubbleStatistics bubbleProcess::calculateBubbleStatistics(const vector<bubblePoint>& bubble)
{
  bubbleStatistics result;

  // cv::Mat bubbleArr(1,bubble.size(),CV_32FC1);
  std::vector<float> values(bubble.size());

  //qDebug()<<bubble.size();

  for(uint i = 0; i < bubble.size(); i++)

  {
    values[i] = bubble.at(i).val;
    //bubbleArr.at<float>(1,i) = (float)bubble.at(i).val;
    //qDebug()<<values[i];
  }

  cv::Scalar summ = cv::sum(values);

  result.mean = summ[0]/(60*60);

  if(result.mean > 1.0) result.mean = 1.0;

  cv::Scalar mean;
  cv::Scalar stddev;

  cv::meanStdDev(values,mean,stddev);

  result.variance = stddev.val[0]*stddev.val[0];
  return result;
}

void bubbleProcess::calculateImagePanAngles(int focalLengthPixels, int imageWidth)
{
  int deltax,panInt;
  float pan;
  qDebug() << "Pan Angles:";
  for(int i = imageWidth-1; i > -1; i--)
  {
    deltax = imageWidth/2 - i;
    pan = atan2((double)deltax,(double)focalLengthPixels);
    qDebug() << (pan*180)/M_PI;
    panInt = (pan*180)/M_PI;
    if(panInt < 0)
        panInt += 360;
    imagePanAngles.push_back(panInt);
  }
}

void bubbleProcess::calculateImageTiltAngles(int focalLengthPixels, int imageHeight)
{
  int deltay,tiltInt;
  float tilt;
  qDebug() << "Tilt Angles:";
  for(int i = imageHeight-1; i > -1; i--)
  {
    deltay = imageHeight/2 - i;
    tilt = atan2((double)deltay,(double)focalLengthPixels);
    qDebug() << (tilt*180)/M_PI;;
    tiltInt = (tilt*180)/M_PI;
    if(tiltInt < 0)
        tiltInt += 360;
    imageTiltAngles.push_back(tiltInt);
  }
}

std::vector<bubblePoint> bubbleProcess::convertGrayImage2Bub(cv::Mat grayImage)
{
  std::vector<bubblePoint> result;
  grayImage.convertTo(grayImage,CV_32FC1);
  float val;

  for(int i = 0; i < grayImage.rows; i++)
  {
    for(int j = 0; j < grayImage.cols; j++)
    {
      val = grayImage.at<float>(i,j);
      bubblePoint pt;
      pt.tiltAng = imageTiltAngles[i];
      pt.panAng = imagePanAngles[j];
      pt.val = val;
      result.push_back(pt);
    }
  }
  return result;
}

std::vector<bubblePoint> bubbleProcess::readBubble(QFile *file){

  QTextStream stream(file);

  vector<bubblePoint> result;

  if(!file->isOpen()) return result;

  QString line = stream.readLine();

  while(line != NULL)
  {
    bubblePoint pt;

    QStringList lt = line.split(" ");

    if(lt.size() == 3){

      pt.panAng = lt[0].toInt();
      pt.tiltAng= lt[1].toInt();
      pt.val = lt[2].toDouble();

      // if val is in normal ranges save it
      if(pt.val < 1 && pt.val > 0){
        // pt.val = pt.val *30;
        result.push_back(pt);
      }
    }
    line = stream.readLine();
  }
  return result;
}

std::vector<bubblePoint> bubbleProcess::reduceBubble(std::vector<bubblePoint> bubble)
{
  vector<bubblePoint> result;

  double vals[360][360];
  double counts[360][360];

  for(int i = 0; i < 360; i++)
  {
    for(int j = 0; j< 360; j++)
    {
      vals[i][j] = 0;
      counts[i][j] = 0;
    }
  }

  for(ulong i = 0; i < bubble.size(); i++)
  {
    bubblePoint pt;
    pt = bubble[i];
    vals[pt.panAng][pt.tiltAng] += pt.val;
    counts[pt.panAng][pt.tiltAng] += 1;
  }

  for(int i = 0; i < 360; i++){
    for(int j = 0; j< 360; j++){
      if(vals[i][j] != 0)  {
        bubblePoint pt;

        pt.panAng = i;
        pt.tiltAng = j;

        if(counts[i][j] > 1){
          pt.val = vals[i][j]/counts[i][j];
        }
        result.push_back(pt);
      }
    }
  }
  return result;
}

DFCoefficients bubbleProcess::calculateDFCoefficients(const std::vector<bubblePoint>& bubble)
{
  DFCoefficients dfcoeff;
  float f1,f2,rho;

  for(int i = 0 ; i < HARMONIC1; i++)
  {
    for(int j = 0 ; j < HARMONIC2; j++)
    {
      dfcoeff.a[i][j] = 0;
      dfcoeff.b[i][j] = 0;
      dfcoeff.c[i][j] = 0;
      dfcoeff.d[i][j] = 0;
    }
  }

  for(uint i = 0; i < bubble.size(); i++)
  {
    f1 = bubble[i].panAng;
    f2 = bubble[i].tiltAng;
    rho = bubble[i].val + RHO_0;

    for (int h1=0;h1<HARMONIC1;h1++)
    {
      for (int h2=0;h2<HARMONIC2;h2++)
      {
        dfcoeff.a[h1][h2] += rho*cos(h1*f1*M_PI/180)*cos(h2*f2*M_PI/180)*4;
        dfcoeff.b[h1][h2] += rho*sin(h1*f1*M_PI/180)*cos(h2*f2*M_PI/180)*4;
        dfcoeff.c[h1][h2] += rho*cos(h1*f1*M_PI/180)*sin(h2*f2*M_PI/180)*4;
        dfcoeff.d[h1][h2] += rho*sin(h1*f1*M_PI/180)*sin(h2*f2*M_PI/180)*4;
      }
    }
  }
  return dfcoeff;
}

cv::Mat bubbleProcess::calculateInvariantsMat(DFCoefficients coeff)
{
  //Reconstruct: result matrix is transposed and unnecessary operations removed in next step.
  cv::Mat result(HARMONIC1*HARMONIC2,1,CV_32FC1);
  for (int h1=0;h1<HARMONIC1;h1++)
  {
    for (int h2=0;h2<HARMONIC2;h2++)
    {
      float val = 0;
      if (h1==0)
      val =coeff.a[h1][h2]*coeff.a[h1][h2]+coeff.c[h1][h2]*coeff.c[h1][h2];
      else
      val =coeff.a[h1][h2]*coeff.a[h1][h2]+coeff.b[h1][h2]*coeff.b[h1][h2]+coeff.c[h1][h2]*coeff.c[h1][h2]+coeff.d[h1][h2]*coeff.d[h1][h2];
      result.at<float>(HARMONIC2*h1+h2,0) = val;
    }
  }
  return result;
}
