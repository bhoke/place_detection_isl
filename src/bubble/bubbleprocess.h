#ifndef BUBBLEPROCESS_H
#define BUBBLEPROCESS_H
//#include "globals.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <QFile>
//#include <QRgb>
#include <opencv2/opencv.hpp>

struct bubblePoint{

    int panAng;
    int tiltAng;
    double val;
};
struct bubbleStatistics
{
    // mean of the bubble Surface
    double mean;
    // variance of the bubble Surface
    double variance;
    // maximum value of the bubbble point
    double maxDist;

};

struct DFCoefficients
{
    std::vector< std::vector<float> > a;
    std::vector< std::vector<float> > b;
    std::vector< std::vector<float> > c;
    std::vector< std::vector<float> > d;
};

using std::vector;

class bubbleProcess : public QObject
{
    Q_OBJECT
public:

    bubbleProcess();

    static DFCoefficients calculateDFCoefficients(const std::vector <bubblePoint>& bubble, int harmonic1, int harmonic2);

    static  cv::Mat calculateInvariantsMat(DFCoefficients coeff, int harmonic1, int harmonic2);

    static bubbleStatistics calculateBubbleStatistics(const vector<bubblePoint>& bubble, float maxDist);

    // Round double to int
    static double round(double num);

    // Reduces the number of points in a bubble by combining points falling in the same patch
    static vector<bubblePoint> reduceBubble(vector<bubblePoint>bubble);

    // Reads the bubble
    static vector<bubblePoint> readBubble(QFile* file);

    static vector<bubblePoint> convertGrayImage2Bub(cv::Mat grayImage, float maxval);

    static vector<vector <int> > calculateImagePanAngles(int focalLengthPixels,int imageWidth,int imageHeight);

    static vector<vector <int> > calculateImageTiltAngles(int focalLengthPixels,int imageWidth,int imageHeight);

private:

    vector< vector<bubblePoint> > bubbles;
};

#endif // BUBBLEPROCESS_H
