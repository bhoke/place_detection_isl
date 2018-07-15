#ifndef BUBBLEPROCESS_H
#define BUBBLEPROCESS_H
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <QFile>
#include <opencv2/opencv.hpp>

#define RHO_0 1
#define HARMONIC1 10
#define HARMONIC2 10

struct bubblePoint{
    int panAng;
    int tiltAng;
    float val;
};
struct bubbleStatistics
{
    float mean;// mean of the bubble Surface
    float variance;// variance of the bubble Surface
};

struct DFCoefficients
{
    float a[HARMONIC1][HARMONIC2];
    float b[HARMONIC1][HARMONIC2];
    float c[HARMONIC1][HARMONIC2];
    float d[HARMONIC1][HARMONIC2];
};

class bubbleProcess
{
public:
    bubbleProcess();

    static DFCoefficients calculateDFCoefficients(const std::vector <bubblePoint>& bubble);
    static cv::Mat calculateInvariantsMat(DFCoefficients coeff);
    static bubbleStatistics calculateBubbleStatistics(const std::vector<bubblePoint>& bubble);

    // Reduces the number of points in a bubble by combining points falling in the same patch
    static std::vector<bubblePoint> reduceBubble(std::vector<bubblePoint>bubble);

    static std::vector<bubblePoint> convertGrayImage2Bub(cv::Mat grayImage);
    static void calculateImagePanAngles(int focalLengthPixels,int imageWidth);
    static void calculateImageTiltAngles(int focalLengthPixels,int imageHeight);

private:

    std::vector< std::vector<bubblePoint> > bubbles;
};

#endif // BUBBLEPROCESS_H
