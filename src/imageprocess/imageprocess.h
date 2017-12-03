#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#include <opencv2/opencv.hpp>
#include <QString>

using namespace cv;

class ImageProcess
{
public:

    ImageProcess();

    static Mat convertToIlluminationInvariant(const Mat& image, float lambda);

    static void scaleResponse(cv::Mat &response, std::pair<float,float> minMax, float newMin, float newMax);

    static void readFilter(QString fileName);

    static std::vector<Mat> applyFilters(Mat singleChannelImage);

    static Mat generateChannelImage(const Mat& rgbimage, int channelNo, int satLower, int satUpper, int valLower, int valUpper);

};

#endif // IMAGEPROCESS_H
