#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#include <opencv2/opencv.hpp>
#include <QString>

using namespace cv;

class ImageProcess
{
public:

    ImageProcess();

    static void scaleResponse(Mat &response);

    static void readFilters(QString folderName, std::vector<int> filterIds);

    static std::vector<Mat> applyFilters(Mat singleChannelImage);

    static Mat generateChannelImage(const Mat& rgbimage, int channelNo, int satLower, int satUpper, int valLower, int valUpper);

};

#endif // IMAGEPROCESS_H
