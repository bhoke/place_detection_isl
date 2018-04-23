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

    static void generateChannelImage(const Mat& rgbimage, int satLower, int valLower, int valUpper,cv::Mat &hueChannel,cv::Mat &valChannel);

};

#endif // IMAGEPROCESS_H
