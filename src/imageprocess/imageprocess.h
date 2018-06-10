#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#include <opencv2/opencv.hpp>
#include <QString>

class ImageProcess
{
public:

    ImageProcess();
    static void scaleResponse(cv::Mat &response);
    static void readFilters(QString folderName, std::vector<int> filterIds);
    static std::vector<cv::Mat> applyFilters(cv::Mat singleChannelImage);
    static void generateChannelImage(const cv::Mat& rgbimage, int satLower, int valLower, int valUpper,cv::Mat &hueChannel,cv::Mat &valChannel);
};

#endif // IMAGEPROCESS_H
