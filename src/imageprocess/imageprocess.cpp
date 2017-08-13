#include "imageprocess.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QDateTime>

Mat orgImg;

Mat filter;

static Mat filterOrg;

static std::vector<Mat> filters;

ImageProcess::ImageProcess()
{

}

void ImageProcess::readFilter(QString fileName, int filterSize, bool transpose, bool save, bool show)
{

    filterOrg = Mat(filterSize,filterSize,CV_32FC1);

    QString dirr = fileName;

    /* QString str;

    str.setNum(filterNum);

    dirr.append(str);

    dirr.append(".txt"); */

    qDebug()<<"Dir is :"<<dirr;

    QFile file(dirr);

    if(!file.open(QFile::ReadOnly))
    {
        qDebug()<<"Error! filter "<<fileName<<"could not be read returning...";
        return;
    }

    QTextStream stream(&file);

    QString line = stream.readLine();

    double count = 0;

    double count2 = 0;

    while(line != NULL)
    {

        filterOrg.at<float>(count,count2) = line.toFloat();

        count++;


        if(count == filterSize){

            count2++;
            count = 0;

        }

        line = stream.readLine();

    }

    file.close();


    if(transpose)
        cv::transpose(filterOrg,filterOrg);

    cv::convertScaleAbs(filterOrg,filter,128,128);

    cv::Mat resizedFilter;

    cv::resize(filter,resizedFilter,resizedFilter.size(),5,5);

    if(show)
    {
        namedWindow("filter");

        imshow("filter",resizedFilter);

        waitKey();

        destroyWindow("filter");
    }

    if(save)
    {
        imwrite("filter.jpg",resizedFilter);
        qDebug()<<"Filter image saved";
    }
    filters.push_back(filterOrg);
}
std::vector<Mat> ImageProcess::applyFilters(Mat singleChannelImage)
{
    std::vector<Mat> results;
    //Reconsturct: 8-bit gray image is converted to 32-bit float in order to get rid of overflow after filtering
    singleChannelImage.convertTo(singleChannelImage,CV_32FC1);
    //Reconstruct: Maximum and minimum possible filter responses for scaling responses (Calculated with Matlab)
    std::pair<float,float> minMax[5] = {std::make_pair(-60506.34, 83692.17),
                                                 std::make_pair(-60761.93, 60761.93),
                                                 std::make_pair(-71329.80, 63222.63),
                                                 std::make_pair(-68296.54, 68296.54),
                                                 std::make_pair(-68296.54, 68296.54)};
    /// TODO: Create a function to automatize possible minimum and maximum filter responses
    for(uint i = 0 ; i < filters.size(); i++)
    {
        Mat copyImage = singleChannelImage.clone();

        Mat result = Mat::zeros(singleChannelImage.rows,singleChannelImage.cols,CV_32FC1);
        Mat blurred;

        cv::medianBlur(copyImage,blurred,3);

        cv::filter2D(blurred,result,result.depth(),filters[i]);
        //Reconstruct: scaleResponse function added here
        scaleResponse(result,minMax[i],-500,1000);
        results.push_back(result);
    }
    //cv::norm
    //std::cout << cv::norm(results[1],results[0]) << std::endl;
    return results;
}

void ImageProcess::scaleResponse(cv::Mat &response, std::pair<float,float> minMax, float newMin, float newMax){
    //Reconstruct:This function is added, since response of the filter was varying within a huge interval.
    //Also we get rid of the negative coefficients while obtaining DF Coefficients with this function
    response = (response - minMax.first) * (newMax - newMin) / (minMax.second - minMax.first) + newMin;
}

Mat ImageProcess::generateChannelImage(const Mat& rgbimage, int channelNo, int satLower, int satUpper, int valLower, int valUpper)
{
    Mat hsvImage;
    cv::cvtColor(rgbimage,hsvImage,CV_BGR2HSV);

    // channel_0 hue channel_1 saturation channel_2 value
    std::vector<Mat> channels;
    cv::split(hsvImage,channels);
    //Reconstruct: If hue channel is processed, do not change values, since it corresponds to another color
    if(channelNo == 0) return channels[0].clone();

    Mat result;
    result = Mat::zeros(rgbimage.rows,rgbimage.cols,CV_8UC1);

    for(int i = 0; i < rgbimage.rows; i++)
    {
        for(int j = 0; j < rgbimage.cols; j++)
        {
            uchar currentSat = channels[1].at<uchar>(i,j);
            uchar currentVal = channels[2].at<uchar>(i,j);

            if(currentVal > valLower && currentVal < valUpper)
            {
                if(currentSat > satLower && currentSat < satUpper)
                {
                    result.at<uchar>(i,j) = channels[channelNo].at<uchar>(i,j);
                }
            }
        }
    }
    return result;
}

Mat ImageProcess::convertToIlluminationInvariant(const Mat &image,float lambda)
{
    Mat intensity,f;
    cvtColor(image,intensity,CV_BGR2GRAY,1);
    int rowCount = intensity.rows;
    int colCount = intensity.cols;

    Mat wiener_x = lambda*Mat::eye(rowCount,rowCount,CV_32F);
    Mat wiener_y = lambda*Mat::eye(colCount,colCount,CV_32F);

    intensity.setTo(1,intensity == 0);
    intensity.convertTo(f,CV_32F);

    cv::log(f,f);
    Mat D_x = Mat::zeros(rowCount,rowCount,CV_32F);
    Mat D_y = Mat::zeros(colCount,colCount,CV_32F);

    int j=1;
    float* p;
    p = D_x.ptr<float>(0);
    p[0] = 1.0; p[1] = -1.0;
    p = D_x.ptr<float>(rowCount-1);
    p[rowCount-2] = -1.0; p[rowCount-1] = 1.0;
    for(int i=1;i < rowCount-1; ++i)
    {
        p = D_x.ptr<float>(i);
        p[j-1] = -1.0;
        p[j] = 2.0;
        p[++j] = -1.0;
    }

    p = D_y.ptr<float>(0);
    p[0] = 1.0; p[1] = -1.0;
    p = D_y.ptr<float>(colCount-1);
    p[colCount-2] = -1.0; p[colCount-1] = 1.0;
    j = 1;
    for(int i=1;i < colCount-1; ++i)
    {
        p = D_y.ptr<float>(i);
        p[j-1] = -1.0;
        p[j] = 2.0;
        p[++j] = -1.0;
    }
    Mat mu_x,mu_y;

    wiener_x = wiener_x + D_x;
    wiener_y = wiener_y + D_y;

    solve(wiener_x,f,mu_x,DECOMP_CHOLESKY);
    solve(wiener_y,f.t(),mu_y,DECOMP_CHOLESKY);

    Mat v_x = f - lambda*mu_x;
    Mat v_y = f - lambda*mu_y.t();

    Mat v = v_x + v_y;

    normalize(v,v,0,1,NORM_MINMAX);
    return v;
}
