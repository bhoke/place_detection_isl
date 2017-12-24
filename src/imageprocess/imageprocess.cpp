#include "imageprocess.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QDateTime>

Mat orgImg;

Mat filter;

static std::vector<Mat> filters;

ImageProcess::ImageProcess()
{

}

void ImageProcess::readFilters(QString folderName, std::vector<int> filterIds)
{
    QString filterDir;
    QFile file;
    int filterSize;

    for(int i = 0; i < filterIds.size(); i++){
        filterDir = folderName + "/filtre" + QString::number(filterIds[i]) + ".txt";
        qDebug()<<"Filter directory is :"<<filterDir;
        file.setFileName(filterDir);

        if(!file.open(QFile::ReadOnly))
        {
            qDebug()<<"Error! filter "<<folderName<<"could not be read returning...";
            return;
        }

        QTextStream stream(&file);

        int numElems = 0;

        //    std::vector<float> filterElems;
        Mat_<float> filterOrg;
        QString line = stream.readLine();
        while(line != NULL)
        {
            filterOrg.push_back(line.toFloat());
            numElems++;
            line = stream.readLine();
        }
        filterSize = sqrt(numElems);

        bool perfectSquare = (filterSize*filterSize) == numElems ? true:false;

        if(!perfectSquare)
        {
            qDebug() << "Number of elements in the file is not a perfect square";
            return;
        }

        file.close();
        filterOrg = filterOrg.reshape(1,filterSize);
        scaleResponse(filterOrg);
        filters.push_back(filterOrg);
    }

}

std::vector<Mat> ImageProcess::applyFilters(Mat singleChannelImage)
{
    std::vector<Mat> results;
    //Reconsturct: 8-bit gray image is converted to 32-bit float in order to get rid of overflow after filtering
    singleChannelImage.convertTo(singleChannelImage,CV_32F);
    for(uint i = 0 ; i < filters.size(); i++)
    {
        Mat copyImage = singleChannelImage.clone();
        Mat result;
        //        Mat blurred;

        //        cv::medianBlur(copyImage,blurred,3);
        cv::filter2D(copyImage,result,CV_32F,filters[i]);
        //Reconstruct: scaleResponse function added here to scale each filter [rho/2, 3*rho/2]
        //        scaleResponse(filters[i]);
        results.push_back(result);
    }
    return results;
}

void ImageProcess::scaleResponse(cv::Mat &response)
{
    int nCols = response.cols;
    int nRows = response.rows;
    float val,minResponse = 0.0f ,maxResponse = 0.0f;

    if (response.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    float *p;
    for(int i = 0; i < nRows; ++i)
    {
        p = response.ptr<float>(i);
        for (int j = 0; j < nCols; ++j)
        {
            val = p[j];
            if (val < 0.0f )
                minResponse += 255.0f * val;
            else
                maxResponse += 255.0f * val;
        }
    }
    response = response / (maxResponse - minResponse);
}

Mat ImageProcess::generateChannelImage(const Mat& rgbimage, int channelNo, int satLower, int satUpper, int valLower, int valUpper)
{
    Mat hsvImage;
    cv::cvtColor(rgbimage,hsvImage,CV_BGR2HSV);

    // channel_0 hue channel_1 saturation channel_2 value
    std::vector<Mat> channels;
    cv::split(hsvImage,channels);

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
