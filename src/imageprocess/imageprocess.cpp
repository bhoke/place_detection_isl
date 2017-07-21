#include "imageprocess.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QDateTime>

Mat orgImg;

vector<Mat> channels;

Mat filter;

static Mat filterOrg;

static std::vector<Mat> filters;

static QString datasetPath;

ImageProcess::ImageProcess()
{

}
void ImageProcess::setDataSetPath(QString path){

    datasetPath = path;
}
QString ImageProcess::getDataSetPath(){


    return datasetPath;
}
void ImageProcess::setImage(Mat image)
{
    orgImg = image;

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

    for(uint i = 0 ; i < filters.size(); i++)
    {
        Mat copyImage = singleChannelImage.clone();

        Mat result = Mat::zeros(singleChannelImage.rows,singleChannelImage.cols,CV_8UC1);

        // cv::GaussianBlur(copyImage,copyImage,cv::Size(5,5),5,5);
        Mat blurred;

        cv::medianBlur(copyImage,blurred,3);

        cv::filter2D(blurred,result,result.depth(),filters[i]);
        results.push_back(result);

    }

    return results;

}
Mat ImageProcess::getFilter()
{
    return filterOrg;
}
Mat ImageProcess::loadImage(QString fileName, bool show)
{
    Mat result = imread(fileName.toStdString(),-1);

    if(show)
    {

        namedWindow("img");

        imshow("img",result);

        waitKey();

        destroyWindow("img");
    }
    if(!result.empty())
    {

        orgImg = result;

    }
    return result;

}
Mat ImageProcess::applyFilter(Mat singleChannelImage)
{
    Mat result = Mat::zeros(singleChannelImage.rows,singleChannelImage.cols,CV_8UC1);

    cv::GaussianBlur(singleChannelImage,singleChannelImage,cv::Size(5,5),5,5);

    cv::filter2D(singleChannelImage,result,result.depth(),filterOrg);

    //  cv::threshold(result,result,250,255,CV_THRESH_BINARY);

    /*    namedWindow("filterResult");

       namedWindow("orgImage");

       imshow("filterResult",result);

       imshow("orgImage",singleChannelImage);

      waitKey();

      destroyAllWindows();*/

    return result;


}
Mat ImageProcess::generateChannelImage(const Mat& rgbimage, int channelNo, int satLower, int satUpper, int valLower, int valUpper)
{
    Mat hsvimage;
    cv::cvtColor(rgbimage,hsvimage,CV_BGR2HSV);

    // channel_0 hue channel_1 saturation channel_2 value
    //  std::vector<Mat> channels;

    Mat result;

    result = Mat::zeros(rgbimage.rows,rgbimage.cols,CV_8UC1);

    cv::split(hsvimage,channels);
    // Mat mask;

    // cv::inRange(hsvimage,Scalar(0,satLower,valLower),Scalar(180,satUpper,valUpper),mask);

    for(int i = 0; i < rgbimage.rows; i++)
    {

        for(int j = 0; j < rgbimage.cols; j++)
        {

            //uchar hueval = channels[0].at<uchar>(i,j);

            uchar satval = channels[1].at<uchar>(i,j);

            uchar valval = channels[2].at<uchar>(i,j);

            /*   if(mask.at<uchar>(i,j) > 0)
            {
                 result.at<uchar>(i,j) = channels[channelNo].at<uchar>(i,j);

            }*/

            if(valval > valLower && valval < valUpper)
            {

                if(satval > satLower && satval < satUpper)
                {
                    //   if(hueval < 15 ) hueval = 180;
                    result.at<uchar>(i,j) = channels[channelNo].at<uchar>(i,j);
                }
            }
        }
    }

    return result;

}
Mat ImageProcess::generateHueImage(int satLower, int satUpper, int valLower, int valUpper)
{

    Mat result;

    if(!orgImg.empty())
    {

        Mat hsvimage;
        cv::cvtColor(orgImg,hsvimage,CV_BGR2HSV);

        // channel_0 hue channel_1 saturation channel_2 value
        //  std::vector<Mat> channels;


        result = Mat::zeros(orgImg.rows,orgImg.cols,CV_8UC1);

        cv::split(hsvimage,channels);

        for(int i = 0; i < orgImg.rows; i++)
        {

            for(int j = 0; j < orgImg.cols; j++)
            {

                uchar hueval = channels[0].at<uchar>(i,j);

                uchar satval = channels[1].at<uchar>(i,j);

                uchar valval = channels[2].at<uchar>(i,j);

                if(valval > valLower && valval < valUpper)
                {

                    if(satval > satLower && satval < satUpper)
                    {
                        if(hueval < 15 ) hueval = 180;

                        result.at<uchar>(i,j) = hueval;

                    }

                }

            }
        }
    }
    return result;

}
int ImageProcess::getFrameNumber(QString fullFilePath)
{
    int frameNumber = -1;

    /******** Try to find the name of the frame *****************/
    //QString name = filePathwithName;
    QString path = fullFilePath;

    QStringList list = path.split("/");

    QString name = list[list.size()-1];
    // int lastindex = path.lastIndexOf("/",0,Qt::CaseInsensitive);

    //   QString name = path.right(lastindex+1);

    //name = name.remove(filePath);

    int j = name.size();
    int k = 0;

    while(k < j)
    {
        QChar character = name.at(k);

        if(!character.isNumber())
        {
            name.remove(character);
            k = 0;
            j = name.size();
        }
        else
            k++;

    }
    /***********************************************************************/

    qDebug()<<"The number of the current frame is "<<name;

    if(name.size()>0)
        frameNumber = name.toInt();

    if(frameNumber == -1){

        qDebug()<<"Error!! Frame number could not be determined, returning...";
        return -1;
    }

    return frameNumber;
}
Mat ImageProcess::getImage()
{

    return orgImg;
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
