#include "bubbleprocess.h"
#include <QStringList>
#include <QTextStream>
#include <math.h>
#include <QDebug>
double bubbleProcess::round(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

using std::vector;

QString bubblesRootDirectory;
QString threeDFilesRootDirectory;
QString writeBubblesRootDirectory;


QStringList bubblesFolderList;
QStringList bubblesfileList;
QStringList threeDfileList;

vector< vector<bubblePoint> > staticBubbles;

static vector<vector<int> > imagePanAngles;

static vector<vector<int> > imageTiltAngles;


bubbleProcess::bubbleProcess()
{
}


bubbleStatistics bubbleProcess::calculateBubbleStatistics(const vector<bubblePoint>& bubble, float maxDist)
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
    /*    imshow("bubbleArr",bubbleArr);
    cv::waitKey(0);
    cv::destroyAllWindows();*/

    cv::Scalar     mean;
    cv::Scalar     stddev;

    cv::meanStdDev(values,mean,stddev);

    result.maxDist = maxDist;

    //  result.mean = mean.val[0];

    result.variance = stddev.val[0]*stddev.val[0];

    return result;

    // qDebug()<<"mean: "<<mean.val[0]<< "var: "<<stddev.val[0]*stddev.val[0];
    //

}
vector<vector<int> > bubbleProcess::calculateImagePanAngles(int focalLengthPixels, int imageWidth, int imageHeight)
{
    vector<vector<int> > result(imageHeight, std::vector<int>(imageWidth));


    for(int i = 0; i < imageHeight; i++){

        for(int j = 0; j < imageWidth; j++ )
        {

            int deltax = imageWidth/2 - j;


            float pan = atan2((double)deltax,(double)focalLengthPixels);


            int panInt = (pan*180)/M_PI;


            if(panInt < 0)   panInt += 360;
            else if(panInt > 359)   panInt -=360;


            result[i][j] = panInt;

        }

    }

    if(imagePanAngles.size() > 0){

        for(uint i = 0; i < imagePanAngles.size(); i++){
            imagePanAngles[i].clear();

            imagePanAngles.clear();

            //  qDebug()<<imagePanAngles[0].size();
            //  qDebug()<<imagePanAngles.size();

        }

    }

    imagePanAngles.resize(imageHeight,std::vector<int> (imageWidth));

    imagePanAngles = result;


    // qDebug()<<"Image pan angles row size: "<<imagePanAngles[0].size();

    // qDebug()<<"Image pan angles total size: "<<imagePanAngles.size();

    return result;

}
vector<vector<int> > bubbleProcess::calculateImageTiltAngles(int focalLengthPixels, int imageWidth, int imageHeight)
{
    vector<vector<int> > result(imageHeight, std::vector<int>(imageWidth));


    for(int i = 0; i < imageHeight; i++){

        for(int j = 0; j < imageWidth; j++ )
        {

            int deltay = imageHeight/2 - i;

            float tilt = atan2((double)deltay,(double)focalLengthPixels);


            int tiltInt = (tilt*180)/M_PI;

            if(tiltInt < 0)tiltInt += 360;
            else if(tiltInt > 359) tiltInt -=360;

            result[i][j] = tiltInt;

            //result.at(i).at(j).pushback()

        }

    }

    if(imageTiltAngles.size() > 0){

        for(uint i = 0; i < imageTiltAngles.size(); i++){
            imageTiltAngles[i].clear();
        }

        imageTiltAngles.clear();

        //  qDebug()<<imageTiltAngles[0].size();
        // qDebug()<<imageTiltAngles.size();
    }

    imageTiltAngles.resize(imageHeight,std::vector<int>(imageWidth));

    imageTiltAngles = result;

    return result;

}
vector<bubblePoint> bubbleProcess::convertGrayImage2Bub(cv::Mat grayImage, float maxval)
{

    vector<bubblePoint> result;

    // if(imagePanAngles[12][12] == 0) calculateImagePanAngles(focalLengthPixels,grayImage.cols,grayImage.rows);

    // if(imageTiltAngles.size() == 0) calculateImageTiltAngles(focalLengthPixels,grayImage.cols,grayImage.rows);


    //int centerx = grayImage.cols/2;

    //int centery = grayImage.rows/2;

    for(int i = 0; i < grayImage.rows; i++)
    {
        for(int j = 0; j < grayImage.cols; j++)
        {
            /*    int deltax = centerx - j;

            int deltay = centery - i;

            float pan = atan2((double)deltax,(double)focalLengthPixels);

            float tilt = atan2((double)deltay,(double)focalLengthPixels);*/
            grayImage.convertTo(grayImage,CV_32FC1);
            float val = grayImage.at<float>(i,j)/maxval;
            //qDebug() << "Val convert: " << val<< " = " << grayImage.at<float>(i,j) << " / " << maxval;

            //            if(val > 0)
            //            {
            //    qDebug()<<pan<<" "<<tilt;

            bubblePoint pt;

            pt.panAng = imagePanAngles[i][j];

            pt.tiltAng = imageTiltAngles[i][j];

            /*   pt.panAng = pan*180/M_PI;

                if(pt.panAng < 0)pt.panAng += 360;
                else if(pt.panAng > 359) pt.panAng -=360;

                pt.tiltAng = tilt*180/M_PI;

                if(pt.tiltAng < 0)pt.tiltAng += 360;
                else if(pt.tiltAng > 359) pt.tiltAng -=360;*/
            pt.val = val;

            result.push_back(pt);
            //            }
        }
    }

    return result;
}

vector<bubblePoint> bubbleProcess::readBubble(QFile *file){

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
vector<bubblePoint> bubbleProcess::reduceBubble(std::vector<bubblePoint> bubble){

    vector<bubblePoint> result;

    double vals[360][360];

    double counts[360][360];

    for(int i = 0; i < 360; i++){

        for(int j = 0; j< 360; j++){

            vals[i][j] = 0;
            counts[i][j] = 0;

        }
    }

    for(ulong i = 0; i < bubble.size(); i++){

        bubblePoint pt;

        //		int simCount = 1;

        //		if(bubble[i].val < 1){
        pt = bubble[i];
        //double sum = bubble[i].val;
        //	pt.panAng = bubble[i].panAng;
        //	pt.tiltAng = bubble[i].tiltAng;

        vals[pt.panAng][pt.tiltAng] += pt.val;
        counts[pt.panAng][pt.tiltAng] += 1;
        /*
            for(long j = 0; j < bubble.size(); j++){

                if(bubble[j].val < 1 && bubble[i].panAng == bubble[j].panAng && bubble[i].tiltAng == bubble[j].tiltAng && i != j){

                    simCount++;

                    sum += bubble[j].val;

                    bubble[j].val = 1.1; // remove the pt from scanning


                } // end if

            } // end for*/

        //	pt.val = sum/simCount;

        //	result.push_back(pt);

        //	bubble[i].val = 1.1; // remove the pt from scanning



        //		}
    }

    for(int i = 0; i < 360; i++){

        for(int j = 0; j< 360; j++){

            if(vals[i][j] != 0)  {
                bubblePoint pt;

                pt.panAng = i;

                pt.tiltAng = j;

                if(counts[i][j] > 1){

                    pt.val = vals[i][j]/counts[i][j];
                    //qDebug("Vals: %f -- Counts: %f" , vals[i][j] , counts[i][j]);
                }

                result.push_back(pt);
                // counts[i][j] = 0;
            }
        }
    }


    return result;

}

void bubbleProcess::saveBubble(QFile *file, std::vector<bubblePoint> bubble){

    QTextStream stream(file);

    for(ulong i = 0; i < bubble.size(); i++){

        // if val is in normal ranges save it
        if(bubble[i].val < 1 && bubble[i].val > 0)
            stream<<(int)bubble[i].panAng<<" "<<(int)bubble[i].tiltAng<<" "<<(double)bubble[i].val<<"\n";
    }
}
