#include "bubble/bubbleprocess.h"
#include "imageprocess/imageprocess.h"
#include "database/databasemanager.h"
#include "Utility.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <QDir>
#include <QDebug>
#include <QDateTime>
#include <std_msgs/Bool.h>

namespace enc = sensor_msgs::image_encodings;

double compareHistHK(InputArray _H1, InputArray _H2, int method );

double compareHKCHISQR(cv::Mat input1, cv::Mat input2);

ros::Timer timer;
PlaceDetector detector;

DatabaseManager dbmanager;
std::vector<BasePoint> basepoints;

ros::Publisher placedetectionPublisher;
ros::Publisher filePathPublisher;

bool firsttime = true;

bool sendLastPlaceandShutdown = false;

QString mainDirectoryPath;
QString imagesPath;
QTextStream strim;
QFile rawInvariants;

bool createDirectories(QString previousMemoryPath)
{
    QDir dir(QDir::home());

    QString mainDirectoryName = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh:mm:ss");

    if(!dir.mkdir(mainDirectoryName)) return false;

    dir.cd(mainDirectoryName);

    mainDirectoryPath = dir.path();
    qDebug()<<"Main Directory Path"<<mainDirectoryPath;


    QDir mainDir(QDir::homePath().append("/").append(mainDirectoryName));

    QString imageDirectory = "images";

    if(!mainDir.mkdir(imageDirectory)) return false;

    mainDir.cd(imageDirectory);

    imagesPath = mainDir.path();

    qDebug()<<"Image directory path"<<imagesPath;

    QString databasepath = QDir::homePath();

    databasepath.append("/emptydb");

    QString detecplacesdbpath = databasepath;
    detecplacesdbpath.append("/detected_places.db");

    QFile file(detecplacesdbpath);

    if(file.exists())
    {
        QString newdir = mainDirectoryPath;
        newdir.append("/detected_places.db");
        QFile::copy(detecplacesdbpath,newdir);

        if(!dbmanager.openDB(newdir))
            return false;
        //   file.close();
    }
    else
        return false;

    // If we don't have a previous memory then create an empty memory
    if(previousMemoryPath.size() <= 1 || previousMemoryPath.isNull())
    {
        QString knowledgedbpath = databasepath;
        knowledgedbpath.append("/knowledge.db");

        QFile file2(knowledgedbpath);

        if(file2.exists())
        {
            QString newdir = mainDirectoryPath;
            QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
            // file.close();
        }
        else
            return false;

    }
    // If we have supplied a previous memory path, then open that db
    else
    {
        QString knowledgedbpath = previousMemoryPath;
        knowledgedbpath.append("/knowledge.db");

        QFile file2(knowledgedbpath);

        if(file2.exists())
        {
            QString newdir = mainDirectoryPath;
            QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
            // file.close();
        }
        else
            return false;
    }

    return true;
}

bool saveParameters(QString filepath)
{
    QString fullpath = filepath;

    fullpath.append("/PDparams.txt");

    QFile file(fullpath);

    if(file.open(QFile::WriteOnly))
    {
        QTextStream str(&file);

        str<<"tau_w "<<detector.tau_w<<"\n";
        str<<"tau_n "<<detector.tau_n<<"\n";
        str<<"tau_p "<<detector.tau_p<<"\n";
        str<<"tau_inv "<<detector.tau_inv<<"\n";
        str<<"tau_inv2 "<<detector.tau_inv2<<"\n";
        str<<"tau_avgdiff "<<detector.tau_avgdiff<<"\n";
        str<<"focal_length_pixels "<<detector.focalLengthPixels<<"\n";
        str<<"tau_val_mean "<<detector.tau_val_mean<<"\n";
        str<<"tau_val_var "<<detector.tau_val_var<<"\n";
        str<<"sat_lower "<<detector.tau_avgdiff<<"\n";
        str<<"sat_upper "<<detector.focalLengthPixels<<"\n";
        str<<"val_lower "<<detector.tau_val_mean<<"\n";
        str<<"val_upper "<<detector.tau_val_var<<"\n";
        str<<"debug_mode "<<detector.debugMode<<"\n";
        str<<"debug_filePath "<<QString::fromStdString(detector.debugFilePath)<<"\n";

        file.close();
    }
    else
    {
        qDebug()<<"Param File could not be opened for writing!!";
        return false;
    }
    return true;
}

void timerCallback(const ros::TimerEvent& event)
{
    detector.shouldProcess = false;
    detector.processImage();
}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    if(detector.shouldProcess)
    {
        Mat imm = cv_bridge::toCvCopy(original_image, enc::BGR8)->image;
        cv::Rect rect(0,0,imm.cols,(imm.rows/2));
        detector.currentImage = imm(rect);//cv_bridge::toCvCopy(original_image, enc::BGR8)->image;
        // detector.shouldProcess = false;
    }

    //cv::imshow("win",image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    //cv::waitKey(3);
}
void startStopCallback(const std_msgs::Int16 startstopSignal)
{
    // Start processing
    if(startstopSignal.data == 1)
    {
        detector.shouldProcess = true;

        if(firsttime)
        {
            firsttime = false;

            bool canCreateDir = false;

            if(!detector.usePreviousMemory)
            {
                canCreateDir = createDirectories("");
            }
            else
            {
                canCreateDir = createDirectories(QString::fromStdString(detector.previousMemoryPath));
            }

            if(canCreateDir)
            {
                qDebug()<<"Directories have been created successfully!!";

                std_msgs::String sstr;

                sstr.data = mainDirectoryPath.toStdString();

                //std::cout<<"Ssstr data: "<<sstr.data<<std::endl;
                QString rawInvariantsName = mainDirectoryPath;

                rawInvariantsName.append("/Place Indices.txt");
                rawInvariants.setFileName(rawInvariantsName);

                if(rawInvariants.open(QFile::WriteOnly))
                {
                    qDebug() << "Index File is Opened";
                    strim.setDevice(&rawInvariants);
                }

                saveParameters(mainDirectoryPath);

                filePathPublisher.publish(sstr);
            }
            else
            {
                qDebug()<<"Error!! Necessary directories could not be created!! Detector will not work!!";

                detector.shouldProcess = false;
                //  return -1;
            }
        }
    }
    // Stop the node
    else if(startstopSignal.data == -1)
    {
        sendLastPlaceandShutdown = true;

    }
    // Pause the node
    else if(startstopSignal.data == 0)
    {
        detector.shouldProcess = false;
    }

}

double compareHKCHISQR(Mat input1, Mat input2)
{
    if(input1.rows != input2.rows)
    {
        qDebug()<<"Comparison failed due to col size mismatch";
        return -1;
    }
    double summ  = 0;

    for(int i = 0; i < input1.rows; i++)
    {
        float in1 = input1.at<float>(i,0);
        float in2 = input2.at<float>(i,0);

        double mul = (in1-in2)*(in1-in2);

        double ss =  in1+in2;
        summ += mul/ss;
        //        qDebug()<< "CHISQR " <<mul<<ss<<summ;

    }

    return summ;
}

// For DEBUGGING: Writing invariants to a file
void writeInvariant(cv::Mat inv, int count)
{
    QString pathh = QDir::homePath();
    pathh.append("/invariants_").append(QString::number(count)).append(".txt");
    QFile file(pathh);

    if(file.open(QFile::WriteOnly))
    {
        QTextStream str(&file);

        for(int i = 0; i < inv.rows; i++)
        {
            str<<inv.at<float>(i,0)<<"\n";
        }
        file.close();
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "placeDetectionISL");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hints("compressed");

    // detector.tau_w = 1;
    // detector.tau_n = 1;
    // detector.tau_p = 20;
    // detector.tau_avgdiff = 0.45;
    // std::string camera_topic = "";
    int img_width;
    int img_height;
    std::string camera_topic;
    // detector.focalLengthPixels = 525;

    // detector.satLower = 30;
    // detector.satUpper = 230;
    // detector.valLower = 30;
    // detector.valUpper = 230;

    detector.noHarmonics = 10;
    detector.image_counter = 1;

    detector.shouldProcess = false;
    // detector.debugMode = false;
    // detector.usePreviousMemory = false;

    std_msgs::String filterPath;


    pnh.getParam("tau_w",detector.tau_w);
    pnh.getParam("tau_n",detector.tau_n);
    pnh.getParam("tau_p",detector.tau_p);
    pnh.getParam("tau_inv",detector.tau_inv);
    pnh.getParam("tau_inv2",detector.tau_inv2);
    pnh.getParam("tau_avgdiff",detector.tau_avgdiff);
    pnh.getParam("camera_topic",camera_topic);
    pnh.getParam("image_width",img_width);
    pnh.getParam("image_height",img_height);
    pnh.getParam("focal_length_pixels",detector.focalLengthPixels);
    pnh.getParam("tau_val_mean",detector.tau_val_mean);
    pnh.getParam("tau_val_var",detector.tau_val_var);
    pnh.getParam("sat_lower",detector.satLower);
    pnh.getParam("sat_upper",detector.satUpper);
    pnh.getParam("val_lower",detector.valLower);
    pnh.getParam("val_upper",detector.valUpper);
    /***** GET THE DEBUG MODE ****************/

    pnh.getParam("debug_mode",detector.debugMode);

    /*************GET DEBUG FILE PATH ************************/
    std_msgs::String file_path;

    pnh.getParam("file_path", detector.debugFilePath);

    //detector.debugFilePath = file_path.data;

    /***********************************************************/
    qDebug()<<"Saturation and Value thresholds"<<detector.satLower<<detector.satUpper<<detector.valLower<<detector.valUpper<<detector.tau_avgdiff;

    if(detector.debugMode)
    {
        qDebug()<<"Debug mode is on!! File Path"<<QString::fromStdString(detector.debugFilePath);
    }

    /****** GET THE USE PREVIOUS MEMORY PARAM ****/

    pnh.getParam("use_previous_memory",detector.usePreviousMemory);

    if(detector.usePreviousMemory)
    {
        pnh.getParam("previous_memory_path", detector.previousMemoryPath);
    }

    bubbleProcess::calculateImagePanAngles(detector.focalLengthPixels,img_width,img_height);
    bubbleProcess::calculateImageTiltAngles(detector.focalLengthPixels,img_width,img_height);

    QString basepath = QDir::homePath();
    basepath.append("/visual_filters");

    // QString basepath(filterPath.data.data());

    QString path(basepath);

    path.append("/filtre0.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre6.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre12.txt");
    qDebug()<<path;


    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;

    path.append("/filtre18.txt");
    qDebug()<<path;


    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;


    path.append("/filtre36.txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    image_transport::Subscriber imageSub = it.subscribe(camera_topic.data(), 1, imageCallback,hints);

    ros::Subscriber sssub = nh.subscribe("placeDetectionISL/nodecontrol",1, startStopCallback);

    placedetectionPublisher = nh.advertise<std_msgs::Int16>("placeDetectionISL/placeID",5);

    filePathPublisher = nh.advertise<std_msgs::String>("placeDetectionISL/mainFilePath",2);

    ros::Rate loop(50);

    while(ros::ok())
    {
        ros::spinOnce();

        loop.sleep();

        if(detector.shouldProcess)
        {
            if(!detector.debugMode)
            {
                detector.shouldProcess = false;
                detector.processImage();

                if(sendLastPlaceandShutdown)
                {
                    if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                    {

                        detector.currentPlace->calculateMeanInvariant();

                        // qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                        if(detector.currentPlace->memberIds.rows >= detector.tau_p){

                            dbmanager.insertPlace(*detector.currentPlace);

                            detector.detectedPlaces.push_back(*detector.currentPlace);

                            std_msgs::Int16 plID;
                            plID.data = detector.placeID;

                            placedetectionPublisher.publish(plID);

                            ros::spinOnce();

                            // loop.sleep();
                            strim << "Place ID: " << detector.currentPlace->id << "\n";
                            for(MatIterator_<int> it = detector.currentPlace->memberIds.begin<int>();
                                it != detector.currentPlace->memberIds.end<int>();++it)
                            {
                                strim << *it << "\n";
                            }

                            detector.placeID++;
                        }

                        delete detector.currentPlace;
                        detector.currentPlace = 0;

                    }

                    ros::shutdown();
                }
            }
            else
            {
                QString processingPerImageFilePath = mainDirectoryPath;

                processingPerImageFilePath = processingPerImageFilePath.append("/pperImage.txt");

                QFile file(processingPerImageFilePath);

                QTextStream strm;

                if(file.open(QFile::WriteOnly))
                {
                    qDebug()<<"Processing per Image file Path has been opened";
                    strm.setDevice(&file);

                }
                // FOR DEBUGGING
                QStringList filters;
                filters << "*.jpeg";
                QString path = QString::fromStdString(detector.debugFilePath);
                QDir dir(path);
                dir.setNameFilters(filters);
                QStringList files = dir.entryList(filters,QDir::Files,QDir::Name);
                uint fileCount = dir.count();

                for(uint i = 0; i < fileCount; i++)
                {
                    QString filePath = path + files.at(i);
                    Mat imm = imread(filePath.toStdString().data(),CV_LOAD_IMAGE_COLOR);
                    qint64 starttime = QDateTime::currentMSecsSinceEpoch();

                    //cv::Rect rect(0,0,imm.cols,imm.rows);
                    // detector.currentImage = imm(rect);

                    detector.currentImage = imm;

                    detector.processImage();

                    qint64 stoptime = QDateTime::currentMSecsSinceEpoch();

                    //qDebug()<<(float)(stoptime-starttime);

                    if(strm.device() != NULL)
                        strm<<(float)(stoptime-starttime)<<"\n";

                    ros::spinOnce();

                    loop.sleep();

                    if(!ros::ok())
                        break;
                    // qDebug()<<i;
                }


                if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                {
                    qDebug()<<"I am here";

                    detector.currentPlace->calculateMeanInvariant();

                    //qDebug()<<"Current place mean invariant: "<<detector.currentPlace->meanInvariant.rows<<detector.currentPlace->meanInvariant.cols<<detector.currentPlace->members.size();

                    if(detector.currentPlace->memberIds.rows >= detector.tau_p)
                    {
                        dbmanager.insertPlace(*detector.currentPlace);

                        detector.detectedPlaces.push_back(*detector.currentPlace);

                        strim << "Place ID: " << detector.currentPlace->id << "\n";
                        for(MatIterator_<int> it = (detector.currentPlace)->memberIds.begin<int>();
                            it != (detector.currentPlace)->memberIds.end<int>();++it)
                        {
                            strim << *it << "\n";
                        }

                        std_msgs::Int16 plID;
                        plID.data = detector.placeID;

                        placedetectionPublisher.publish(plID);

                        ros::spinOnce();

                        detector.placeID++;
                    }

                    delete detector.currentPlace;
                    detector.currentPlace = 0;
                    detector.shouldProcess = false;
                    ros::shutdown();
                } // end if detector.currentPlace

                file.close();

            } // end else
        } // end if detector.should Process
        //  qDebug()<<"New Place";
    } //  while(ros::ok())

    rawInvariants.close();
    /// Delete the current place
    if(detector.currentPlace)
    {
        delete detector.currentPlace;
        detector.currentPlace = 0;
    }

    /// Insert basepoints to the database
    if(detector.wholebasepoints.size()>0)
        dbmanager.insertBasePoints(detector.wholebasepoints);


    dbmanager.closeDB();

    return 0;
}

void PlaceDetector::processImage()
{
    if(!currentImage.empty())
    {
        timer.stop();
        //qint64 start = QDateTime::currentMSecsSinceEpoch();
        //Mat ii_image = ImageProcess::convertToIlluminationInvariant(currentImage,0.05);
        //qint64 end = QDateTime::currentMSecsSinceEpoch();
        //qDebug() <<"Illumination - Invariant Process Time:"<< (end-start) << "ms" << endl;

        Mat hueChannel= ImageProcess::generateChannelImage(currentImage,0,satLower,satUpper,valLower,valUpper);

        Mat hueChannelFiltered;

        cv::medianBlur(hueChannel, hueChannelFiltered,3);

        vector<bubblePoint> hueBubble = bubbleProcess::convertGrayImage2Bub(hueChannelFiltered,180);
        vector<bubblePoint> reducedHueBubble = bubbleProcess::reduceBubble(hueBubble);

        /************************** Perform filtering and obtain resulting mat images ***********************/
        //  Mat satChannel= ImageProcess::generateChannelImage(img,1,satLower,satUpper,valLower,valUpper);
        Mat valChannel= ImageProcess::generateChannelImage(currentImage,2,satLower,satUpper,valLower,valUpper);
        /*****************************************************************************************************/
        //if(image_counter == 1) std::cout << valChannel << "\n";
        /*************************** Convert images to bubbles ***********************************************/

        //  vector<bubblePoint> satBubble = bubbleProcess::convertGrayImage2Bub(satChannel,focalLengthPixels,255);
        vector<bubblePoint> valBubble = bubbleProcess::convertGrayImage2Bub(valChannel,255);
        //vector<bubblePoint> valBubble = bubbleProcess::convertGrayImage2Bub(currentImage,focalLengthPixels,255);
        /*****************************************************************************************************/

        /***************** Reduce the bubbles ********************************************************/
        //   vector<bubblePoint> reducedSatBubble = bubbleProcess::reduceBubble(satBubble);
        vector<bubblePoint> reducedValBubble = bubbleProcess::reduceBubble(valBubble);


        // Calculate statistics
        //  bubbleStatistics statsHue =  bubbleProcess::calculateBubbleStatistics(reducedHueBubble,180);
        // bubbleStatistics statsSat =  bubbleProcess::calculateBubbleStatistics(reducedSatBubble,255);
        bubbleStatistics statsVal = bubbleProcess::calculateBubbleStatistics(reducedValBubble,255);

        //qDebug()<<"Bubble statistics: "<<statsVal.mean<<statsVal.variance;

        currentBasePoint.avgVal = statsVal.mean;
        currentBasePoint.varVal = statsVal.variance;
        currentBasePoint.id = image_counter;
        QString imagefilePath = imagesPath;
        imagefilePath.append("/rgb_");
        imagefilePath.append(QString::number(image_counter)).append(".jpg");
        imwrite(imagefilePath.toStdString().data(),currentImage);

        //imwrite()
        currentBasePoint.status = 0;
        qDebug() << "Current Mean: " << statsVal.mean << "and" << statsVal.variance ;
        /*********************** WE CHECK FOR THE UNINFORMATIVENESS OF THE FRAME   *************************/
        if(statsVal.mean <= this->tau_val_mean || statsVal.variance <= this->tau_val_var)
        {

            //qDebug() << "Mean: " << statsVal.mean << "Variance: "  << statsVal.variance ;

            currentBasePoint.status = 1;

            //  this->shouldProcess = true;

            // If we don't have an initialized window then initialize
            if(!this->tempwin)
            {
                this->tempwin = new TemporalWindow();
                this->tempwin->tau_n = this->tau_n;
                this->tempwin->tau_w = this->tau_w;
                this->tempwin->startPoint = image_counter;
                this->tempwin->endPoint = image_counter;
                this->tempwin->id = twindow_counter;
                this->tempwin->totalDiff = 0.99;

                this->tempwin->members.push_back(currentBasePoint);
            }
            else
            {
                this->tempwin->endPoint = image_counter;
                this->tempwin->totalDiff += 0.99;
                this->tempwin->members.push_back(currentBasePoint);
            }

            ///  dbmanager.insertBasePoint(currentBasePoint);
            wholebasepoints.push_back(currentBasePoint);

            //  previousBasePoint = currentBasePoint;

            image_counter++;

            detector.currentImage.release();
            qDebug() << "Image " <<image_counter << "is uninformative";
            //  timer.start();

            detector.shouldProcess = true;

            return;
        }
        /***********************************  IF THE FRAME IS INFORMATIVE *************************************************/
        else
        {
            Mat totalInvariants;

            DFCoefficients dfcoeff = bubbleProcess::calculateDFCoefficients(reducedHueBubble,noHarmonics,noHarmonics);
            Mat hueInvariants = bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics, noHarmonics);

            totalInvariants = hueInvariants.clone();
            cv::Mat logTotal;

            Mat grayImage;
            cv::cvtColor(currentImage,grayImage,CV_BGR2GRAY);
            std::vector<Mat> sonuc = ImageProcess::applyFilters(grayImage);

            for(size_t j = 0; j < sonuc.size(); j++)
            {
                //if(image_counter == 1)
                    //std::cout << "Response of filter" << j << ":" << std::endl << sonuc[j] << std::endl;
                vector<bubblePoint> imgBubble = bubbleProcess::convertGrayImage2Bub(sonuc[j],255);

                vector<bubblePoint> resred = bubbleProcess::reduceBubble(imgBubble);

                DFCoefficients dfcoeff = bubbleProcess::calculateDFCoefficients(resred,noHarmonics,noHarmonics);

                Mat invariants = bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics,noHarmonics);
                if(j==-1) // Set this to negative value to use hue channel, set 0 otherwise
                    totalInvariants = invariants.clone();
                else
                    cv::hconcat(totalInvariants, invariants, totalInvariants);
            }

            //            cv::log(totalInvariants,logTotal);
            //            logTotal = logTotal/25;
            //            cv::transpose(logTotal,logTotal);

            cv::transpose(totalInvariants/1e8,logTotal);
            //std::cout << logTotal;
            //qint64 stop = QDateTime::currentMSecsSinceEpoch();

            //qDebug()<<"Bubble time"<<(stop-start);
            // TOTAL INVARIANTS N X 1 vector
            //logTotal = logTotal / 1e8;
//            for(int kk = 0; kk < logTotal.rows; kk++)
//            {
//                if(logTotal.at<float>(kk,0) < 0)
//                    logTotal.at<float>(kk,0) = 0.5;
//            }

            bool similar =false;
            // We don't have a previous base point
            currentBasePoint.id = image_counter;
            //currentBasePoint.invariants = totalInvariants;
            currentBasePoint.invariants = logTotal;

            if(previousBasePoint.id == 0)
            {
                previousBasePoint = currentBasePoint;
                currentPlace->members.push_back(currentBasePoint);

                /// dbmanager.insertBasePoint(currentBasePoint);
                wholebasepoints.push_back(currentBasePoint);
            }
            else
            {
                double result = compareHKCHISQR(currentBasePoint.invariants,previousBasePoint.invariants);

                // JUST FOR DEBUGGING-> WRITES INVARIANT TO THE HOME FOLDER
                //   writeInvariant(previousBasePoint.invariants,previousBasePoint.id);

                ///////////////////////////// IF THE FRAMES ARE COHERENT ///////////////////////////////////////////////////////////////////////////////////////////////////////
                qDebug() << "Result of coherency function for the " << previousBasePoint.id
                         <<" and " << currentBasePoint.id << ": " << result;
                if(result <= tau_inv && result > tau_inv2)
                {
                    ///  dbmanager.insertBasePoint(currentBasePoint);
                    wholebasepoints.push_back(currentBasePoint);
                    qDebug() << currentBasePoint.id << " and " << previousBasePoint.id << "are coherent \n";
                    /// If we have a temporal window
                    if(tempwin)
                    {
                        //qDebug() << "We had tempwin \n";
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {
                            tempwin->cohMembers.push_back(currentBasePoint);
                            basepointReservoir.push_back(currentBasePoint);
                        }
                        // Temporal window will not extend anymore, we should check whether it is really a temporal window or not
                        else
                        {
                            float area = this->tempwin->totalDiff/(tempwin->endPoint - tempwin->startPoint+1);
                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w && area>= tau_avgdiff)
                            {
                                //qDebug() << tempwin->startPoint << " - " << tempwin->endPoint << "is valid \n";
                                qDebug()<<"New Place";
                                currentPlace->calculateMeanInvariant();

                                //qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                                if(currentPlace->memberIds.rows >= tau_p){

                                    dbmanager.insertPlace(*currentPlace);

                                    std_msgs::Int16 plID;
                                    plID.data = this->placeID;

                                    strim << "Place ID: " << currentPlace->id << "\n";
                                    for(MatIterator_<int> it = currentPlace->memberIds.begin<int>(); it != currentPlace->memberIds.end<int>();++it)
                                    {
                                        strim << *it << "\n";
                                    }

                                    placedetectionPublisher.publish(plID);

                                    this->detectedPlaces.push_back(*currentPlace);

                                    this->placeID++;
                                }

                                delete currentPlace;
                                currentPlace = 0;

                                currentPlace = new Place(this->placeID);
                                qDebug()<< "Adding the basepoint "<< currentBasePoint.id << "to place" << currentPlace->id;
                                basepointReservoir.push_back(currentBasePoint);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();

                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published

                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size() ); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;
                                qDebug()<< "Adding basepoint "<< AB.back().id << "to place" << currentPlace->id;

                                basepointReservoir.clear();
                            }
                        }
                    }
                    else
                    {
                        currentPlace->members.push_back(currentBasePoint);
                        qDebug()<< "Adding basepoint "<< currentBasePoint.id << "to place" << currentPlace->id;
                    }
                }
                else if(result <= tau_inv2)
                {
                    qDebug() << "Skipping the image:" << image_counter << "\n" ;
                    similar = true;
                    image_counter++;
                    wholebasepoints.push_back(currentBasePoint);
                }
                ///////////////////////// IF THE FRAMES ARE INCOHERENT /////////////////////////////////////
                else
                {
                    //qDebug() << previousBasePoint.id<< " and " << currentBasePoint.id << "are incoherent \n";
                    currentBasePoint.status = 2;

                    ///    dbmanager.insertBasePoint(currentBasePoint);
                    wholebasepoints.push_back(currentBasePoint);

                    // If we don't have a temporal window create one
                    if(!tempwin)
                    {
                        qDebug() << "We create new tempwin \n";
                        tempwin = new TemporalWindow();
                        this->tempwin->tau_n = this->tau_n;
                        this->tempwin->tau_w = this->tau_w;
                        this->tempwin->startPoint = image_counter;
                        this->tempwin->endPoint = image_counter;
                        this->tempwin->id = twindow_counter;
                        this->tempwin->totalDiff +=result;
                        this->tempwin->members.push_back(currentBasePoint);

                    }
                    // add the basepoint to the temporal window
                    else
                    {
                        qDebug() << "We had tempwin(incoherent) \n";
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {
                            qDebug() << "We extend tempwin(incoherent) \n";
                            this->tempwin->endPoint = image_counter;

                            this->tempwin->members.push_back(currentBasePoint);

                            this->tempwin->totalDiff +=result;

                            basepointReservoir.clear();
                        }
                        else
                        {
                            qDebug() << "tempwin does not extend, we check if it is valid \n" ;
                            float avgdiff;

                            avgdiff = this->tempwin->totalDiff/(tempwin->endPoint - tempwin->startPoint+1);
                            qDebug() << "Avg Diff here" << avgdiff;
                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w && avgdiff >= tau_avgdiff)
                            {

                                currentPlace->calculateMeanInvariant();

                                //qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols;

                                if(currentPlace->memberIds.rows >= tau_p)
                                {
                                    qDebug()<<"New Place";
                                    dbmanager.insertPlace(*currentPlace);

                                    std_msgs::Int16 plID ;
                                    plID.data = this->placeID;

                                    placedetectionPublisher.publish(plID);

                                    //qDebug() << "**********"<<currentPlace->id << "\t" <<beginMember << "-" << endMember << "\n";

                                    strim << "Place ID: " << currentPlace->id << "\n";

                                    for(MatIterator_<int> it = currentPlace->memberIds.begin<int>(); it != currentPlace->memberIds.end<int>();++it)
                                    {
                                        strim << *it << "\n";
                                    }
                                    this->detectedPlaces.push_back(*currentPlace);

                                    this->placeID++;
                                }
                                delete currentPlace;
                                currentPlace = 0;
                                // this->placeID++;

                                //  cv::Mat result = DatabaseManager::getPlaceMeanInvariant(this->placeID-1);

                                //  qDebug()<<"Previous place mean invariant: "<<result.rows<<result.cols<<result.at<float>(50,0);

                                //  result = DatabaseManager::getPlaceMemberIds(this->placeID-1);

                                /*    for(int k = 0; k< result.rows; k++){

                                    qDebug()<<"Previous place members: "<<result.rows<<result.cols<<result.at<unsigned short>(k,0);
                                }*/

                                currentPlace = new Place(this->placeID);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();

                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published
                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                qDebug() << "Noisy Temporal Window Here";
                                basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size()); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;
                                basepointReservoir.clear();
                            }

                            tempwin = new TemporalWindow();
                            this->tempwin->tau_n = this->tau_n;
                            this->tempwin->tau_w = this->tau_w;
                            this->tempwin->startPoint = image_counter;
                            this->tempwin->endPoint = image_counter;
                            this->tempwin->id = twindow_counter;

                            this->tempwin->members.push_back(currentBasePoint);
                        }
                        /*     this->tempwin->endPoint = image_counter;

                          this->tempwin->members.push_back(currentBasePoint);

                          basepointReservoir.clear();*/
                    }
                }
                previousBasePoint = currentBasePoint;

                //////////////////////////////////////////////////////////////////////////////////////////////////
            } //IF PREVIOUS POINT ID != 0

            // DatabaseManager::insertInvariants(HUE_TYPE,frameNumber,invariants);
            //   qDebug()<<"Image Counter: "<<image_counter;
            if(!similar)
                image_counter++;

            //this->shouldProcess = true;

        } //IF INFORMATIVE



    } //IF CURRENT IMAGE != EMPTY

    this->currentImage.release();

    this->shouldProcess = true;

    //  timer.start();



} //end of processImage

PlaceDetector::PlaceDetector()
{

    this->tempwin = 0;
    this->currentBasePoint.id = 0;
    this->previousBasePoint.id = 0;
    this->placeID = 1;
    currentPlace = new Place(this->placeID);
    this->twindow_counter = 1;
}

bool TemporalWindow::checkExtensionStatus(uint currentID)
{
    if(currentID - this->endPoint <= tau_n)
    {
        return true;
    }

    return false;
}

double compareHistHK( InputArray _H1, InputArray _H2, int method )
{
    Mat H1 = _H1.getMat(), H2 = _H2.getMat();
    const Mat* arrays[] = {&H1, &H2, 0};
    Mat planes[2];
    NAryMatIterator it(arrays, planes);
    double result = 0;
    int j, len = (int)it.size;

    CV_Assert( H1.type() == H2.type() && H1.depth() == CV_32F );

    //double s1 = 0, s2 = 0, s11 = 0, s12 = 0, s22 = 0;

    CV_Assert( it.planes[0].isContinuous() && it.planes[1].isContinuous() );

    for( size_t i = 0; i < it.nplanes; i++, ++it )
    {
        const float* h1 = (const float*)it.planes[0].data;
        const float* h2 = (const float*)it.planes[1].data;
        len = it.planes[0].rows*it.planes[0].cols*H1.channels();


        for( j = 0; j < len; j++ )
        {
            double a = h1[j] - h2[j];
            double b =  h1[j] + h2[j];
            if( fabs(b) > DBL_EPSILON )
                result += a*a/b;
        }
    }
    return result;
}
