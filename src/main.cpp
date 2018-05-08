#include "bubble/bubbleprocess.h"
#include "Utility/PlaceDetector.h"
#include "imageprocess/imageprocess.h"
#include "database/databasemanager.h"
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

    QString databasepath = QDir::homePath() + "/emptydb";
    QString detectedPlacesdbpath = databasepath + "/detected_places.db";

    QFile file(detectedPlacesdbpath);

    if(file.exists())
    {
        QString newdir = mainDirectoryPath + "/detected_places.db";
        QFile::copy(detectedPlacesdbpath,newdir);

        if(!dbmanager.openDB(newdir))
            return false;
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
        str<<"sat_lower "<<detector.satLower<<"\n";
        str<<"val_lower "<<detector.valLower<<"\n";
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

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    if(detector.shouldProcess)
    {
        Mat imm = cv_bridge::toCvCopy(original_image, enc::BGR8)->image;
        cv::Rect rect(0,0,imm.cols,(imm.rows/2));
        detector.currentImage = imm(rect);//cv_bridge::toCvCopy(original_image, enc::BGR8)->image;
        // detector.shouldProcess = false;
    }
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

            if(createDirectories(QString::fromStdString(detector.previousMemoryPath)))
            {
                qDebug()<<"Directories have been created successfully!!";

                std_msgs::String sstr;

                sstr.data = mainDirectoryPath.toStdString();
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
            }
        }
    }

    else if(startstopSignal.data == -1) sendLastPlaceandShutdown = true; // Stop the node
    else if(startstopSignal.data == 0) detector.shouldProcess = false;  // Pause the node
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
    }
    return summ;
}

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

    int img_width;
    int img_height;
    std::string camera_topic;
    detector.image_counter = 1;

    detector.shouldProcess = false;

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
    pnh.getParam("val_lower",detector.valLower);
    detector.valUpper = 250;
    /***** GET THE DEBUG MODE ****************/

    pnh.getParam("debug_mode",detector.debugMode);
    pnh.getParam("file_path", detector.debugFilePath);

    qDebug()<<"Saturation and Value thresholds"<<detector.satLower<<detector.valLower<<detector.valUpper<<detector.tau_avgdiff;

    if(detector.debugMode) qDebug()<<"Debug mode is on!! File Path"<<QString::fromStdString(detector.debugFilePath);

    pnh.getParam("use_previous_memory",detector.usePreviousMemory);

    if(detector.usePreviousMemory) pnh.getParam("previous_memory_path", detector.previousMemoryPath);
    else detector.previousMemoryPath = "";

    bubbleProcess::calculateImagePanAngles(detector.focalLengthPixels,img_width);
    bubbleProcess::calculateImageTiltAngles(detector.focalLengthPixels,img_height);

    QString filterPath = QDir::homePath() + "/visual_filters";
    std::vector<int> filtersToRead = {0,6,12,18,36};
    ImageProcess::readFilters(filterPath,filtersToRead);

    cv::destroyAllWindows();

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
            if(!detector.debugMode) // Live Robot Mode
            {
                detector.shouldProcess = false;
                detector.processImage();

                if(sendLastPlaceandShutdown)
                {
                    if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                    {

                        detector.currentPlace->calculateMeanInvariant();
                        if(detector.currentPlace->memberIds.rows >= detector.tau_p){

                            dbmanager.insertPlace(*detector.currentPlace);

                            detector.detectedPlaces.push_back(*detector.currentPlace);

                            std_msgs::Int16 plID;
                            plID.data = detector.placeID;

                            placedetectionPublisher.publish(plID);

                            ros::spinOnce();

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
            else // Debug(Database) Mode
            {
                QStringList extensions;
                extensions << "*.jpeg";
                QString path = QString::fromStdString(detector.debugFilePath);
                QDir dir(path);
                dir.setNameFilters(extensions);
                QStringList files = dir.entryList(extensions,QDir::Files,QDir::Name);
                uint fileCount = dir.count();

                for(uint i = 0; i < fileCount; i++)
                {
                    QString filePath = path + files.at(i);
                    Mat imm = imread(filePath.toStdString().data(),CV_LOAD_IMAGE_COLOR);
                    //cv::Rect rect(0,0,imm.cols,imm.rows);
                    // detector.currentImage = imm(rect);
                    detector.currentImage = imm;
                    detector.processImage();
                    ros::spinOnce();

                    loop.sleep();

                    if(!ros::ok())
                        break;
                }

                if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                {
                    qDebug()<<"Evaluate last place and shutdown";

                    detector.currentPlace->calculateMeanInvariant();

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
            } // end Debug Mode
        } // end if detector.should Process
    } //  while(ros::ok())

    rawInvariants.close();
    if(detector.currentPlace)
    {
        delete detector.currentPlace;
        detector.currentPlace = 0;
    }

    // Insert basepoints to the database
    if(detector.wholebasepoints.size()>0)
        dbmanager.insertBasePoints(detector.wholebasepoints);

    dbmanager.closeDB();

    return 0;
} // End of main()
