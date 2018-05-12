#include "bubble/bubbleprocess.h"
#include "Utility/PlaceDetector.h"
#include "imageprocess/imageprocess.h"
#include "database/databasemanager.h"
#include <opencv2/ml/ml.hpp>
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

  QDir mainDir = QDir::homePath() + "/" + mainDirectoryName;

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

    QString knowledgedbpath = databasepath + "/knowledge.db";
    QFile file2(knowledgedbpath);

    if(file2.exists())
    {
      QString newdir = mainDirectoryPath;
      QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
    }
    else
    return false;
  }
  // If we have supplied a previous memory path, then open that db
  else
  {
    QString knowledgedbpath = previousMemoryPath + "/knowledge.db";
    QFile file2(knowledgedbpath);

    if(file2.exists())
    {
      QString newdir = mainDirectoryPath;
      QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
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

void PlaceDetector::processImage()
{
  if(!currentImage.empty())
  {
    timer.stop();

    cv::Mat hueChannel,valChannel,hueChannelFiltered;
    ImageProcess::generateChannelImage(currentImage,satLower,valLower,valUpper,hueChannel,valChannel);
    cv::medianBlur(hueChannel, hueChannelFiltered,3);
    std::vector<bubblePoint> hueBubble = bubbleProcess::convertGrayImage2Bub(hueChannelFiltered);
    std::vector<bubblePoint> reducedHueBubble = bubbleProcess::reduceBubble(hueBubble);
    std::vector<bubblePoint> valBubble = bubbleProcess::convertGrayImage2Bub(valChannel);
    std::vector<bubblePoint> reducedValBubble = bubbleProcess::reduceBubble(valBubble);

    bubbleStatistics statsVal = bubbleProcess::calculateBubbleStatistics(reducedValBubble);

    currentBasePoint.avgVal = statsVal.mean;
    currentBasePoint.varVal = statsVal.variance;
    currentBasePoint.id = image_counter;
    QString imagefilePath = imagesPath + "/rgb_" + QString::number(image_counter) + ".jpg";
    imwrite(imagefilePath.toStdString().data(),currentImage);
    currentBasePoint.status = 0;
    //qDebug() << "Current Mean: " << statsVal.mean << "and" << statsVal.variance ;
    /*********************** WE CHECK FOR THE UNINFORMATIVENESS OF THE FRAME   *************************/
    if(statsVal.mean <= this->tau_val_mean || statsVal.variance <= this->tau_val_var)
    {
      currentBasePoint.status = 1;

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
      // Hue Channel Processing
      DFCoefficients hueDFC = bubbleProcess::calculateDFCoefficients(reducedHueBubble);
      cv::Mat hueInvariants = bubbleProcess::calculateInvariantsMat(hueDFC);

      // Intensity Channel Processing
      cv::Mat grayImage;
      cv::cvtColor(currentImage,grayImage,CV_BGR2GRAY);
      std::vector<Mat> filteredVals = ImageProcess::applyFilters(grayImage);
      size_t invariantSize = filteredVals.size()*HARMONIC1*HARMONIC2;
      cv::Mat intensityInvariants;
      intensityInvariants.reserve(invariantSize);
      DFCoefficients intensityDFC;
      std::vector<bubblePoint> imgBubble,reducedBubble;

      for(size_t j = 0; j < filteredVals.size(); j++)
      {
        imgBubble = bubbleProcess::convertGrayImage2Bub(filteredVals[j]);
        reducedBubble = bubbleProcess::reduceBubble(imgBubble);
        intensityDFC = bubbleProcess::calculateDFCoefficients(reducedBubble);
        Mat invariants = bubbleProcess::calculateInvariantsMat(intensityDFC);
        intensityInvariants.push_back(invariants);
      }
      //bool similar = false;
      const float normFactor_int = 1e+08f,normFactor_hue = 1e+08f;

      Mat normalizedIntInvariant =  intensityInvariants / normFactor_int;
      Mat normalizedHueInvariant =  hueInvariants / normFactor_hue;
      currentBasePoint.intensityInvariants = normalizedIntInvariant.clone();
      currentBasePoint.hueInvariants = normalizedHueInvariant;

      // We don't have a previous base point
      if(previousBasePoint.id == 0)
      {
        previousBasePoint = currentBasePoint;
        currentPlace->members.push_back(currentBasePoint);
        wholebasepoints.push_back(currentBasePoint);
      }
      else
      {
        double intensityCoh = compareHKCHISQR(currentBasePoint.intensityInvariants,previousBasePoint.intensityInvariants);
        double hueCoh = compareHKCHISQR(currentBasePoint.hueInvariants,previousBasePoint.hueInvariants);
        ///////////////////////////// IF THE FRAMES ARE COHERENT ///////////////////////////////////////////////////////////////////////////////////////////////////////
        std::cout << "Result of intensity coherency function for the " << previousBasePoint.id
        <<" and " << currentBasePoint.id << ": " <<intensityCoh << std::endl;
        std::cout << "Result of hue coherency function for the " << previousBasePoint.id
        <<" and " << currentBasePoint.id << ": " <<hueCoh << std::endl;
        if(intensityCoh <= tau_inv && hueCoh <= 50) //&& result > tau_inv2)
        {
          ///  dbmanager.insertBasePoint(currentBasePoint);
          wholebasepoints.push_back(currentBasePoint);
          qDebug() << currentBasePoint.id << " and " << previousBasePoint.id << "are coherent \n";
          /// If we have a temporal window
          if(tempwin)
          {
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
              if(tempwin->endPoint - tempwin->startPoint >= tau_w )//&& area>= tau_avgdiff)
              {
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
                //qDebug()<< "Adding basepoint "<< AB.back().id << "to place" << currentPlace->id;

                basepointReservoir.clear();
              }
            }
          }
          else
          {
            currentPlace->members.push_back(currentBasePoint);
            //qDebug()<< "Adding basepoint "<< currentBasePoint.id << "to place" << currentPlace->id;
          }
        } // COHERENT
        // else if(result <= tau_inv2)
        // {
        //     qDebug() << "Skipping the image:" << image_counter << "\n" ;
        //     similar = true;
        //     image_counter++;
        //     wholebasepoints.push_back(currentBasePoint);
        // }
        ///////////////////////// IF THE FRAMES ARE INCOHERENT /////////////////////////////////////
        else
        {
          currentBasePoint.status = 2;
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
            this->tempwin->totalDiff +=intensityCoh;
            this->tempwin->members.push_back(currentBasePoint);

          }
          // add the basepoint to the temporal window
          else
          {
            // Temporal window will extend, we are still looking for the next incoming frames
            if(tempwin->checkExtensionStatus(currentBasePoint.id))
            {
              this->tempwin->endPoint = image_counter;

              this->tempwin->members.push_back(currentBasePoint);

              this->tempwin->totalDiff +=intensityCoh;

              basepointReservoir.clear();
            }
            else
            {
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
          }
        } //INCOHERENT
        previousBasePoint = currentBasePoint;
        //////////////////////////////////////////////////////////////////////////////////////////////////
      } //IF PREVIOUS POINT ID != 0
      image_counter++;
    } //IF INFORMATIVE
  } //IF CURRENT IMAGE != EMPTY
  this->currentImage.release();
  this->shouldProcess = true;
} //end of processImage
