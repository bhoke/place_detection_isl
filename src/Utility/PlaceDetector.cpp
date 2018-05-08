#include "PlaceDetector.h"

void PlaceDetector::processImage()
{
    if(!currentImage.empty())
    {
        timer.stop();

        Mat hueChannel,valChannel;
//        ImageProcess::generateChannelImage(currentImage,satLower,valLower,valUpper,hueChannel,valChannel);
        ImageProcess::generateChannelImage(currentImage,satLower,valLower,valUpper,hueChannel,valChannel);
        Mat hueChannelFiltered;

        cv::medianBlur(hueChannel, hueChannelFiltered,3);
        std::vector<bubblePoint> hueBubble = bubbleProcess::convertGrayImage2Bub(hueChannelFiltered);
        std::vector<bubblePoint> reducedHueBubble = bubbleProcess::reduceBubble(hueBubble);

//        Mat valChannel= ImageProcess::generateChannelImage(currentImage,satLower,satUpper,valLower,valUpper);
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

            //if(!similar)
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
