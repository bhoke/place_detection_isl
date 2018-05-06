#include "Place.h"

int LearnedPlace::lpCounter;
Place::Place()
{
    id = -1;
}

Place::Place(int id)
{
    this->id = id;
    this->memberIds = cv::Mat::zeros(1,1,CV_16UC1);
}

void Place::calculateMeanInvariant()
{
    cv::Mat wholeInvariants;

    for(size_t i = 0; i < this->members.size(); i++)
    {
        if(i == 0)
        {
            wholeInvariants = members[i].intensityInvariants;
        }
        else
        {
            cv::hconcat(wholeInvariants,members[i].intensityInvariants,wholeInvariants);
        }
    }

    this->memberInvariants = wholeInvariants.clone();
    cv::reduce(wholeInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
    this->memberIds = cv::Mat::zeros(this->members.size(),1,CV_32SC1);

    for(size_t i = 0; i < this->members.size(); i++)
        this->memberIds.at<unsigned short>(i,0) = this->members[i].id;
}

LearnedPlace::LearnedPlace()
{
    id = -1;
}

LearnedPlace::LearnedPlace(int id)
{
    this->id = id;
    this->memberIds = cv::Mat::zeros(1,1,CV_16UC1);
}

LearnedPlace::LearnedPlace (Place place)
{
    this->id = LearnedPlace::lpCounter;
    this->meanInvariant = place.meanInvariant;
    this->memberInvariants = place.memberInvariants;
    this-> memberIds = place.memberIds;

    if(this->memberPlaces.empty())
    {
        this->memberPlaces = cv::Mat(1,1,CV_16UC1);
        this->memberPlaces.at<unsigned short>(0,0) = (unsigned short)place.id;
    }

    lpCounter++;
}

void LearnedPlace::calculateMeanInvariant()
{
    cv::reduce(this->memberInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
}
