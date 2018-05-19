#include "Place.h"

int LearnedPlace::lpCounter;
Place::Place()
{
    id = -1;
}

Place::Place(int id)
{
    this->id = id;
}

void Place::calculateMeanInvariant()
{
    cv::Mat wholeInvariants;

    for(size_t i = 0; i < this->memberBPs.size(); i++)
    {
        if(i == 0)
            wholeInvariants = this->memberBPs[i].intensityInvariants;
        else
            cv::hconcat(wholeInvariants,this->memberBPs[i].intensityInvariants,wholeInvariants);
    }

    this->memberInvariants = wholeInvariants.clone();
    cv::reduce(wholeInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
}

/************************************** LEARNED PLACE **********************************/

LearnedPlace::LearnedPlace()
{
    id = -1;
}

LearnedPlace::LearnedPlace(int id)
{
    this->id = id;
}

LearnedPlace::LearnedPlace (Place place)
{
    this->id = LearnedPlace::lpCounter;
    this->meanInvariant = place.meanInvariant;
    this->memberInvariants = place.memberInvariants;
    this-> memberBPIDs = place.memberBPIDs;

    lpCounter++;
}

void LearnedPlace::calculateMeanInvariant()
{
    cv::reduce(this->memberInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
}


/*************************************** SUBPLACE ********************************/

subPlace::subPlace()
{
    this->id = -1;
}

subPlace::subPlace(int id)
{
    this->id = id;
}

void subPlace::calculateMeanInvariant()
{
    cv::reduce(this->memberInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
}
