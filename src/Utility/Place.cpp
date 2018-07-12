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
  BasePoint bp;
  cv::Mat currentInvariant;
  size_t noBPs = this->memberBPs.size();
  this-> memberInvariants = cv::Mat(600,noBPs,CV_32FC1);
  //TODO: All terms contain 600(invarint size) should be adaptive to harmonics and filterCount

  for(int i = 0; i < noBPs; i++)
  {
    bp = this->memberBPs[i];
    cv::vconcat(bp.intensityInvariants,bp.hueInvariants,currentInvariant);
    currentInvariant.copyTo(this->memberInvariants.col(i));
    this->memberBPIDs.push_back(this->memberBPs[i].id);
  }
  std::cout << "memberInvariants.size() in calculateMeanInvariant() " << memberInvariants.size() << std::endl;
  cv::reduce(this->memberInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
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
  this-> memberPlaceIDs.push_back(place.id);

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
