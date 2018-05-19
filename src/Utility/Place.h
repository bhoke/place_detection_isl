#ifndef PLACE_H
#define PLACE_H
#include "BasePoint.h"

class Place
{
public:
    Place();
    Place(int id);
    int id;
    std::vector<int> memberBPIDs;
    std::vector<BasePoint> memberBPs;
    cv::Mat memberInvariants;
    cv::Mat meanInvariant;
    void calculateMeanInvariant();
};

class LearnedPlace : public Place
{
//TODO: Add inheriatance for learnedPlace class
public:
    LearnedPlace();
    LearnedPlace(int id);
    LearnedPlace(Place aPlace);
    std::vector<int> memberPlaceIDs;
    cv::Mat memberInvariants;
    cv::Mat meanInvariant;
    void calculateMeanInvariant();
    static int lpCounter;
};

class subPlace: public Place
{
public:
    subPlace();
    subPlace(int id);
    void calculateMeanInvariant();
};

#endif
