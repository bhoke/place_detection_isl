#ifndef PLACE_H
#define PLACE_H
#include "BasePoint.h"

class Place
{
public:

    Place();
    Place(int id);
    uint id;
    cv::Mat memberIds;
    std::vector<BasePoint> members;
    cv::Mat memberInvariants;
    cv::Mat meanInvariant;
    void calculateMeanInvariant();
};

class LearnedPlace
{
//TODO: Add inheriatance for learnedPlace class
public:
    LearnedPlace();
    LearnedPlace(int id);
    LearnedPlace(Place aPlace);
    uint id;
    // This is a Mx1 vector that will store merged place ids
    cv::Mat memberPlaces;
    // This will be MXN matrix instead of Mx1 because multiple places can be merged in one learned place
    cv::Mat memberIds;
   // std::vector <std::vector<BasePoint> > members;
    cv::Mat memberInvariants;
    cv::Mat meanInvariant;
    void calculateMeanInvariant();
public:
    static int lpCounter;
};

#endif
