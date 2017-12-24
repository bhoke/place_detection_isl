#include "Utility.h"
Place::Place()
{
    id = -1;

}

BasePoint::BasePoint(){
    this->invariants = Mat();
    this->id = 0;
}
//Reconstruct: Operator overloading created, since it was problematic in terms of invariants
void BasePoint::operator = (const BasePoint& bp)
{
    this->id = bp.id;
    this->avgVal = bp.avgVal;
    this->varVal = bp.varVal;
    this->avgLas = bp.avgLas;
    this->varLas = bp.varLas;
    this->invariants = bp.invariants.clone();
    this->status = bp.status;
}

Place::Place(int id)
{
    this->id = id;
    this->memberIds = cv::Mat::zeros(1,1,CV_16UC1);
}

void Place::calculateMeanInvariant()
{
    Mat wholeInvariants;

    for(uint i = 0; i < this->members.size(); i++)
    {
        if(i == 0)
        {
            wholeInvariants = members.at(i).invariants;
        }
        else
        {
            cv::hconcat(wholeInvariants,members.at(i).invariants,wholeInvariants);
        }

    }

    this->memberInvariants = wholeInvariants.clone();

    // Mat avg;

    cv::reduce(wholeInvariants,this->meanInvariant,1,CV_REDUCE_AVG);

    this->memberIds = cv::Mat::zeros(this->members.size(),1,CV_32SC1);

    for(uint i = 0; i < this->members.size(); i++)
    {
        this->memberIds.at<unsigned short>(i,0) = this->members[i].id;

    }

    // this->meanInvariant = avg;
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

void LearnedPlace::calculateMeanInvariant()
{
    cv::reduce(this->memberInvariants,this->meanInvariant,1,CV_REDUCE_AVG);
}

TemporalWindow::TemporalWindow()
{
    startPoint = 0;
    endPoint = 0;
    tau_w = 0;
    tau_n = 0;
    id = -1;
    totalDiff = 0.0;
}
