#ifndef BASEPOINT_H
#define BASEPOINT_H

#include <opencv2/core/core.hpp>

// STATUS OF BASEPOINT
// 0: normal
// 1: uninformative
// 2: incoherent

class BasePoint
{
public:
    BasePoint();
    void operator = (const BasePoint& bp);
    uint id;
    float avgVal;
    float varVal;
    float avgLas;
    float varLas;
    cv::Mat intensityInvariants;
    cv::Mat hueInvariants;
    int status;
};

#endif
