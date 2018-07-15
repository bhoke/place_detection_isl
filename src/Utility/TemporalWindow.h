#ifndef TWIN_H
#define TWIN_H
#include "BasePoint.h"

class TemporalWindow
{
public:
    TemporalWindow();
    bool checkExtensionStatus(uint currentID);
    int id;
    int startPoint;
    int endPoint;
    int tau_w;
    int tau_n;
    std::vector<BasePoint> members;
    std::vector<BasePoint> cohMembers;   // Holds the value of total incoherency inside the temporal window
};

#endif
