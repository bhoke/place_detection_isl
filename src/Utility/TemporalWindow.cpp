#include "TemporalWindow.h"

TemporalWindow::TemporalWindow()
{
    startPoint = 0;
    endPoint = 0;
    tau_w = 0;
    tau_n = 0;
    id = -1;
    totalDiff = 0.0;
}

bool TemporalWindow::checkExtensionStatus(uint currentID)
{
    if(currentID - this->endPoint <= tau_n)
        return true;

    return false;
}
