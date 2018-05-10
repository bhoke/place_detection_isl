#include "PlaceDetector.h"

PlaceDetector::PlaceDetector()
{
    this->tempwin = 0;
    this->currentBasePoint.id = 0;
    this->previousBasePoint.id = 0;
    this->placeID = 1;
    currentPlace = new Place(this->placeID);
    this->twindow_counter = 1;
}
