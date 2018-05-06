#ifndef PLACEDETECTOR_H
#define PLACEDETECTOR_H

#include "Place.h"
#include "TemporalWindow.h"

class PlaceDetector
{
public:

    PlaceDetector();
    PlaceDetector(int tau_w, int tau_n, int tau_p);
    void processImage();

    bool shouldProcess;
    bool isProcessing;
    bool debugMode;
    bool usePreviousMemory;

    std::string previousMemoryPath;
    std::string debugFilePath;
    int debugFileNo;

    int tau_w;
    int tau_n;
    int tau_p;
    double tau_avgdiff;
    double tau_inv;
    double tau_inv2;

    int image_width;
    int image_height;

    int focalLengthPixels;

    double tau_val_mean;
    double tau_val_var;

    int satLower;
    int valLower;
    int valUpper;

    cv::Mat currentImage;

    uint image_counter;
    int  twindow_counter;

    Place* currentPlace;
    std::vector<Place> detectedPlaces;
    uint placeID;

    std::vector<BasePoint> wholebasepoints;

private:

    TemporalWindow* tempwin;
    int img_counter;
    BasePoint previousBasePoint;
    BasePoint currentBasePoint;
    std::vector<BasePoint> basepointReservoir;
};
#endif
