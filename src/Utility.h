#ifndef UTILITY_H
#define UTILITY_H
#include <opencv2/opencv.hpp>

using namespace cv;

// STATUS OF BASEPOINT
// 0: normal
// 1: uninformative
// 2: incoherent

class BasePoint
{
public:
    BasePoint();
    //BasePoint(const BasePoint& bp);
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

class Place
{

public:

    Place();
    Place(int id);
    uint id;
    Mat memberIds;
    std::vector<BasePoint> members;
    Mat memberInvariants;
    Mat meanInvariant;
    void calculateMeanInvariant();
};

class LearnedPlace
{

public:

    LearnedPlace();
    LearnedPlace(Place aPlace);
    LearnedPlace(int id);
    uint id;
    // This is a Mx1 vector that will store merged place ids
    cv::Mat memberPlaces;
    // This will be MXN matrix instead of Mx1 because multiple places can be merged in one learned place
    cv::Mat memberIds;
   // std::vector <std::vector<BasePoint> > members;
    Mat memberInvariants;
    Mat meanInvariant;
    void calculateMeanInvariant();
public:
    static int lpCounter;
};


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
    std::vector<BasePoint> cohMembers;
    // Holds the value of total incoherency inside the temporal window
    float totalDiff;
};

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
    vector<Place> detectedPlaces;
    uint placeID;

    std::vector<BasePoint> wholebasepoints;

private:

    TemporalWindow* tempwin;
    int img_counter;
    BasePoint previousBasePoint;
    BasePoint currentBasePoint;
    std::vector<BasePoint> basepointReservoir;
};


#endif // UTILITY_H
