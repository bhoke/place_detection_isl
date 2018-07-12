#ifndef DATABASEMANAGER_H
#define DATABASEMANAGER_H

#include "bubbleprocess.h"
#include <opencv2/opencv.hpp>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include <QFile>
#include <QDir>

#define LASER_TYPE 55
#define HUE_TYPE 56
#define SAT_TYPE 57
#define VAL_TYPE 58

class Place;
class TemporalWindow;
class BasePoint;
class LearnedPlace;
class Level;

class DatabaseManager
{
public:
    explicit DatabaseManager();
    bool openDB(QString filePath);

    // For accessing multiple databases
    bool openDB(QString filePath, QString connectionName);

    void closeDB();

    bool isOpen();

    bool insertInvariants(int type,int number, std::vector< std::vector<float> > invariants);

    std::vector<bubblePoint> readBubble(int type, int number);

    bool insertTemporalWindow(const TemporalWindow& twindow);

    bool insertBasePoint(const BasePoint& basepoint);

    bool insertBasePoints(const std::vector<BasePoint> basepoints);

    bool insertPlace(const Place& place);

    bool insertLearnedPlace(const LearnedPlace& learnedplace);

    bool updateLearnedPlace(int idToBeUpdated, const LearnedPlace updatedPlace);

    int getLearnedPlaceMaxID();

    LearnedPlace getLearnedPlace(int id);

    bool insertBDSTLevel(int id, const Level& aLevel);

    cv::Mat getPlaceMeanInvariant(int id);

    Place getPlace(int id);

    cv::Mat getPlaceMemberIds(int id);

    QByteArray mat2ByteArray(const cv::Mat &image);

    cv::Mat byteArray2Mat(const QByteArray & byteArray);

    QSqlError lastError();

private:
    QSqlDatabase db;


signals:

public slots:

};

#endif // DATABASEMANAGER_H
