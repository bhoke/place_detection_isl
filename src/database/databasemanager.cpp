#include "databasemanager.h"
#include "../Utility/Place.h"
#include "../Utility/TemporalWindow.h"
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QVariant>
#include <QDebug>
#include <QVector>
#include <QDataStream>

DatabaseManager::DatabaseManager(){
}

bool DatabaseManager::openDB(QString filePath, QString connectionName){

  db = QSqlDatabase::addDatabase("QSQLITE",connectionName);
  QString path = QDir::toNativeSeparators(filePath);

  db.setDatabaseName(path);
  return db.open();
}

bool DatabaseManager::openDB(QString filePath)
{
  if(!db.isOpen())
  {
    // Find QSLite driver
    db = QSqlDatabase::addDatabase("QSQLITE");

    QString path = QDir::toNativeSeparators(filePath);

    db.setDatabaseName(path);
    return db.open();
  }

  db.close();

  db.removeDatabase("QSQLITE");

  db = QSqlDatabase::addDatabase("QSQLITE");

  QString path = QDir::toNativeSeparators(filePath);

  db.setDatabaseName(path);
  return db.open();
}

QSqlError DatabaseManager::lastError()
{
  // If opening database has failed user can ask
  // error description by QSqlError::text()
  return db.lastError();
}

bool DatabaseManager::isOpen()
{
  return db.isOpen();
}

void DatabaseManager::closeDB()
{
  if(db.isOpen()) db.close();
}

bool DatabaseManager::insertBasePoints(const std::vector<BasePoint> basepoints){

  if(db.isOpen())
  {
    QSqlQuery query;

    query.prepare(QString("REPLACE INTO basepoint(id, avgVal, varVal, avgLas, varLas, invariants, status )" "VALUES(?, ?, ?, ?, ?, ?, ?)"));

    QVariantList ids;
    QVariantList avgVals;
    QVariantList varVals;
    QVariantList avgLass;
    QVariantList varLass;
    QVariantList arrs;
    QVariantList statuses;

    db.transaction();

    for(uint i = 0; i < basepoints.size(); i++)
    {
      QByteArray arr = mat2ByteArray(basepoints[i].intensityInvariants);
      ids<<basepoints[i].id;
      avgVals<<basepoints[i].avgVal;
      varVals<<basepoints[i].varVal;
      avgLass<<basepoints[i].avgLas;
      varLass<<basepoints[i].varLas;
      arrs<<arr;
      statuses<<basepoints[i].status;
    }

    query.addBindValue(ids);
    query.addBindValue(avgVals);
    query.addBindValue(varVals);
    query.addBindValue(avgLass);
    query.addBindValue(varLass);
    query.addBindValue(arrs);
    query.addBindValue(statuses);

    if (!query.execBatch()){
      qDebug() << query.lastError();
      return false;
    }

    db.commit();

    return true;

  }
  return false;
}

bool DatabaseManager::insertTemporalWindow(const TemporalWindow &twindow){

  if(db.isOpen())
  {
    QSqlQuery query;

    query.prepare(QString("REPLACE INTO temporalwindow(id, startpoint, endpoint)" "VALUES(?, ?, ?)"));

    query.addBindValue(twindow.id);
    query.addBindValue(twindow.startPoint);
    query.addBindValue(twindow.endPoint);

    bool ret = query.exec();

    return ret;
  }

  return false;
}

int DatabaseManager::getLearnedPlaceMaxID(){
  if(db.isOpen())
  {
    QSqlQuery query(QString("SELECT MAX(id) FROM learnedplace"),QSqlDatabase::database("knowledge"));

    query.next();

    return query.value(0).toInt();
  }
  else
  {
    qDebug() << "Database is not open, Ctrl+C to terminate";
    return 0;
  }
}

bool DatabaseManager::insertLearnedPlace(const LearnedPlace &learnedplace){
  QByteArray arr = mat2ByteArray(cv::Mat(learnedplace.memberPlaceIDs));

  QByteArray arr2 = mat2ByteArray(cv::Mat(learnedplace.memberBPIDs));

  QByteArray arr3 = mat2ByteArray(learnedplace.meanInvariant);

  QByteArray arr4 = mat2ByteArray(learnedplace.memberInvariants);


  if(db.isOpen()){

    QSqlQuery query(QSqlDatabase::database("knowledge"));

    query.prepare("REPLACE INTO learnedplace(id, memberPlaces, memberIds, meanInvariant, memberInvariants)"
    "VALUES(?, ?, ?, ?, ?)");

    query.addBindValue(learnedplace.id); // index 0
    query.addBindValue(arr); // index 1
    query.addBindValue(arr2); // index 2
    query.addBindValue(arr3); //index 3
    query.addBindValue(arr4); // index 4

    bool ret = query.exec();
    if(!ret)
    std::cout << "Warning: LearnedPlace object could not be written to database!" << std::endl;
    return ret;
  }
  std::cout << "Warning: LearnedPlace object could not be written to database!" << std::endl;
  return false;

}

LearnedPlace DatabaseManager::getLearnedPlace(int id){
  LearnedPlace place;

  if(db.isOpen())
  {
    QSqlQuery query(QString("SELECT * FROM learnedplace WHERE id = %1").arg(id), QSqlDatabase::database("knowledge"));

    query.next();

    int id = query.value(0).toInt(); // index 0

    QByteArray arr = query.value(1).toByteArray(); // index 1
    place.memberPlaceIDs = DatabaseManager::byteArray2Mat(arr);

    QByteArray arr2 = query.value(2).toByteArray(); //index 2
    place.memberBPIDs = DatabaseManager::byteArray2Mat(arr2);

    QByteArray arr3 = query.value(3).toByteArray(); //index 3
    place.meanInvariant = DatabaseManager::byteArray2Mat(arr3);

    QByteArray arr4 = query.value(3).toByteArray(); //index 4
    place.memberInvariants = DatabaseManager::byteArray2Mat(arr4);

    place.id = id;
  }
  return place;
}

bool DatabaseManager::insertPlace(const Place &place){
  QByteArray arr = mat2ByteArray(place.meanInvariant);
  QByteArray arr2 = mat2ByteArray(cv::Mat(place.memberBPIDs));
  QByteArray arr3 = mat2ByteArray(place.memberInvariants);

  if(db.isOpen())
  {
    QSqlQuery query;

    query.prepare(QString("REPLACE INTO place(id, meaninvariant, memberIds, memberInvariants)"
    "VALUES(?, ?, ?, ?)"));

    query.addBindValue(place.id);
    query.addBindValue(arr);
    query.addBindValue(arr2);
    query.addBindValue(arr3);

    bool ret = query.exec();

    return ret;
  }

  return false;
}

cv::Mat DatabaseManager::getPlaceMeanInvariant(int id){
  if(db.isOpen())
  {
    QSqlQuery query(QString("SELECT meaninvariant FROM place WHERE id = %1").arg(id));

    query.next();

    QByteArray array = query.value(0).toByteArray();

    return byteArray2Mat(array);
  }

  else
  {
    qDebug() << "Database is not open, Ctrl+C to terminate";
    return cv::Mat();
  }

}
cv::Mat DatabaseManager::getPlaceMemberIds(int id){
  if(db.isOpen())
  {
    QSqlQuery query(QString("SELECT memberIds FROM place WHERE id = %1").arg(id));

    query.next();

    QByteArray array = query.value(0).toByteArray();

    return byteArray2Mat(array);
  }
  else
  {
    qDebug() << "Database is not open, Ctrl+C to terminate";
    return cv::Mat();
  }
}

Place DatabaseManager::getPlace(int id){
  Place place;

  if(db.isOpen())
  {
    QSqlQuery query(QString("SELECT * FROM place WHERE id = %1").arg(id));

    query.next();

    int id = query.value(0).toInt();

    QByteArray array = query.value(1).toByteArray();
    place.meanInvariant = DatabaseManager::byteArray2Mat(array);
    QByteArray array2 = query.value(2).toByteArray();
    place.memberBPIDs = DatabaseManager::byteArray2Mat(array2);
    QByteArray array3 = query.value(3).toByteArray();
    place.memberInvariants = DatabaseManager::byteArray2Mat(array3);

    place.id = id;
  }
  return place;
}

QByteArray DatabaseManager::mat2ByteArray(const cv::Mat &image){
  QByteArray byteArray;
  QDataStream stream(&byteArray,QIODevice::WriteOnly);
  stream << image.type();
  stream << image.rows;
  stream << image.cols;
  const size_t data_size = image.cols * image.rows * image.elemSize();
  QByteArray data = QByteArray::fromRawData( (const char*)image.ptr(), data_size );
  stream << data;
  return byteArray;
}

cv::Mat DatabaseManager::byteArray2Mat(const QByteArray & byteArray){
  QDataStream stream(byteArray);
  int matType, rows, cols;
  QByteArray data;
  stream >> matType;
  stream >> rows;
  stream >> cols;
  stream >> data;
  cv::Mat mat(rows, cols, matType, (void*)data.data() );
  return mat.clone();
}

bool DatabaseManager::updateLearnedPlace(int idToBeUpdated, const LearnedPlace updatedPlace){
  QByteArray arr = mat2ByteArray(cv::Mat(updatedPlace.memberPlaceIDs));
  QByteArray arr2 = mat2ByteArray(cv::Mat(updatedPlace.memberBPIDs));
  QByteArray arr3 = mat2ByteArray(updatedPlace.meanInvariant);
  QByteArray arr4 = mat2ByteArray(updatedPlace.memberInvariants);

  if(db.isOpen()){

    QSqlQuery query(QSqlDatabase::database("knowledge"));

    // query.prepare(QString("UPDATE learnedplace SET(memberPlaces= %1 memberIds= %2 meanInvariant=%3 memberInvariants=%4 WHERE id=%5")));
    query.prepare("UPDATE learnedplace SET memberPlaces = :memberPlaces, memberIds = :memberIds,meanInvariant = :meanInvariant,memberInvariants = :memberInvariants WHERE id = :id");
    query.bindValue(":memberPlaces", arr);
    query.bindValue(":memberIds", arr2) ;
    query.bindValue(":meanInvariant", arr3);
    query.bindValue(":memberInvariants", arr4);
    query.bindValue(":id", idToBeUpdated);

    bool ret = query.exec();
    if(!ret)
    qDebug() << "Warning: LearnedPlace object could not be updated! \n" << query.lastError() ;
    return ret;
  }

  std::cout << "Warning: LearnedPlace database could not be opened!" << std::endl;
  return false;

}
