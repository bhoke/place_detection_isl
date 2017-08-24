#include "databasemanager.h"
#include "Utility.h"
//#include <createBDSTISL/src/bdst.h>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>
#include <QVariant>
#include <QDebug>
#include <QVector>


//static QSqlDatabase bubbledb;
//static QSqlDatabase invariantdb;

//static QSqlDatabase db;
static QVector<int> placeLabels;

DatabaseManager::DatabaseManager(QObject *parent) :
    QObject(parent)
{
}
bool DatabaseManager::openDB(QString filePath, QString connectionName)
{

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

bool DatabaseManager::deleteDB()
{
    // Close database
    db.close();

#ifdef Q_OS_LINUX
    // NOTE: We have to store database file into user home folder in Linux
    QString path(QDir::home().path());
    path.append(QDir::separator()).append("my.bubbledb.sqlite");
    path = QDir::toNativeSeparators(path);
    return QFile::remove(path);
#else

    // Remove created database binary file
    return QFile::remove("my.bubbledb.sqlite");
#endif

}
std::vector<bubblePoint> DatabaseManager::readBubble(int type, int number)
{
    QSqlQuery query(QString("select * from bubble where type = %1 and number = %2").arg(type).arg(number));

    std::vector<bubblePoint> bubble;

    while(query.next())
    {
        bubblePoint pt ;

        pt.panAng = query.value(3).toInt();

        pt.tiltAng = query.value(4).toInt();

        pt.val = query.value(5).toDouble();

        bubble.push_back(pt);

        //qDebug()<<"value is"<<query.value(4).toDouble();
    }
    // select * from bubble where number = 2 and type = 0


    return bubble;
}
bool DatabaseManager::insertBasePoints(const std::vector<BasePoint> basepoints)
{

    if(db.isOpen())
    {
        QSqlQuery query;

        query.prepare(QString("replace into basepoint values(?, ?, ?, ?, ?, ?, ?)"));

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
            QByteArray arr = mat2ByteArray(basepoints[i].invariants);
            ids<<basepoints[i].id;
            avgVals<<basepoints[i].avgVal;
            varVals<<basepoints[i].varVal;
            avgLass<<basepoints[i].avgLas;
            varLass<<basepoints[i].varLas;
            arrs<<arr;
            statuses<<basepoints[i].status;
            if(i > 379  && i < 390)
                qDebug() << qPrintable(arr);
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

bool DatabaseManager::insertTemporalWindow(const TemporalWindow &twindow)
{

    if(db.isOpen())
    {
        QSqlQuery query;

        query.prepare(QString("replace into temporalwindow values(?, ?, ?)"));

        query.addBindValue(twindow.id);
        query.addBindValue(twindow.startPoint);
        query.addBindValue(twindow.endPoint);



        bool ret = query.exec();

        return ret;

    }

    return false;

}
bool DatabaseManager::insertTopologicalMapRelation(int id, std::pair<int,int> relation)
{
    if(db.isOpen())
    {
        QSqlQuery query(QSqlDatabase::database("knowledge"));

        query.prepare(QString("replace into topologicalmap values(?, ?, ?)"));

        query.addBindValue(id);
        query.addBindValue(relation.first);
        query.addBindValue(relation.second);

        bool ret = query.exec();

        return ret;

    }

    return false;

}
int DatabaseManager::getLearnedPlaceMaxID()
{
    if(db.isOpen())
    {
        QSqlQuery query(QString("select MAX(id) from learnedplace"),QSqlDatabase::database("knowledge"));

        // query.prepare();

        // query.exec();

        query.next();

        return query.value(0).toInt();
    }
    else
    {
        qDebug() << "Database is not open, Ctrl+C to terminate";
        return 0;
    }
}

bool DatabaseManager::insertLearnedPlace(const LearnedPlace &learnedplace)
{

    QByteArray arr= mat2ByteArray(learnedplace.memberPlaces);

    QByteArray arr2 = mat2ByteArray(learnedplace.memberIds);

    QByteArray arr3 = mat2ByteArray(learnedplace.meanInvariant);

    QByteArray arr4 = mat2ByteArray(learnedplace.memberInvariants);


    if(db.isOpen())
    {

        QSqlQuery query(QSqlDatabase::database("knowledge"));

        query.prepare(QString("replace into learnedplace values(?, ?, ?, ?, ?)"));

        query.addBindValue(learnedplace.id);
        query.addBindValue(arr);
        query.addBindValue(arr2);
        query.addBindValue(arr3);
        query.addBindValue(arr4);

        bool ret = query.exec();

        return ret;

    }

    return false;

}
bool DatabaseManager::insertBDSTLevel(int id, const Level &aLevel)
{
    Mat members(aLevel.members);
    QByteArray arr= mat2ByteArray(members);

    //  QByteArray arr2 =

    Mat meanInvariant(aLevel.meanInvariant);
    QByteArray arr2 = mat2ByteArray(meanInvariant);

    if(db.isOpen())
    {

        QSqlQuery query(QSqlDatabase::database("knowledge"));

        query.prepare(QString("replace into cuetree values(?, ?, ?, ?, ?)"));

        query.addBindValue(id);
        query.addBindValue(arr);
        query.addBindValue(aLevel.connectionIndex);
        query.addBindValue(arr2);
        query.addBindValue(aLevel.val);


        bool ret = query.exec();

        return ret;

    }

    return false;

}

LearnedPlace DatabaseManager::getLearnedPlace(int id)
{
    LearnedPlace place;

    if(db.isOpen())
    {
        QSqlQuery query(QString("select* from learnedplace where id = %1").arg(id), QSqlDatabase::database("knowledge"));

        query.next();

        int id = query.value(0).toInt();

        // id;
        //qDebug()<<"Learned Place id"<<id;
        QByteArray array = query.value(1).toByteArray();
        place.memberPlaces = DatabaseManager::byteArray2Mat(array);

        QByteArray array2 = query.value(2).toByteArray();
        place.memberIds = DatabaseManager::byteArray2Mat(array2);

        QByteArray array3 = query.value(3).toByteArray();
        place.meanInvariant = DatabaseManager::byteArray2Mat(array3);

        QByteArray array4 = query.value(4).toByteArray();
        place.memberInvariants = DatabaseManager::byteArray2Mat(array4);

        //qDebug()<<meanInv.rows<<members.rows;

        place.id = id;

        //  QByteArray array = query.value(0).toByteArray();

        // return byteArray2Mat(array);
    }


    return place;

}

bool DatabaseManager::insertPlace(const Place &place)
{
    QByteArray arr = mat2ByteArray(place.meanInvariant);

    QByteArray arr2 = mat2ByteArray(place.memberIds);

    QByteArray arr3 = mat2ByteArray(place.memberInvariants);

    if(db.isOpen())
    {
        QSqlQuery query;

        query.prepare(QString("replace into place values(?, ?, ?, ?)"));

        query.addBindValue(place.id);
        query.addBindValue(arr);
        query.addBindValue(arr2);
        query.addBindValue(arr3);

        bool ret = query.exec();

        return ret;

    }

    return false;
}
cv::Mat DatabaseManager::getPlaceMeanInvariant(int id)
{
    if(db.isOpen())
    {
        QSqlQuery query(QString("select meaninvariant from place where id = %1").arg(id));

        query.next();

        QByteArray array = query.value(0).toByteArray();

        return byteArray2Mat(array);
    }

    else
    {
        qDebug() << "Database is not open, Ctrl+C to terminate";
        return Mat();
    }

}
cv::Mat DatabaseManager::getPlaceMemberIds(int id)
{
    if(db.isOpen())
    {
        QSqlQuery query(QString("select memberIds from place where id = %1").arg(id));

        query.next();

        QByteArray array = query.value(0).toByteArray();

        return byteArray2Mat(array);
    }
    else
    {
        qDebug() << "Database is not open, Ctrl+C to terminate";
        return Mat();
    }
}
Place DatabaseManager::getPlace(int id)
{
    Place place;

    if(db.isOpen())
    {
        QSqlQuery query(QString("select* from place where id = %1").arg(id));

        query.next();

        int id = query.value(0).toInt();

        // id;
        qDebug()<<id;
        QByteArray array = query.value(1).toByteArray();
        place.meanInvariant = DatabaseManager::byteArray2Mat(array);
        QByteArray array2 = query.value(2).toByteArray();
        place.memberIds = DatabaseManager::byteArray2Mat(array2);
        QByteArray array3 = query.value(3).toByteArray();
        place.memberInvariants = DatabaseManager::byteArray2Mat(array3);

        place.id = id;
        //  QByteArray array = query.value(0).toByteArray();

        // return byteArray2Mat(array);
    }
    return place;
}
QByteArray DatabaseManager::mat2ByteArray(const cv::Mat &image)
{
    QByteArray byteArray;
    QDataStream stream( &byteArray, QIODevice::WriteOnly );
    stream << image.type();
    stream << image.rows;
    stream << image.cols;
    const size_t data_size = image.cols * image.rows * image.elemSize();
    QByteArray data = QByteArray::fromRawData( (const char*)image.ptr(), data_size );
    stream << data;
    return byteArray;
}
cv::Mat DatabaseManager::byteArray2Mat(const QByteArray & byteArray)
{
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
