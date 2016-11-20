#ifndef DATABASE_H
#define DATABASE_H

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>
#include <QVariant> 

class DatabaseManager{
public:
  DatabaseManager();
  ~DatabaseManager();
  void reset();
  QVariant get(QString queryText);
  int addImage(QString path);
  int addImage(QVariant image, QString path);
  int addPoint2D(double x, double y, int pt3D_ID, int image_ID);
  int addPoint2D(double x, double y, int pt3D_ID, int image_ID, QVariant descriptor);
  int addPoint3D(double x, double y, double z);
private:
  QSqlDatabase db;
  QSqlQuery queryGlobal;
};


#endif //DATABASE_H
