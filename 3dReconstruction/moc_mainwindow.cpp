/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "include/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x0a,
      49,   12,   11,   11, 0x0a,
      83,   81,   11,   11, 0x0a,
     116,   81,   11,   11, 0x0a,
     153,   81,   11,   11, 0x0a,
     182,   11,   11,   11, 0x0a,
     211,   81,   11,   11, 0x0a,
     236,   11,   11,   11, 0x0a,
     251,   11,   11,   11, 0x0a,
     282,   11,  277,   11, 0x0a,
     307,   11,   11,   11, 0x08,
     331,   11,   11,   11, 0x08,
     356,   11,   11,   11, 0x08,
     374,   11,   11,   11, 0x08,
     389,   11,   11,   11, 0x08,
     408,   11,   11,   11, 0x08,
     425,   11,   11,   11, 0x08,
     440,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0,,,\0displayImage(QImage,int,int,int)\0"
    "onlaserAquired(int,int,int,int)\0,\0"
    "onCalibrationSuccess(QImage,int)\0"
    "onIntrCalibrationSuccess(QImage,int)\0"
    "onIntrCalibStopped(int,bool)\0"
    "onpmCalibrationSuccess(bool)\0"
    "onLaserAquis(QImage,int)\0runCalibrate()\0"
    "writePoint(vector<Point>)\0bool\0"
    "allFrames(vector<Point>)\0"
    "on_cmdLightOn_clicked()\0"
    "on_cmdLightOff_clicked()\0on_menuStartCam()\0"
    "on_menuCalib()\0on_menuIntrCalib()\0"
    "on_menuPMCalib()\0on_menuLaser()\0"
    "on_menuQuit()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->displayImage((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 1: _t->onlaserAquired((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 2: _t->onCalibrationSuccess((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->onIntrCalibrationSuccess((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->onIntrCalibStopped((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 5: _t->onpmCalibrationSuccess((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->onLaserAquis((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->runCalibrate(); break;
        case 8: _t->writePoint((*reinterpret_cast< const vector<Point>(*)>(_a[1]))); break;
        case 9: { bool _r = _t->allFrames((*reinterpret_cast< const vector<Point>(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 10: _t->on_cmdLightOn_clicked(); break;
        case 11: _t->on_cmdLightOff_clicked(); break;
        case 12: _t->on_menuStartCam(); break;
        case 13: _t->on_menuCalib(); break;
        case 14: _t->on_menuIntrCalib(); break;
        case 15: _t->on_menuPMCalib(); break;
        case 16: _t->on_menuLaser(); break;
        case 17: _t->on_menuQuit(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 18)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 18;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
