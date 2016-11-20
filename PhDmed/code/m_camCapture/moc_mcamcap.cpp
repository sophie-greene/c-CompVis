/****************************************************************************
** Meta object code from reading C++ file 'mcamcap.h'
**
** Created: Tue Jun 11 12:24:48 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "src/mcamcap.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mcamcap.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_mCamCap[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      35,    8,    8,    8, 0x08,
      59,    8,    8,    8, 0x08,
      71,    8,    8,    8, 0x0a,
      93,    8,    8,    8, 0x0a,
     115,    8,    8,    8, 0x0a,
     137,    8,    8,    8, 0x0a,
     159,    8,    8,    8, 0x0a,
     181,    8,    8,    8, 0x0a,
     203,    8,    8,    8, 0x0a,
     225,    8,    8,    8, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_mCamCap[] = {
    "mCamCap\0\0on_cmdCalibrate_clicked()\0"
    "on_cmdCapture_clicked()\0calibrate()\0"
    "onImageAquired0(bool)\0onImageAquired1(bool)\0"
    "onImageAquired2(bool)\0onImageAquired3(bool)\0"
    "onImageAquired4(bool)\0onImageAquired5(bool)\0"
    "onImageAquired6(bool)\0onImageAquired7(bool)\0"
};

const QMetaObject mCamCap::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_mCamCap,
      qt_meta_data_mCamCap, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &mCamCap::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *mCamCap::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *mCamCap::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_mCamCap))
        return static_cast<void*>(const_cast< mCamCap*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int mCamCap::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_cmdCalibrate_clicked(); break;
        case 1: on_cmdCapture_clicked(); break;
        case 2: calibrate(); break;
        case 3: onImageAquired0((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: onImageAquired1((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: onImageAquired2((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: onImageAquired3((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: onImageAquired4((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: onImageAquired5((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: onImageAquired6((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: onImageAquired7((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
