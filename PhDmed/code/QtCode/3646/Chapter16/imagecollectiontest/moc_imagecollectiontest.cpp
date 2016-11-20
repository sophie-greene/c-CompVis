/****************************************************************************
** Meta object code from reading C++ file 'imagecollectiontest.h'
**
** Created: Tue May 21 11:31:57 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "imagecollectiontest.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'imagecollectiontest.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ImageCollectionTest[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x08,
      32,   20,   20,   20, 0x08,
      45,   20,   20,   20, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ImageCollectionTest[] = {
    "ImageCollectionTest\0\0testTags()\0"
    "testImages()\0testImagesFromTags()\0"
};

const QMetaObject ImageCollectionTest::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_ImageCollectionTest,
      qt_meta_data_ImageCollectionTest, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ImageCollectionTest::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ImageCollectionTest::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ImageCollectionTest::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ImageCollectionTest))
        return static_cast<void*>(const_cast< ImageCollectionTest*>(this));
    return QObject::qt_metacast(_clname);
}

int ImageCollectionTest::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: testTags(); break;
        case 1: testImages(); break;
        case 2: testImagesFromTags(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
