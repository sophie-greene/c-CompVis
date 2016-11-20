/****************************************************************************
** Meta object code from reading C++ file 'spinboxtest.h'
**
** Created: Tue May 21 11:32:45 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "spinboxtest.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'spinboxtest.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SpinBoxTest[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      24,   12,   12,   12, 0x08,
      40,   12,   12,   12, 0x08,
      53,   12,   12,   12, 0x08,
      71,   12,   12,   12, 0x08,
      85,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SpinBoxTest[] = {
    "SpinBoxTest\0\0testKeys()\0testKeys_data()\0"
    "testClicks()\0testClicks_data()\0"
    "testSetting()\0testSetting_data()\0"
};

const QMetaObject SpinBoxTest::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SpinBoxTest,
      qt_meta_data_SpinBoxTest, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SpinBoxTest::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SpinBoxTest::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SpinBoxTest::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SpinBoxTest))
        return static_cast<void*>(const_cast< SpinBoxTest*>(this));
    return QObject::qt_metacast(_clname);
}

int SpinBoxTest::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: testKeys(); break;
        case 1: testKeys_data(); break;
        case 2: testClicks(); break;
        case 3: testClicks_data(); break;
        case 4: testSetting(); break;
        case 5: testSetting_data(); break;
        default: ;
        }
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
