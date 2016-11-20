/****************************************************************************
** Meta object code from reading C++ file 'ProcessingThread.h'
**
** Created: Fri Dec 7 10:22:51 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ProcessingThread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ProcessingThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ProcessingThread[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      24,   18,   17,   17, 0x05,

 // slots: signature, parameters, type, tag, flags
      41,   17,   17,   17, 0x08,
      90,   17,   17,   17, 0x08,
     145,   17,   17,   17, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ProcessingThread[] = {
    "ProcessingThread\0\0frame\0newFrame(QImage)\0"
    "updateImageProcessingFlags(ImageProcessingFlags)\0"
    "updateImageProcessingSettings(ImageProcessingSettings)\0"
    "updateTaskData(TaskData)\0"
};

const QMetaObject ProcessingThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ProcessingThread,
      qt_meta_data_ProcessingThread, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ProcessingThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ProcessingThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ProcessingThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ProcessingThread))
        return static_cast<void*>(const_cast< ProcessingThread*>(this));
    return QThread::qt_metacast(_clname);
}

int ProcessingThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newFrame((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 1: updateImageProcessingFlags((*reinterpret_cast< ImageProcessingFlags(*)>(_a[1]))); break;
        case 2: updateImageProcessingSettings((*reinterpret_cast< ImageProcessingSettings(*)>(_a[1]))); break;
        case 3: updateTaskData((*reinterpret_cast< TaskData(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ProcessingThread::newFrame(const QImage & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
