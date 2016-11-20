/****************************************************************************
** Meta object code from reading C++ file 'FrameLabel.h'
**
** Created: Fri Dec 7 10:22:53 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "FrameLabel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FrameLabel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_FrameLabel[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      22,   12,   11,   11, 0x05,
      46,   11,   11,   11, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_FrameLabel[] = {
    "FrameLabel\0\0mouseData\0newMouseData(MouseData)\0"
    "onMouseMoveEvent()\0"
};

const QMetaObject FrameLabel::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_FrameLabel,
      qt_meta_data_FrameLabel, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &FrameLabel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *FrameLabel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *FrameLabel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_FrameLabel))
        return static_cast<void*>(const_cast< FrameLabel*>(this));
    return QLabel::qt_metacast(_clname);
}

int FrameLabel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newMouseData((*reinterpret_cast< MouseData(*)>(_a[1]))); break;
        case 1: onMouseMoveEvent(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void FrameLabel::newMouseData(MouseData _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void FrameLabel::onMouseMoveEvent()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
