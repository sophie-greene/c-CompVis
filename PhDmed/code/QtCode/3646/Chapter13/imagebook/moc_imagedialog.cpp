/****************************************************************************
** Meta object code from reading C++ file 'imagedialog.h'
**
** Created: Tue May 21 11:30:08 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "imagedialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'imagedialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ImageDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      27,   12,   12,   12, 0x08,
      45,   12,   12,   12, 0x08,
      59,   12,   12,   12, 0x08,
      77,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ImageDialog[] = {
    "ImageDialog\0\0nextClicked()\0previousClicked()\0"
    "tagsChanged()\0addImageClicked()\0"
    "addTagClicked()\0"
};

const QMetaObject ImageDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ImageDialog,
      qt_meta_data_ImageDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ImageDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ImageDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ImageDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ImageDialog))
        return static_cast<void*>(const_cast< ImageDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ImageDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: nextClicked(); break;
        case 1: previousClicked(); break;
        case 2: tagsChanged(); break;
        case 3: addImageClicked(); break;
        case 4: addTagClicked(); break;
        default: ;
        }
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
