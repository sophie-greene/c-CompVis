/****************************************************************************
** Meta object code from reading C++ file 'textdialog.h'
**
** Created: Tue May 21 11:28:28 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "textdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'textdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_TextDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x0a,
      40,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TextDialog[] = {
    "TextDialog\0\0tan\0showText(TextAndNumber)\0"
    "buttonClicked(QAbstractButton*)\0"
};

const QMetaObject TextDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_TextDialog,
      qt_meta_data_TextDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TextDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TextDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TextDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TextDialog))
        return static_cast<void*>(const_cast< TextDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int TextDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: showText((*reinterpret_cast< TextAndNumber(*)>(_a[1]))); break;
        case 1: buttonClicked((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
