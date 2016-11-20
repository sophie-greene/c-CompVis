/****************************************************************************
** Meta object code from reading C++ file 'ImageProcessingSettingsDialog.h'
**
** Created: Fri Dec 7 10:22:54 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ImageProcessingSettingsDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ImageProcessingSettingsDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ImageProcessingSettingsDialog[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      42,   31,   30,   30, 0x05,

 // slots: signature, parameters, type, tag, flags
      94,   30,   30,   30, 0x0a,
     121,   30,   30,   30, 0x0a,
     154,   30,   30,   30, 0x08,
     184,   30,   30,   30, 0x08,
     214,   30,   30,   30, 0x08,
     243,   30,   30,   30, 0x08,
     271,   30,   30,   30, 0x08,
     300,   30,   30,   30, 0x08,
     317,   30,   30,   30, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ImageProcessingSettingsDialog[] = {
    "ImageProcessingSettingsDialog\0\0"
    "p_settings\0"
    "newImageProcessingSettings(ImageProcessingSettings)\0"
    "resetAllDialogToDefaults()\0"
    "updateStoredSettingsFromDialog()\0"
    "resetSmoothDialogToDefaults()\0"
    "resetDilateDialogToDefaults()\0"
    "resetErodeDialogToDefaults()\0"
    "resetFlipDialogToDefaults()\0"
    "resetCannyDialogToDefaults()\0"
    "validateDialog()\0smoothTypeChange(QAbstractButton*)\0"
};

const QMetaObject ImageProcessingSettingsDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ImageProcessingSettingsDialog,
      qt_meta_data_ImageProcessingSettingsDialog, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ImageProcessingSettingsDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ImageProcessingSettingsDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ImageProcessingSettingsDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ImageProcessingSettingsDialog))
        return static_cast<void*>(const_cast< ImageProcessingSettingsDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ImageProcessingSettingsDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newImageProcessingSettings((*reinterpret_cast< ImageProcessingSettings(*)>(_a[1]))); break;
        case 1: resetAllDialogToDefaults(); break;
        case 2: updateStoredSettingsFromDialog(); break;
        case 3: resetSmoothDialogToDefaults(); break;
        case 4: resetDilateDialogToDefaults(); break;
        case 5: resetErodeDialogToDefaults(); break;
        case 6: resetFlipDialogToDefaults(); break;
        case 7: resetCannyDialogToDefaults(); break;
        case 8: validateDialog(); break;
        case 9: smoothTypeChange((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void ImageProcessingSettingsDialog::newImageProcessingSettings(ImageProcessingSettings _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
