/****************************************************************************
** Meta object code from reading C++ file 'realtimezoomscroll.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../realtimezoomscroll.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'realtimezoomscroll.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RealTimeZoomScroll_t {
    QByteArrayData data[19];
    char stringdata0[214];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RealTimeZoomScroll_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RealTimeZoomScroll_t qt_meta_stringdata_RealTimeZoomScroll = {
    {
QT_MOC_LITERAL(0, 0, 18), // "RealTimeZoomScroll"
QT_MOC_LITERAL(1, 19, 11), // "onDataTimer"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 3), // "ik1"
QT_MOC_LITERAL(4, 36, 3), // "ik2"
QT_MOC_LITERAL(5, 40, 3), // "ik3"
QT_MOC_LITERAL(6, 44, 3), // "ik4"
QT_MOC_LITERAL(7, 48, 3), // "tau"
QT_MOC_LITERAL(8, 52, 19), // "onMouseUsageChanged"
QT_MOC_LITERAL(9, 72, 10), // "mouseUsage"
QT_MOC_LITERAL(10, 83, 6), // "onSave"
QT_MOC_LITERAL(11, 90, 21), // "onUpdatePeriodChanged"
QT_MOC_LITERAL(12, 112, 19), // "onMouseMovePlotArea"
QT_MOC_LITERAL(13, 132, 12), // "QMouseEvent*"
QT_MOC_LITERAL(14, 145, 5), // "event"
QT_MOC_LITERAL(15, 151, 18), // "onChartUpdateTimer"
QT_MOC_LITERAL(16, 170, 17), // "onViewPortChanged"
QT_MOC_LITERAL(17, 188, 19), // "onHScrollBarChanged"
QT_MOC_LITERAL(18, 208, 5) // "value"

    },
    "RealTimeZoomScroll\0onDataTimer\0\0ik1\0"
    "ik2\0ik3\0ik4\0tau\0onMouseUsageChanged\0"
    "mouseUsage\0onSave\0onUpdatePeriodChanged\0"
    "onMouseMovePlotArea\0QMouseEvent*\0event\0"
    "onChartUpdateTimer\0onViewPortChanged\0"
    "onHScrollBarChanged\0value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RealTimeZoomScroll[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    5,   54,    2, 0x0a /* Public */,
       8,    1,   65,    2, 0x08 /* Private */,
      10,    1,   68,    2, 0x08 /* Private */,
      11,    1,   71,    2, 0x08 /* Private */,
      12,    1,   74,    2, 0x08 /* Private */,
      15,    0,   77,    2, 0x08 /* Private */,
      16,    0,   78,    2, 0x08 /* Private */,
      17,    1,   79,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    4,    5,    6,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   18,

       0        // eod
};

void RealTimeZoomScroll::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RealTimeZoomScroll *_t = static_cast<RealTimeZoomScroll *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onDataTimer((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 1: _t->onMouseUsageChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->onSave((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->onUpdatePeriodChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->onMouseMovePlotArea((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 5: _t->onChartUpdateTimer(); break;
        case 6: _t->onViewPortChanged(); break;
        case 7: _t->onHScrollBarChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject RealTimeZoomScroll::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_RealTimeZoomScroll.data,
      qt_meta_data_RealTimeZoomScroll,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RealTimeZoomScroll::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RealTimeZoomScroll::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RealTimeZoomScroll.stringdata0))
        return static_cast<void*>(const_cast< RealTimeZoomScroll*>(this));
    return QDialog::qt_metacast(_clname);
}

int RealTimeZoomScroll::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
