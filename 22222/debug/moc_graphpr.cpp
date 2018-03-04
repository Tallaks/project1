/****************************************************************************
** Meta object code from reading C++ file 'graphpr.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../graphpr.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'graphpr.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_GraphPR_t {
    QByteArrayData data[18];
    char stringdata0[193];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GraphPR_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GraphPR_t qt_meta_stringdata_GraphPR = {
    {
QT_MOC_LITERAL(0, 0, 7), // "GraphPR"
QT_MOC_LITERAL(1, 8, 11), // "onDataTimer"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 1), // "x"
QT_MOC_LITERAL(4, 23, 1), // "y"
QT_MOC_LITERAL(5, 25, 1), // "z"
QT_MOC_LITERAL(6, 27, 3), // "tau"
QT_MOC_LITERAL(7, 31, 19), // "onMouseUsageChanged"
QT_MOC_LITERAL(8, 51, 10), // "mouseUsage"
QT_MOC_LITERAL(9, 62, 6), // "onSave"
QT_MOC_LITERAL(10, 69, 21), // "onUpdatePeriodChanged"
QT_MOC_LITERAL(11, 91, 19), // "onMouseMovePlotArea"
QT_MOC_LITERAL(12, 111, 12), // "QMouseEvent*"
QT_MOC_LITERAL(13, 124, 5), // "event"
QT_MOC_LITERAL(14, 130, 18), // "onChartUpdateTimer"
QT_MOC_LITERAL(15, 149, 17), // "onViewPortChanged"
QT_MOC_LITERAL(16, 167, 19), // "onHScrollBarChanged"
QT_MOC_LITERAL(17, 187, 5) // "value"

    },
    "GraphPR\0onDataTimer\0\0x\0y\0z\0tau\0"
    "onMouseUsageChanged\0mouseUsage\0onSave\0"
    "onUpdatePeriodChanged\0onMouseMovePlotArea\0"
    "QMouseEvent*\0event\0onChartUpdateTimer\0"
    "onViewPortChanged\0onHScrollBarChanged\0"
    "value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GraphPR[] = {

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
       1,    4,   54,    2, 0x0a /* Public */,
       7,    1,   63,    2, 0x08 /* Private */,
       9,    1,   66,    2, 0x08 /* Private */,
      10,    1,   69,    2, 0x08 /* Private */,
      11,    1,   72,    2, 0x08 /* Private */,
      14,    0,   75,    2, 0x08 /* Private */,
      15,    0,   76,    2, 0x08 /* Private */,
      16,    1,   77,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    4,    5,    6,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   17,

       0        // eod
};

void GraphPR::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GraphPR *_t = static_cast<GraphPR *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onDataTimer((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
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

const QMetaObject GraphPR::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_GraphPR.data,
      qt_meta_data_GraphPR,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GraphPR::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GraphPR::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GraphPR.stringdata0))
        return static_cast<void*>(const_cast< GraphPR*>(this));
    return QDialog::qt_metacast(_clname);
}

int GraphPR::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
