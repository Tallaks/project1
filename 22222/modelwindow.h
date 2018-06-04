#ifndef MODELWINDOW_H
#define MODELWINDOW_H

#include <qwidget.h>
#include "graphicalmodel.h"

class modelwindow : public QWidget
{
    Q_OBJECT

public:
    explicit modelwindow(QWidget *parent = nullptr);
    GraphicalModel *c;
};

#endif
