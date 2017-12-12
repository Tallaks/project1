#include "succes.h"

Succes::Succes(QWidget *parent) : QWidget(parent)
{
    setGeometry(120,100,200,40);
    setWindowTitle("Съемка удалась!");
    QLabel *a = new QLabel("Съемка удалась!");
    QHBoxLayout *l = new QHBoxLayout;
    l->addWidget(a);
    setLayout(l);
}
