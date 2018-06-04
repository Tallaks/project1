#include "modelwindow.h"
#include <QPushButton>
#include <QSlider>
#include <QLayout>
#include <QFrame>
#include <QMenuBar>
#include <QMenu>
#include <QApplication>


modelwindow::modelwindow(QWidget *parent)
    : QWidget(parent)
{
    setWindowTitle("3D Modelling");
    // Create a menu
    QMenu *file = new QMenu(this);
    file->addAction(tr("Exit"), qApp, &QApplication::quit);

    // Create a menu bar
    QMenuBar *m = new QMenuBar(this);
    m->addMenu(file)->setText(tr("&File"));

    // Create a nice frame to put around the OpenGL widget
    QFrame *f = new QFrame(this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);

    // Create our OpenGL widget
    c = new GraphicalModel(f);

    // Now that we have all the widgets, put them into a nice layout

    // Top level layout, puts the sliders to the left of the frame/GL widget
    QHBoxLayout *hlayout = new QHBoxLayout(this);

    // Put the sliders on top of each other
    QVBoxLayout *vlayout = new QVBoxLayout();

    // Put the GL widget inside the frame
    QHBoxLayout *flayout = new QHBoxLayout(f);
    flayout->setMargin(0);
    flayout->addWidget(c, 1);

    hlayout->setMenuBar(m);
    hlayout->addLayout(vlayout);
    hlayout->addWidget(f, 1);
}
