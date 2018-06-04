#ifndef GRAPHICALMODEL_H
#define GRAPHICALMODEL_H

#include <QtOpenGL>
#include <QOpenGLFunctions_1_1>
#include <QAxBindable>

class GraphicalModel : public QGLWidget,
                       public QOpenGLFunctions_1_1,
                       public QAxBindable
{
    Q_OBJECT
    Q_CLASSINFO("ClassID",     "{5fd9c22e-ed45-43fa-ba13-1530bb6b03e0}")
    Q_CLASSINFO("InterfaceID", "{33b051af-bb25-47cf-a390-5cfd2987d26a}")
    Q_CLASSINFO("EventsID",    "{8c996c29-eafa-46ac-a6f9-901951e765b5}")

public:
    explicit GraphicalModel(QWidget *parent);
    virtual ~GraphicalModel();

signals:

public slots:
    void                setKAPosition(double xj2000,double yj2000,double zj2000);
    void                setKAOrientation(double l0,double l1,double l2,double l3);

protected:
    void                initializeGL();
    void                paintGL();
    void                resizeGL(int w, int h);
    virtual GLuint      makeObject();
    GLuint              makeObject1();


private:
    GLuint  m_object;
    GLuint  m_object1;
    GLdouble m_xRot;
    GLdouble m_yRot;
    GLdouble m_zRot;
    GLdouble Phi_deg;
    GLdouble m_scale;
    GLdouble RJ2000x;
    GLdouble RJ2000y;
    GLdouble RJ2000z;
    GLdouble dx;
};

#endif // GLBOX_H
