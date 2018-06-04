#include "graphicalmodel.h"
#include <QAxAggregated>
#include <QUuid>

GraphicalModel::GraphicalModel(QWidget *parent) :
    QGLWidget(parent)
{
    m_xRot = 1.0;
    m_yRot = m_zRot = 0.0;       // default object rotation
    dx = 0;
    m_scale = 1.25;                       // default object scale
    m_object = 0;
    m_object1 = 0;
}


GraphicalModel::~GraphicalModel()
{
    makeCurrent();
    if (m_object)
        glDeleteLists(m_object, 1);
    if (m_object1)
        glDeleteLists(m_object1, 1);

}


void GraphicalModel::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glLoadIdentity();
    glTranslatef(RJ2000x/5000000, RJ2000y/5000000, RJ2000z/5000000-10);
    glScalef(1, 1, 1);
    glRotatef(Phi_deg, m_xRot, m_yRot,m_zRot);
    glCallList(m_object);
    glTranslatef(-RJ2000x/5000000,-RJ2000y/5000000,-RJ2000z/5000000+10);

    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);
    glScalef(1, 1, 1);
    glRotatef(0, 1, 0,0);
    glCallList(m_object1);

}


/*!
  Set up the OpenGL rendering state, and define display list
*/

void GraphicalModel::initializeGL()
{
    initializeOpenGLFunctions();

    qglClearColor(Qt::white);           // Let OpenGL clear to black
    m_object = makeObject();            // Generate an OpenGL display list
    m_object1 = makeObject1();
    glShadeModel(GL_FLAT);
}



/*!
  Set up the OpenGL view port, matrix mode, etc.
*/

void GraphicalModel::resizeGL(int w, int h)
{
    glViewport(0, 0, (GLint)w, (GLint)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1.0, 1.0, -1.0, 1.0, 5.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}


/*!
  Generate an OpenGL display list for the object to be shown, i.e. the box
*/

GLuint GraphicalModel::makeObject()
{
    GLuint list;

    list = glGenLists(1);

    glNewList(list, GL_COMPILE);

    qglColor(Qt::black);                      // Shorthand for glColor3f or glIndex

    glLineWidth(2.0);

    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 1.0, 0.0, 0.0);
    glEnd();


    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 0.0, 1.0, 0.0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 0.0,  0.0, 1.0);
    glEnd();

    glEndList();

    return list;
}

GLuint GraphicalModel::makeObject1()
{
    dx+=1;
    GLuint list;

    list = glGenLists(1);

    glNewList(list, GL_COMPILE);

    qglColor(Qt::red);                      // Shorthand for glColor3f or glIndex

    glLineWidth(2.0);

    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 1.0, 0.0, 0.0);
    glEnd();


    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 0.0, 1.0, 0.0);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f( 0.0,  0.0, 0.0);
    glVertex3f( 0.0,  0.0, 1.0);
    glEnd();

    glEndList();

    return list;
}

void GraphicalModel::setKAPosition(double xj2000,double yj2000,double zj2000){
    RJ2000x = xj2000;
    RJ2000y = yj2000;
    RJ2000z = zj2000;
    updateGL();
}

void GraphicalModel::setKAOrientation(double l0,double l1,double l2,double l3){
    Phi_deg = acos(l0)*180.0/M_PI;
    m_xRot  = l1/sin(Phi_deg*M_PI/180.0);
    m_yRot  = l2/sin(Phi_deg*M_PI/180.0);
    m_zRot  = l3/sin(Phi_deg*M_PI/180.0);
    updateGL();
}
