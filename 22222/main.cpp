#include "estar.h"
#include "count.h"
#include <QApplication>
#include <QMainWindow>
#include <QErrorMessage>
#include <QtOpenGL>
#include <QAxFactory>
#include "graphicalmodel.h"
#include "modelwindow.h"

QAXFACTORY_BEGIN(
    "{2c3c183a-eeda-41a4-896e-3d9c12c3577d}", // type library ID
    "{83e16271-6480-45d5-aaf1-3f40b7661ae4}") // application ID
    QAXCLASS(GraphicalModel)
QAXFACTORY_END()

int main(int argc, char *argv[])
{

        QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
        QApplication::setColorSpec(QApplication::CustomColor);
        QApplication a(argc,argv);

        if (QOpenGLContext::openGLModuleType() != QOpenGLContext::LibGL) {
            qWarning("This system does not support OpenGL. Exiting.");
            return -1;
        }

  //  qApp->addLibraryPath("C:/Qt/Qt5.7.0/5.7/mingw53_32/plugins");   // Задает путь для плагинов Qt
    //QApplication a(argc, argv);                                     // Создаем приложение
    Estar es;                                                    // Создаем основное окошко, имеющее класс Estar
    es.setWindowTitle("Estar Kinematika");                           // Задаем название этого окна Estar Kinematika                                                    // Выводим основное окно
    Count c(&es);
    es.show();
    return a.exec();                                               // Запускаем приложение
}
