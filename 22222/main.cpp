#include "estar.h"
#include "count.h"
#include <QApplication>
#include <QMainWindow>
#include <QErrorMessage>

int main(int argc, char *argv[])
{
    qApp->addLibraryPath("F:/Qt/Qt5.7.0/5.7/mingw53_32/plugins");   // Задает путь для плагинов Qt
    QApplication a(argc, argv);                                     // Создаем приложение
    Estar es;                                                    // Создаем основное окошко, имеющее класс Estar
    es.setWindowTitle("Estar Kinematika");                           // Задаем название этого окна Estar Kinematika
  //  w.show();                                                       // Выводим основное окно
    Count c(&es);
    es.show();
    return a.exec();                                               // Запускаем приложение

}
