/*
Estar - это класс основного окна, в котором на данный момент можно выполнить следующие действия:
    1) Запуск моделирования движения спутника по круговой орбите с заданными параметрами.
    2) Вызов диалогового окна с выбором режима съемки для спутника
    3) Вызов окна, выводящего график зависимости угловой скорости по трем координатам от времени
    4) Вывод значений проекций угловой скорости по трем осям в системе координат КА, приборной системе координат, значения ИК датчика угловой скорости
    5) Вывод координат КА в ск J2000
    6) Вывод продолжительности работы спутника (текущего времени)
*/

#ifndef ESTAR_H
#define ESTAR_H


#include "dialogmotion.h"
//#include "count.h"
#include <QWidget>
#include <QPushButton>
#include <QLayout>
#include <QLabel>
#include <QLineEdit>
#include <QGroupBox>
#include <QDateEdit>
#include <QComboBox>
#include <QThread>
#include "succes.h"
#include "realtimezoomscroll.h"

class Estar : public QWidget
{
    Q_OBJECT

public:
    Estar(QWidget *parent = 0);
    ~Estar();
    enum Mode{START, STOP, PAUSE};  //  Текущее состояние окна в зависимости от нажатой кнопки
     Mode currentMode;               //  Переменная, показывающая текущее состояние окна
    QComboBox *GraphView;           //  Панель, в которой можно выбрать выводимый график (пока не используется)

    QPushButton *StartButton;       //  Кнопка "Старт"
    QPushButton *QuitButton;        //  Кнопка "Выход"
    QPushButton *PauseButton;       //  Кнопка "Пауза"  (пока не работает)
    QPushButton *StopButton;        //  Кнопка "Стоп"   (пока не работает)
    QPushButton *GraphButton;       //  Кнопка "Вывод графика"
    QPushButton *MotionButton;      //  Кнопка "Выбор режима съемки"
    QPushButton *SpeedUpButton;
    QPushButton *SpeedDownButton;
    DialogMotion *DM;               //  Диалоговое окно выбора режима съемки спутника
                                    //  Параметры и свойства окна можно посмотреть в dialogmotion.h
    RealTimeZoomScroll *demo;
    QDateTimeEdit *dateEdit;            //  Элемент вывода даты (пока не используется)

private:


    Mode previousMode;              //  Переменная, показывающая предыдущее состояние окна


    void changeMode(Mode mode);     //  Функция меняющая текущее состояние окна на mode

    QStringList GraphList;          //  Список возможных вариантов вывода графика в графическом окне (пока не используется)
    QLabel *KrenFin;            //  Поле ввода геодезической широты
    QLabel *TangFin;            //  Поле ввода геодезической долготы
    QLabel *label1;                 //  Подпись "Высота в км"
    QLabel *label2;                 //  Подпись "Наклон в градусах"
    QLabel *label3;                 //  Подпись "Скорость в км/ч"
    QLabel *label4;                 //  Подпись "Дата запуска"
    QLabel *label5;                 //  Подпись "Координаты КА в J2000"
    QLabel *label6;                 //  Подпись "Время работы"
    QLabel *label7;                 //  Подпись "В с.к. КА"
    QLabel *label8;                 //  Подпись "В с.к. прибора"
    QLabel *label9;                 //  Подпись "Показания ИК"
    QLabel *label10;                //  Подпись "Начальная позиция в геод. с.к"
    QLabel *label11;                //  Подпись "Широта"
    QLabel *label12;                //  Подпись "Долгота"
    QLabel *label13;                //  Подпись "Текущий поворот по крену"
    QLabel *label14;                //  Подпись "Текущий поворот по тангажу"
    QLabel *label15;                //  Подпись "Координаты КА в геод-х коорд."
    QLabel *label16;                //  Подпись "Изменить скорость обработки"

    QLabel *KrenLine;               //  Строка вывода текущего поворота по крену в градусах
    QLabel *TangLine;               //  Строка вывода текущего поворота по тангажу в градусах

    QLabel *KoordinateLineX;        //  Строка вывода координаты X КА в системе J2000
    QLabel *KoordinateLineY;        //  Строка вывода координаты Y КА в системе J2000
    QLabel *KoordinateLineZ;        //  Строка вывода координаты Z КА в системе J2000

    QLabel *GeodLineL;        //  Строка вывода координаты X КА в системе J2000
    QLabel *GeodLineB;        //  Строка вывода координаты Y КА в системе J2000
    QLabel *GeodLineH;        //  Строка вывода координаты Z КА в системе J2000

    QLabel *omega_KA_x_label;       //  Строка вывода проекции угловой скорости КА на ось X с.к. КА
    QLabel *omega_KA_y_label;       //  Строка вывода проекции угловой скорости КА на ось Y с.к. КА
    QLabel *omega_KA_z_label;       //  Строка вывода проекции угловой скорости КА на ось Z с.к. КА

    QLabel *omega_PR_x_label;       //  Строка вывода проекции угловой скорости КА на ось X в приборной с.к.
    QLabel *omega_PR_y_label;       //  Строка вывода проекции угловой скорости КА на ось Y в приборной с.к.
    QLabel *omega_PR_z_label;       //  Строка вывода проекции угловой скорости КА на ось Z в приборной с.к.


    QLabel *IK1_label;              //  Строка вывода показаний ИК-1
    QLabel *IK2_label;              //  Строка вывода показаний ИК-2
    QLabel *IK3_label;              //  Строка вывода показаний ИК-3
    QLabel *IK4_label;              //  Строка вывода показаний ИК-4

    QLabel *TimeLine;               //  Строка вывода времени работы спутника

    QLabel *HeightLine;             //  Строка, в которую записывается высота орбиты
    QLabel *InclineLine;            //  Строка, в которую записывается наклон орбиты
    QLabel *VelocityLine;           //  Строка, в которую записывается скорость КА

    QGroupBox *gb1;                 //  Раздел окна "Параметры орбиты"
    QGroupBox *gb2;                 //  Раздел окна "Начальные параметры КА"
    QGroupBox *gb3;                 //  Раздел окна "Графики угловых скоростей"
    QGroupBox *gb4;                 //  Раздел окна "Угловая скорость"


    QDateTime StartDate;                //  Дата запуска        (пока не используется)


    Succes *S;

signals:
    void send_kadr_position(double b, double l,int mode);

public slots:
    void movie_started();

private slots:
    void Succesed();
    void StartButton_clicked();     //  Слот, внутри которого описывается работа программы при нажатии кнопки "Старт"
    void PauseButton_clicked();     //  Слот, внутри которого описывается работа программы при нажатии кнопки "Пауза" (пока только меняет вид основного окна)
    void StopButton_clicked();      //  Слот, внутри которого описывается работа программы при нажатии кнопки "Стоп"  (пока только меняет вид основного окна)
    void GraphButton_clicked();     //  Слот, внутри которого описывается работа программы при нажатии кнопки "Вывод графика"
    void QuitButton_clicked();      //  Слот, внутри которого описывается работа программы при нажатии кнопки "Выход"
    void MotionButton_clicked();    //  Слот, внутри которого описывается работа программы при нажатии кнопки "Выбор режима съемки"
    void update(double r00, double r01, double r02, double w00,
                double w01, double w02, QTime TIME);      // Функция, перезаписывающая значения координат КА, его угл. скорости и текущего времени в соответствующих строках основного окна
    void updategeod(double B,double L,double H);
    void update1(double wnv0, double wnv1, double wnv2);
    void update_ik(double ik1,double ik2,double ik3,double ik4);
};

#endif // ESTAR_H
