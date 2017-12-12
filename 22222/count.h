/*
Count - Это расчетный класс, в котором на данный момент можно выполнить следующие действия:
1) Задать параметры движения КА в начальный момент времени
2) Смоделировать невозмущенное движение КА по орбите
3) Смоделировать вращение КА по крену
4) Смоделировать движение КА при отслеживании точки с поверхности\
5) Передача параметров движения основному окну программы
6) Передача значений угловой скорости графическому окну
7) Задание базиса КА
8)
*/
#ifndef COUNT_H
#define COUNT_H

#include "settings.h"
#include <QObject>
#include <estar.h>


class Count : public Estar
{
    Q_OBJECT

    public:
        explicit Count(Estar *parent = Q_NULLPTR);
        Settings *settings;
        double tau_kadr;
        QTime tau;
        int speed;
        // Текущая дата и время
       QDateTime DT;
        int MotionMode;

    public slots:
        void NevozMotion();
        /* NevozMotion - это функция класса Count, моделирующая невозмущенное движение спутника по орбите */
        void ResultMotion();
        void SetStop();
        void SetStart();
        void SetPause();
        void delay(int n);
        void speedup();
        void speeddown();


    signals:
        void send_graph(double,double,double,double);
        void send_nv(double,double,double,double,double,double,QTime);
        void send_m(int);
        void send_kadr(double,double,double,double,double,double,double,double,double);
        void done();
        void send_geod(double,double,double);
       // void send_nev(double,double,double,QDateTime);
        void send_ik(double,double,double,double);
        void send_graph1(double,double,double,double,double);

    private slots:
        //void StartPosition(double lat, double lon, int mode, QDateTime ntd); //Кадровая съемка

};

#endif // COUNT_H
