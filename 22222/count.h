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

#include <QObject>
#include <estar.h>
#include "emath.h"
#include "j2000.h"

class Count : public Estar
{
    Q_OBJECT

    public:
        explicit Count(Estar *parent = Q_NULLPTR);
        double tau_kadr;
        QTime tau;
        int speed;
        // Текущая дата и время
       QDateTime DT;
        int MotionMode;

        // Параметры спутника и орбиты

        double Height = 650000;                            //высота орбиты в метрах
        double Velocity = 7450.491707484375;               //модуль начальной скорости КА в м/с
        double Incline = 98;                               //наклон орбиты в градусах

        /* Координаты спутника r в:
         *      geod  - геодезической с.к.
         *      j2000 - в инерциальной с.к. J2000
         *      wgs84 - в неинерциальной с.к. WGS-84
        */

        vectord r_geod;
        vectord r_j2000;
        vectord r_wgs84;

        /* Скорость спутника v в:
         *      j2000 - в инерциальной с.к. J2000
         *      wgs84 - в неинерциальной с.к. WGS-84
        */

        vectord v_j2000;
        vectord v_wgs84;

        vectord r_kadr_geod;                                            // Радиус-вектор точки начала съемки в геодезических координатах
        vectord r_kadr_wgs84;                                           // Радиус-вектор точки начала съемки в WGS-84
        vectord r_kadr_j2000;                                           // Радиус-вектор точки начала съемки в J2000
        vectord r_kadr_KA;                                              // Радиус-вектор точки начала съемки в ск КА

        quaterniond FrJ2000toKA;                                        // кватернион перехода от системы j2000 к с.к. КА
        quaterniond FrKAtoJ2000;                                        // кватернион перехода от системы КА к с.к. J2000
        quaterniond FrKAtoPr = {cos(to_rad(45)),                        // кватернион перехода от с.к. КА к с.к. ДУС
                                0,
                                -sin(to_rad(45)),
                                0};
        matrixd1     FrPrtoIK;                                          // Матрица перехода от с. к. ДУС к показаниям ИК
        matrixd      FrWGStoJ2000;
        matrixd      FrJ2000toWGS;


        vectord AngVel_KA;                                              // Угловая скорость КА в J2000
        vectord AngVel_pr;                                              // Угловая скорость в приборной с.к.
        vectord AngVel_upr;                                             // Угловая скорость от управляющего воздействия
        vectord AngVel_ka_nv;                                           // Угловая скорость от невозмущенного движения

         // Направление визирования при нулевом тангаже и крене
        vectord zeroDir = {0,
                          1,
                          0};

        // Четырехмерный вектор с показаниями ИК
        quaterniond Ik;

    public slots:
        void NevozMotion();
        void StartParameters();
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
        void send_ik(double,double,double,double);
        void send_graph1(double,double,double,double,double);

    private slots:
        //void StartPosition(double lat, double lon, int mode, QDateTime ntd); //Кадровая съемка
        void GeoToWGS84(vectord *a, double B, double L, double H);

};

#endif // COUNT_H
