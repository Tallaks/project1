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
#include "emath.h"
#include "j2000.h"
#include "estar.h"
#include <QFile>


class Count : public Estar
{
    Q_OBJECT

    public:
        explicit Count(Estar *parent = Q_NULLPTR);
        vectord r0_geod;
        vectord r0_j2000;           // Радиус-вектор КА в системе j2000
        vectord v0_j2000;           // Скорость КА в системе j2000
        vectord r0_wgs84;           // Радиус-вектор КА в системе WGS-84
        vectord v0_wgs84;           // Скорость КА в системе WGS-84
        vectord r0_KA;
        vectord v0_KA;
        vectord r_kadr_geod;        // Радиус-вектор точки начала съемки в геодезических координатах
        vectord r_kadr_wgs84;       // Радиус-вектор точки начала съемки в WGS-84
        vectord r_kadr_j2000;       // Радиус-вектор точки начала съемки в J2000
        vectord r_kadr_Orb;          // Радиус-вектор точки начала съемки в ОСК
        vectord r_kadr_KA;
        Estar *parent1;
        quaterniond FrJ2000toKA;    // кватернион поворота от системы j2000 к с.к. КА в произвольный момент времени
        quaterniond FrKAtoJ2000;
        quaterniond FrKA1toOrb;  // кватернион поворота от требуемой системы КА к ОСК
        quaterniond FrOrbtoKA1;  // кватернион поворота от системы ОСК к требуемой КА
        quaterniond FrKA1toKA;   // кватернион поворота от требуемой системы КА к КА
        quaterniond FrKAtoKA1;   // кватернион поворота от системы КА к требуемой системе КА
        quaterniond FrKAtoPr = {cos(to_rad(45)),0,-sin(to_rad(45)),0};

        matrixd1     FrPrtoIK;
        quaterniond  Ik;

        vectord omega_Orb;
        vectord omega_KA;        // угловая скорость КА в J2000
        vectord omega_pr;
        vectord omega_upr;
        vectord omega_ka_nv;        // Угловая скорость от невозмущенного движения

        double g_lat;
        double g_lon;
        double t0;
        double k0;
        double kren_fin;
        double tang_fin;
        QDateTime DT;
        QDateTime tdn;
        double tau_kadr;
        QTime tau;
        int speed;
        void SetStartParameters(double lon, double lat);
        /*
        SetStartParameters - это функция класса Count, задающая параметры движения КА в начальный момент времени
        Входные переменные:
            lon         - геодезическая долгота
            lat         - геодезическая широта
        Выходные данные:
            omega_j2000 - Вектор угловой скорости КА в системе J2000 (вращение вокруг Земли)
            M0          - Кватернион поворота от системы j2000 в систему КА
            r0_j2000    - Радиус-вектор КА в системе j2000 в начальный момент времени
            v0_j2000    - Скорость КА в системе j2000 в начальный момент времени
        */

        int MotionMode;
        QFile file,file1;

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
        void send_nev(double,double,double,QDateTime);
        void send_ik(double,double,double,double);
        void send_graph1(double,double,double,double,double);

};

#endif // COUNT_H
