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
#include "j2000.h"
#include "estar.h"
#include <QFile>

using namespace Trajectory;

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
        Estar *parent1;
        quaterniond FrJ2000toKA;    // кватернион поворота от системы j2000 к с.к. КА в произвольный момент времени
        quaterniond FrJ2000toKA_n1;
        quaterniond FrJ2000toKA_n2;
        quaterniond FrJ2000toKA_n3;
        quaterniond FrKAtoJ2000;
        quaterniond FrKAtoPr = {cos(to_rad(45)),0,-sin(to_rad(45)),0};
        matrixd FrJ2000toWGS;

        matrixd1     FrPrtoIK;
        quaterniond  Ik;

        vectord omega_Orb;
        vectord omega_KA;        // угловая скорость КА в J2000
        vectord omega_KA_Nm1;
        vectord omega_KA_Nm2;
        vectord omega_pr;
        vectord omega_upr;
        vectord omega_ka_nv;        // Угловая скорость от невозмущенного движения
        vectord dir;
        vectord kadr_WGS84;

        double g_lat;
        double g_lon;
        double delta_phi;

        QDateTime DT;
        QDateTime tdn;
        QTime tau;
        double time;
        int speed;

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

        void SetStartParameters(double lon, double lat);



        void currentDirection(ModeDesc *data);

        int MotionMode;
        int NextMode;
        int start;
        int line_number;

        QFile file,file1,file2;
        Settings settings;
        ModeDesc mode_desc;
        ITrajectory traj;
        MotionDesc pos;
        MotionDesc reqpos;


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
        void InitTest(double b, double l, int mode);
        void KadrMotion();
        void PloshadMotion();
        void StereoMotion();


    signals:
        void send_graph_KA(double,double,double,double);
        void send_nv(double,double,double,double,double,double,QTime);
        void send_m(int);
        void done();
        void send_geod(double,double,double);
        void send_geod_point(double,double,double);
        void send_pr(double,double,double);
        void send_ik(double,double,double,double);
        void send_graphIK(double,double,double,double,double);
        void send_graph_PR(double,double,double,double);
        void send_kadr_motion_status(int);
        void send_pos(double,double);
        void send_rotation_opengl(double,double,double,double);
        void send_ka_pos_opengl(double,double,double);

};

#endif // COUNT_H
