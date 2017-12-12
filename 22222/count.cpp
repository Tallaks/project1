 #include "count.h"

#include <windows.h>
#include<QCoreApplication>
#include <QTextStream>

Count::Count(Estar *parent):Estar(parent)
{
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(SetStart()));
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(ResultMotion()));

    connect(this,
            SIGNAL(send_nv(double,double,double,double,double,double,QTime)),
            parent,
            SLOT(update(double,double,double,double,double,double,QTime))
            );

    connect(this,
            SIGNAL(send_geod(double,double,double)),
            parent,
            SLOT(updategeod(double,double,double))
            );

    connect(parent->StopButton,SIGNAL(clicked()),this,SLOT(SetStop()));
    connect(parent->PauseButton,SIGNAL(clicked()),this,SLOT(SetPause()));

    connect(this,
            SIGNAL(send_kadr(double,double,double,double,double,double,double,double,double)),
            parent->DM,
            SLOT(update(double,double,double,double,double,double,double,double,double))
            );

    connect(this,SIGNAL(done()),parent,SLOT(Succesed()));
    connect(parent->SpeedUpButton,SIGNAL(clicked()),this,SLOT(speedup()));
    connect(parent->SpeedDownButton,SIGNAL(clicked()),this,SLOT(speeddown()));

    connect(this,
            SIGNAL(send_ik(double,double,double,double)),
            parent,
            SLOT(update_ik(double,double,double,double))
            );

    connect(this,
            SIGNAL(send_graph1(double,double,double,double,double)),
            parent->demo,
            SLOT(onDataTimer(double,double,double,double,double))
            );

    speed = 0;
    SetStop();
    StartParameters();
}

void Count::NevozMotion(){
    vectord v_gamma,r_gamma,omega_Orb;
    copy_v(&r_gamma,r_wgs84);
    set_v(&v_gamma,v_wgs84[0]-omegaEarth*v_wgs84[1],v_wgs84[1]+omegaEarth*v_wgs84[0],v_wgs84[2]);
    cross_v(&omega_Orb,r_wgs84,v_wgs84);
    mul_vf(&omega_Orb,omega_Orb,1/(abs_v(r_wgs84)*abs_v(r_wgs84)));
    project_v(&AngVel_ka_nv,omega_Orb,FrJ2000toKA);

    AngVel_ka_nv[0] = AngVel_ka_nv[0]*180/M_PI;
    AngVel_ka_nv[1] = AngVel_ka_nv[1]*180/M_PI;
    AngVel_ka_nv[2] = AngVel_ka_nv[2]*180/M_PI;

    emit send_nv(r_gamma[0]/1000,r_gamma[1]/1000,r_gamma[2]/1000,
            v_gamma[0],v_gamma[1],v_gamma[2],
            tau);

   j2000::IS_GS(DT.date().year(),
                DT.date().month(),
                DT.date().day(),
                DT.time().hour(),
                DT.time().minute(),
                DT.time().second(),
                 &FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);

    // Расчет невозмущенного движения методом Рунге-Кутты четвортого порядка
    DiffRungKutt(&r_gamma,&v_gamma,r_gamma,v_gamma,0.2);
    copy_v(&r_wgs84,r_gamma);
    set_v(&v_wgs84,v_gamma[0]+omegaEarth*r_gamma[1],v_gamma[1]-omegaEarth*r_gamma[0],v_gamma[2]);
    mul_mv(&r_j2000,FrWGStoJ2000,r_wgs84);                        //перевод Радиус-вектора КА из системы WGS-84 в систему j2000
    mul_mv(&v_j2000,FrWGStoJ2000,v_wgs84);
}
/*
NevozMotion - это функция класса Count, моделирующая невозмущенное движение спутника по орбите
*/

void Count::delay(int n){
    QTime dieTime = QTime::currentTime().addMSecs(n);
    while(QTime::currentTime()<dieTime){
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
}

void Count::speedup(){
    speed--;
}

void Count::speeddown(){
    speed++;
}

bool countStop = false;

int j;

void Count::ResultMotion(){
    while(!countStop){
        switch(speed){
            case 0:
                delay(10);

                break;
            case 1:
                delay(500);
                break;
            case 2:
                delay(1000);
                break;
            case 3:
                delay(2500);
                break;
            case 4:
                delay(5000);
            break;
            default:
            delay(100);
            break;
        }
        DT=DT.addMSecs(200);
        tau=tau.addMSecs(200);
        NevozMotion();
        add_v(&AngVel_KA,AngVel_ka_nv,AngVel_upr);
        AngVel_KA[0] = AngVel_KA[0]*M_PI/180;
        AngVel_KA[1] = AngVel_KA[1]*M_PI/180;
        AngVel_KA[2] = AngVel_KA[2]*M_PI/180;

        intgr_qt(&FrJ2000toKA,FrJ2000toKA,AngVel_KA,0.2);

        AngVel_KA[0] = AngVel_KA[0]*180/M_PI;
        AngVel_KA[1] = AngVel_KA[1]*180/M_PI;
        AngVel_KA[2] = AngVel_KA[2]*180/M_PI;

        project_v(&AngVel_pr,AngVel_KA,FrKAtoPr);
        mul_m1v(&Ik,FrPrtoIK,AngVel_pr);

       if(j%5 == 0){
        emit send_graph1(Ik[0],Ik[1],Ik[2],Ik[3],j/5);}
        j++;
        emit send_ik(omegaEarth,Ik[1],Ik[2],Ik[3]);
    }
}

void Count::SetStop(){
    countStop = true;
}

void Count::SetPause(){
    countStop = true;
}

void Count::SetStart(){
   countStop = false;
}

void Count::StartParameters(){
    Height = 650000;                            //высота орбиты в метрах
    Velocity = 7450.491707484375;               //модуль начальной скорости КА в м/с
    Incline = 98;                               //наклон орбиты в градусах

    // Обнуляем угловую скорость от управляющего воздействия
    AngVel_upr[0] = 0;
    AngVel_upr[1] = 0;
    AngVel_upr[2] = 0;

    //Задаем вектор геодезических координат КА в начальный момент времени
    r_geod[0] = 62.96028;
    r_geod[1] = 40.68334;
    r_geod[2] = Height;

    // Перевод из геодезических координат в систему WGS-84
    GeoToWGS84(&r_wgs84,r_geod[0],r_geod[1],r_geod[2]);
    QDateTime DT(QDateTime::currentDateTime());
    // Нахождение координат спутника в инерциальной системе отсчета
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);                                                                                       // Задание матрицы перевода из системы WGS84 в J2000
    mul_mv(&r_j2000,FrWGStoJ2000,r_wgs84);                                                                               //перевод Радиус-вектора КА из системы WGS-84 в систему j2000

    // Задание вектора скорости КА в системе WGS-84 исходя из его положения на орбите и параметров орбиты
    v_wgs84[0] = -Velocity*(r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline)))/sqrt(r_wgs84[0]*r_wgs84[0]+ (r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline)))*(r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline))));
    v_wgs84[1] = Velocity*r_wgs84[0]/sqrt(r_wgs84[0]*r_wgs84[0]+ (r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline)))*(r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline))))*cos(to_rad(Incline));
    v_wgs84[2] = Velocity*r_wgs84[0]/sqrt(r_wgs84[0]*r_wgs84[0]+ (r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline)))*(r_wgs84[1]*cos(to_rad(Incline))+r_wgs84[2]*sin(to_rad(Incline))))*sin(to_rad(Incline));

    // Перевод вектора скорости КА из системы WGS-84 в систему j2000
    mul_mv(&v_j2000,FrWGStoJ2000,v_wgs84);

    // Нахождение кватерниона перехода от системы координат J2000 к с.к. КА
    Povorot0(&FrJ2000toKA,r_j2000,v_j2000);

    // Задание матрицы перехода от с.к. ДУС к показаниям измерительных каналов ДУС
    FrPrtoIK[0][0] = cos(to_rad(54.7356));  FrPrtoIK[0][1] = cos(to_rad(45));   FrPrtoIK[0][2] = cos(to_rad(45));
    FrPrtoIK[1][0] = cos(to_rad(54.7356));  FrPrtoIK[1][1] = -cos(to_rad(45));  FrPrtoIK[1][2] = cos(to_rad(45));
    FrPrtoIK[2][0] = cos(to_rad(54.7356));  FrPrtoIK[2][1] = -cos(to_rad(45));  FrPrtoIK[2][2] = -cos(to_rad(45));
    FrPrtoIK[3][0] = cos(to_rad(54.7356));  FrPrtoIK[3][1] = cos(to_rad(45));   FrPrtoIK[3][2] = -cos(to_rad(45));

}

/* Из геодезических в WGS-84 */
void Count::GeoToWGS84( vectord *a, double B, double L, double H){
    vectord A;
    double N;
    double e = 1/298.257223563;
    N = 6378137.0/sqrt(1 - (2*e-e*e)*sin(to_rad(B))*sin(to_rad(B)));
    A[0] = (N + H)*cos(to_rad(B))*cos(to_rad(L));
    A[1] = (N + H)*cos(to_rad(B))*sin(to_rad(L));
    A[2] = ((1-e)*(1-e)*N + H)*sin(to_rad(B));
    copy_v(a,A);
}
