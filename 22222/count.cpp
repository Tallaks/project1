#include "count.h"
#include <windows.h>
#include<QCoreApplication>
#include <QTextStream>
#include "traj.h"

Count::Count(Estar *parent):Estar(parent)
{
    parent1 = parent;
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(SetStart()));
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(ResultMotion()));
    connect(this,SIGNAL(send_nv(double,double,double,double,double,double,
                                QTime)),parent,SLOT(update(double,double,double,double,double,
                                                                                        double,QTime)));
    connect(this,SIGNAL(send_geod(double,double,double)),parent,SLOT(updategeod(double,double,double)));
    connect(parent->StopButton,SIGNAL(clicked()),this,SLOT(SetStop()));
    connect(parent->PauseButton,SIGNAL(clicked()),this,SLOT(SetPause()));
    connect(this,SIGNAL(send_kadr(double,double,double,double,double,double,double,double,double)),parent->DM,SLOT(update(double,double,double,double,double,double,double,double,double)));
    connect(this,SIGNAL(done()),parent,SLOT(Succesed()));
    connect(this,SIGNAL(send_nev(double,double,double,QDateTime)),parent,SLOT(update1(double,double,double,QDateTime)));
    connect(parent->SpeedUpButton,SIGNAL(clicked()),this,SLOT(speedup()));
    connect(parent->SpeedDownButton,SIGNAL(clicked()),this,SLOT(speeddown()));
    connect(this,SIGNAL(send_ik(double,double,double,double)),parent,SLOT(update_ik(double,double,double,double)));
    connect(this,SIGNAL(send_graph1(double,double,double,double,double)),parent->demo,SLOT(onDataTimer(double,double,double,double,double)));

    file.setFileName("H:/projects/wgs84.txt");
    file.open(QIODevice::WriteOnly);
    file1.setFileName("H:/projects/j2000.txt");
    file1.open(QIODevice::WriteOnly);
    DT = parent->dateEdit->dateTime();
    speed = 0;
    SetStop();
}

int i = 0;

void Count::NevozMotion(){
    vectord v_gamma,r_gamma;

    copy_v(&r_gamma,r0_wgs84);
    set_v(&v_gamma,v0_wgs84[0]-omegaEarth*r0_wgs84[1],v0_wgs84[1]+omegaEarth*r0_wgs84[0],v0_wgs84[2]);

    cross_v(&omega_Orb,r0_j2000,v0_j2000);
    mul_vf(&omega_Orb,omega_Orb,1/(abs_v(r0_j2000)*abs_v(r0_j2000)));
    project_v(&omega_ka_nv,omega_Orb,FrJ2000toKA);

    omega_ka_nv[0] = omega_ka_nv[0]*180/M_PI;
    omega_ka_nv[1] = omega_ka_nv[1]*180/M_PI;
    omega_ka_nv[2] = omega_ka_nv[2]*180/M_PI;

    matrixd M_SNP;
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&M_SNP);              // Задание матрицы перевода из системы j2000 в WGS84
    transp_m(&M_SNP,M_SNP);

    // Расчет невозмущенного движения методом Рунге-Кутты четвортого порядка
    DiffRungKutt(&r_gamma,&v_gamma,r_gamma,v_gamma,0.2);
    copy_v(&r0_wgs84,r_gamma);
    set_v(&v0_wgs84,v_gamma[0]+omegaEarth*r_gamma[1],v_gamma[1]-omegaEarth*r_gamma[0],v_gamma[2]);
    WGS84ToGeo(&r0_geod,r0_wgs84[0],r0_wgs84[1],r0_wgs84[2]);
    mul_mv(&r0_j2000,M_SNP,r0_wgs84);                        //перевод Радиус-вектора КА из системы WGS-84 в систему j2000
    mul_mv(&v0_j2000,M_SNP,v0_wgs84);

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
    DT = parent1->dateEdit->dateTime();
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

        add_v(&omega_KA,omega_ka_nv,omega_upr);
        omega_KA[0] = omega_KA[0]*M_PI/180;
        omega_KA[1] = omega_KA[1]*M_PI/180;
        omega_KA[2] = omega_KA[2]*M_PI/180;

        intgr_qt(&FrJ2000toKA,FrJ2000toKA,omega_KA,0.1);

        omega_KA[0] = omega_KA[0]*180/M_PI;
        omega_KA[1] = omega_KA[1]*180/M_PI;
        omega_KA[2] = omega_KA[2]*180/M_PI;

        project_v(&omega_pr,omega_KA,FrKAtoPr);
        mul_m1v(&Ik,FrPrtoIK,omega_pr);

        emit send_nv(r0_j2000[0]/1000.0,r0_j2000[1]/1000.0,r0_j2000[2]/1000.0,
                omega_KA[0],omega_KA[1],omega_KA[2],
                tau);
       if(j%5 == 0){
        emit send_graph1(Ik[0],Ik[1],Ik[2],Ik[3],j/5);}
        j++;
        emit send_nev(omega_pr[0],omega_pr[1],omega_pr[2],tdn);
        emit send_ik(Ik[0],Ik[1],Ik[2],Ik[3]);
        emit send_geod(r0_geod[0]*180/M_PI,r0_geod[1]*180/M_PI,r0_geod[2]/1000);
        QTextStream ts(&file);
        ts.setFieldWidth(10);
        ts.setFieldAlignment(QTextStream::AlignLeft);

        ts << r0_wgs84[0]/1000  << r0_wgs84[1]/1000 << r0_wgs84[2]/1000 <<  DT.time().toString("hh:mm:ss") <<"\n";

        QTextStream ts1(&file1);
        ts1.setFieldWidth(10);
        ts1.setFieldAlignment(QTextStream::AlignLeft);

        ts1 << r0_j2000[0]/1000  << r0_j2000[1]/1000 << r0_j2000[2]/1000 <<  DT.time().toString("hh:mm:ss") <<"\n";

    }
}

void Count::SetStop(){
    countStop = true;

    this->SetStartParameters(30,30);
}

void Count::SetPause(){
    countStop = true;
}

void Count::SetStart(){
   countStop = false;
}



void Count::SetStartParameters(double lon,double lat){
    MotionMode = 0;
    omega_upr[0] = 0;
    omega_upr[1] = 0;
    omega_upr[2] = 0;
    tau.setHMS(0,0,0,0);
    matrixd M_SNP;
    double Height,Velocity,Incline;

    Height = 650000;                    //высота орбиты в метрах
    Velocity = 7450.491707484375;       //модуль скорости КА в м/с
    Incline = 98;                       //наклон орбиты в градусах

    //Задаем вектор геодезических координат КА в начальный момент времени
    r0_geod[0] = lat;//62.96028;
    r0_geod[1] = lon;//40.68334;
    r0_geod[2] = Height;

    GeoToWGS84(&r0_wgs84,r0_geod[0],r0_geod[1],r0_geod[2]);                                                        // Перевод из геодезических координат в систему WGS-84
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&M_SNP);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&M_SNP,M_SNP);                                                                                       // Задание матрицы перевода из системы WGS84 в J2000
    mul_mv(&r0_j2000,M_SNP,r0_wgs84);                                                                               //перевод Радиус-вектора КА из системы WGS-84 в систему j2000

    // Задание вектора скорости КА в системе WGS-84 с учетом его положения на орбите и параметров орбиты
    v0_wgs84[0] = -Velocity*(r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline)))/sqrt(r0_wgs84[0]*r0_wgs84[0]+ (r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline)))*(r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline))));
    v0_wgs84[1] = Velocity*r0_wgs84[0]/sqrt(r0_wgs84[0]*r0_wgs84[0]+ (r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline)))*(r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline))))*cos(to_rad(Incline));
    v0_wgs84[2] = Velocity*r0_wgs84[0]/sqrt(r0_wgs84[0]*r0_wgs84[0]+ (r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline)))*(r0_wgs84[1]*cos(to_rad(Incline))+r0_wgs84[2]*sin(to_rad(Incline))))*sin(to_rad(Incline));
    mul_mv(&v0_j2000,M_SNP,v0_wgs84);                        //перевод вектора скорости КА из системы WGS-84 в систему j2000

    Povorot0(&FrJ2000toKA,r0_j2000,v0_j2000);

    FrPrtoIK[0][0] = cos(to_rad(54.7356));
    FrPrtoIK[0][1] = cos(to_rad(45));
    FrPrtoIK[0][2] = cos(to_rad(45));

    FrPrtoIK[1][0] = cos(to_rad(54.7356));
    FrPrtoIK[1][1] = -cos(to_rad(45));
    FrPrtoIK[1][2] = cos(to_rad(45));

    FrPrtoIK[2][0] = cos(to_rad(54.7356));
    FrPrtoIK[2][1] = -cos(to_rad(45));
    FrPrtoIK[2][2] = -cos(to_rad(45));

    FrPrtoIK[3][0] = cos(to_rad(54.7356));
    FrPrtoIK[3][1] = cos(to_rad(45));
    FrPrtoIK[3][2] = -cos(to_rad(45));
}
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
