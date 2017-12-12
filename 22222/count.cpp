 #include "count.h"

#include <windows.h>
#include<QCoreApplication>
#include <QTextStream>

Count::Count(Estar *parent):Estar(parent)
{
    settings = new Settings;
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
}

void Count::NevozMotion(){
    vectord v_gamma,r_gamma,omega_Orb;
    copy_v(&r_gamma,settings->r_wgs84);
    set_v(&v_gamma,settings->v_wgs84[0]-omegaEarth*settings->v_wgs84[1],settings->v_wgs84[1]+omegaEarth*settings->v_wgs84[0],settings->v_wgs84[2]);

    cross_v(&omega_Orb,settings->r_j2000,settings->v_j2000);
    mul_vf(&omega_Orb,omega_Orb,1/(abs_v(settings->r_j2000)*abs_v(settings->r_j2000)));
    project_v(&settings->AngVel_ka_nv,omega_Orb,settings->FrJ2000toKA);

    settings->AngVel_ka_nv[0] = settings->AngVel_ka_nv[0]*180/M_PI;
    settings->AngVel_ka_nv[1] = settings->AngVel_ka_nv[1]*180/M_PI;
    settings->AngVel_ka_nv[2] = settings->AngVel_ka_nv[2]*180/M_PI;

    emit send_nv(r_gamma[0]/1000,r_gamma[1]/1000,r_gamma[2]/1000,
            v_gamma[0],v_gamma[1],v_gamma[2],
            tau);

   j2000::IS_GS(DT.date().year(),
                DT.date().month(),
                DT.date().day(),
                DT.time().hour(),
                DT.time().minute(),
                DT.time().second(),
                 &settings->FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&settings->FrWGStoJ2000,settings->FrJ2000toWGS);

    // Расчет невозмущенного движения методом Рунге-Кутты четвортого порядка
    DiffRungKutt(&r_gamma,&v_gamma,r_gamma,v_gamma,0.2);
    copy_v(&settings->r_wgs84,r_gamma);
    set_v(&settings->v_wgs84,v_gamma[0]+omegaEarth*r_gamma[1],v_gamma[1]-omegaEarth*r_gamma[0],v_gamma[2]);
    mul_mv(&settings->r_j2000,settings->FrWGStoJ2000,settings->r_wgs84);                        //перевод Радиус-вектора КА из системы WGS-84 в систему j2000
    mul_mv(&settings->v_j2000,settings->FrWGStoJ2000,settings->v_wgs84);
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
        add_v(&settings->AngVel_KA,settings->AngVel_ka_nv,settings->AngVel_upr);
        settings->AngVel_KA[0] = settings->AngVel_KA[0]*M_PI/180;
        settings->AngVel_KA[1] = settings->AngVel_KA[1]*M_PI/180;
        settings->AngVel_KA[2] = settings->AngVel_KA[2]*M_PI/180;

        intgr_qt(&settings->FrJ2000toKA,settings->FrJ2000toKA,settings->AngVel_KA,0.2);

        settings->AngVel_KA[0] = settings->AngVel_KA[0]*180/M_PI;
        settings->AngVel_KA[1] = settings->AngVel_KA[1]*180/M_PI;
        settings->AngVel_KA[2] = settings->AngVel_KA[2]*180/M_PI;

        project_v(&settings->AngVel_pr,settings->AngVel_KA,settings->FrKAtoPr);
        mul_m1v(&settings->Ik,settings->FrPrtoIK,settings->AngVel_pr);

       if(j%5 == 0){
        emit send_graph1(settings->Ik[0],settings->Ik[1],settings->Ik[2],settings->Ik[3],j/5);}
        j++;
      //  emit send_nev(settings->AngVel_pr[0],settings->AngVel_pr[1],settings->AngVel_pr[2],settings->DT);
        emit send_ik(settings->Ik[0],settings->Ik[1],settings->Ik[2],settings->Ik[3]);
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
