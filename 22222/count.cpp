#include "count.h"
#include <windows.h>
#include<QCoreApplication>
#include <QTextStream>

Count::Count(Estar *parent):Estar(parent){
    parent1 = parent;
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(SetStart()));
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(ResultMotion()));
    connect(parent->StopButton,SIGNAL(clicked()),this,SLOT(SetStop()));
    connect(parent->PauseButton,SIGNAL(clicked()),this,SLOT(SetPause()));
    connect(parent->SpeedUpButton,SIGNAL(clicked()),this,SLOT(speedup()));
    connect(parent->SpeedDownButton,SIGNAL(clicked()),this,SLOT(speeddown()));

    connect(
            this,
            SIGNAL(send_ka_pos_opengl(double,double,double)),
            parent->MW->c,SLOT(setKAPosition(double,double,double))
            );

    connect(
            this,
            SIGNAL(send_rotation_opengl(double,double,double,double)),
            parent->MW->c,SLOT(setKAOrientation(double,double,double,double))
            );

    connect(this,SIGNAL(send_kadr_motion_status(int)),parent->DM,SLOT(updateStatus(int)));
    connect(this,SIGNAL(send_pos(double,double)),parent->DM,SLOT(updatePos(double,double)));
    /*  Связь данных о координатах, угловой скорости и времени полета КА
     *  с соответствующими ячейками в главном окне
     */

    connect(
            this,
            SIGNAL(send_nv(double,double,double,double,double,double,QTime)),
            parent,
            SLOT(update(double,double,double,double,double,double,QTime))
            );

    /*
     *  Связь данных о геодезических координатах
     *  с соответсвующими ячейками в главном окне
     */

    connect(this,SIGNAL(send_geod(double,double,double)),parent,SLOT(updategeod(double,double,double)));

    /*
     *  Отправка сигнала для выдачи сообщения об успешной съемке
     */

    connect(this,SIGNAL(done()),parent,SLOT(Succesed()));

    /*
     *  Связь данных об угловой скорости в приборной с. к.
     *  с соответствующими ячейками в главном окне
     */

    connect(this,SIGNAL(send_pr(double,double,double)),parent,SLOT(update1(double,double,double)));

    /*
     *  Связь показаний измерительных каналов
     *  с соответствующими ячейками в главном окне
     */

    connect(this,SIGNAL(send_ik(double,double,double,double)),parent,SLOT(update_ik(double,double,double,double)));

    /*
     *  Перенос нужных данных в графическое окно
     */

    connect(
            this,
            SIGNAL(send_graphIK(double,double,double,double,double)),
            parent->IK_Graph,
            SLOT(onDataTimer(double,double,double,double,double))
            );

    /*
     *  Перенос нужных данных в графическое окно
     */

    connect(
            this,
            SIGNAL(send_graph_PR(double,double,double,double)),
            parent->PR_Graph,
            SLOT(onDataTimer(double,double,double,double))
            );

    /*
     *  Перенос нужных данных в графическое окно
     */

    connect(
            this,
            SIGNAL(send_graph_KA(double,double,double,double)),
            parent->KA_Graph,
            SLOT(onDataTimer(double,double,double,double))
            );

    /*
     *  Связь данных о геодезических координатах точки визирования
     *  с соответствующими ячейками в окне режима съемки
     */

    connect(this,SIGNAL(send_geod_point(double,double,double)),parent->DM,SLOT(updateGeodPoint(double,double,double)));

    /*
     *  Сигнал с начальными данными в виде геодезических координат
     *  начала съемки и типа съемки передается в функцию,
     * инициализирующую движение
     */

    connect(
            parent,
            SIGNAL(send_kadr_position(double,double,int)),
            this,
            SLOT(InitTest(double,double,int))
            );

    file.setFileName("F:/projects/mistake_Euler_1.txt");
    file.open(QIODevice::WriteOnly);
    file1.setFileName("F:/projects/vector_vTel_wgs.txt");
    file1.open(QIODevice::WriteOnly);
    file2.setFileName("F:/projects/vector_vView.txt");
    file2.open(QIODevice::WriteOnly);
    DT = parent->dateEdit->dateTime();
    speed = 0;
    SetStop();
}

int i = 0;

void Count::NevozMotion(){
    vectord v_gamma,r_gamma;

    copy_v(&r_gamma,r0_wgs84);
    set_v(&v_gamma,v0_wgs84[0]-omegaEarth*r0_wgs84[1],v0_wgs84[1]+omegaEarth*r0_wgs84[0],v0_wgs84[2]);

    matrixd M_SNP;
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&M_SNP,FrJ2000toWGS);
    // Расчет невозмущенного движения методом Рунге-Кутты четвортого порядка
    DiffRungKutt(&r_gamma,&v_gamma,r_gamma,v_gamma,0.2);
    copy_v(&r0_wgs84,r_gamma);
    set_v(&v0_wgs84,v_gamma[0]+omegaEarth*r_gamma[1],v_gamma[1]-omegaEarth*r_gamma[0],v_gamma[2]);
    WGS84ToGeo(&r0_geod,r0_wgs84[0],r0_wgs84[1],r0_wgs84[2]);
    mul_mv(&r0_j2000,M_SNP,r0_wgs84);                        //перевод Радиус-вектора КА из системы WGS-84 в систему j2000
    mul_mv(&v0_j2000,M_SNP,v0_wgs84);
    ModeDesc vis;
    currentDirection(&vis);

    cross_v(&omega_Orb,r0_j2000,v0_j2000);
    mul_vf(&omega_Orb,omega_Orb,1/(abs_v(r0_j2000)*abs_v(r0_j2000)));
    project_v(&omega_ka_nv,omega_Orb,FrJ2000toKA);

    omega_ka_nv[0] = omega_ka_nv[0]*180/M_PI;
    omega_ka_nv[1] = omega_ka_nv[1]*180/M_PI;
    omega_ka_nv[2] = omega_ka_nv[2]*180/M_PI;

    emit send_geod_point(vis.b*180/M_PI,vis.l*180/M_PI,vis.h/1000);
}

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
        // Переключатель скорости обработки информации
        if(speed > 6){speed = 6;}
        else{
            if(speed <0){
                speed = 0;
            }
            else{
                switch(speed){
                    case 0:
                        delay(5);
                        break;
                    case 1:
                        delay(20);
                        break;
                    case 2:
                        delay(100);
                        break;
                    case 3:
                        delay(200);
                        break;
                    case 4:
                        delay(500);
                        break;
                    case 5:
                        delay(1000);
                        break;
                    case 6:
                        delay(2000);
                        break;
                }
            }
        }
        NevozMotion();
        DT=DT.addMSecs(200);
        tau=tau.addMSecs(200);
        if(MotionMode == 1){
            KadrMotion();
        }
        if(MotionMode == 2){
            PloshadMotion();
        }
        add_v(&omega_KA,omega_ka_nv,omega_upr);
        omega_KA[0] = omega_KA[0]*M_PI/180;
        omega_KA[1] = omega_KA[1]*M_PI/180;
        omega_KA[2] = omega_KA[2]*M_PI/180;

        //intgr_qt_Euler_1(&FrJ2000toKA,FrJ2000toKA,omega_KA,0.2);
        //intgr_qt_Euler_2_norm(&FrJ2000toKA,FrJ2000toKA,omega_KA,0.2);
        //intgr_qt(&FrJ2000toKA,FrJ2000toKA,omega_KA,0.2);
        intgr_qt_meanv3(&FrJ2000toKA,FrJ2000toKA,omega_KA,omega_KA_Nm1,0.2);
        //intgr_qt_stiltyes(&FrJ2000toKA,FrJ2000toKA,FrJ2000toKA_n1,FrJ2000toKA_n2,FrJ2000toKA_n3,omega_KA,0.2);
        conj_q(&FrKAtoJ2000,FrJ2000toKA);

        omega_KA[0] = omega_KA[0]*180/M_PI;
        omega_KA[1] = omega_KA[1]*180/M_PI;
        omega_KA[2] = omega_KA[2]*180/M_PI;

        project_v(&omega_pr,omega_KA,FrKAtoPr);
        mul_m1v(&Ik,FrPrtoIK,omega_pr);
        emit send_nv(r0_j2000[0]/1000.0,r0_j2000[1]/1000.0,r0_j2000[2]/1000.0,
                omega_KA[0],omega_KA[1],omega_KA[2],
                tau);
        delta_phi=Mistake_Kinemtic_MeanV3(omega_KA,omega_KA_Nm1,omega_KA_Nm2,0.2);
       if(j%5 == 0){
       // emit send_graphIK(Ik[0],Ik[1],Ik[2],Ik[3],j/5);
        emit send_graphIK(FrJ2000toKA_n1[0],FrJ2000toKA_n1[1],FrJ2000toKA_n1[2],FrJ2000toKA_n1[3],j/5);
        emit send_graph_KA(omega_KA[0],omega_KA[1],omega_KA[2],j/5);
        emit send_graph_PR(omega_pr[0],omega_pr[1],omega_pr[2],j/5);
        emit send_rotation_opengl(FrKAtoJ2000[0],FrKAtoJ2000[1],FrKAtoJ2000[2],FrKAtoJ2000[3]);
        emit send_ka_pos_opengl(r0_j2000[0],r0_j2000[1],r0_j2000[2]);
       }
        j++;
        emit send_pr(omega_pr[0],omega_pr[1],omega_pr[2]);
        emit send_ik(Ik[0],Ik[1],Ik[2],Ik[3]);
        emit send_geod(r0_geod[0]*180/M_PI,r0_geod[1]*180/M_PI,r0_geod[2]/1000);
        emit send_pos(pos.posK,pos.posT);
        QTextStream ts(&file);
        if(delta_phi>pow(10.0,-16)){
        ts << delta_phi  << ";" << (double)(tau.msec())/1000 + (double)(tau.second()) + (double)(tau.minute())*60.0 + (double)(tau.hour())*3600.0 <<"\n";
        }
        copy_v(&omega_KA_Nm2,omega_KA_Nm1);
        copy_v(&omega_KA_Nm1,omega_KA);
        copy_q(&FrJ2000toKA_n3,FrJ2000toKA_n2);
        copy_q(&FrJ2000toKA_n2,FrJ2000toKA_n1);
        copy_q(&FrJ2000toKA_n1,FrJ2000toKA);
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

void Count::SetStartParameters(double lon,double lat){
    MotionMode = 0;
    omega_upr[0] = 0;
    omega_upr[1] = 0;
    omega_upr[2] = 0;
    tau.setHMS(0,0,0,0);
    double Height,Velocity,Incline;

    Height = 650000;                    //высота орбиты в метрах
    Velocity = 7450.491707484375;       //модуль скорости КА в м/с
    Incline = 98;                       //наклон орбиты в градусах

    //Задаем вектор геодезических координат КА в начальный момент времени
    r0_geod[0] = lat;//62.96028;
    r0_geod[1] = lon;//40.68334;
    r0_geod[2] = Height;
    matrixd M_SNP;
    GeoToWGS84(&r0_wgs84,r0_geod[0],r0_geod[1],r0_geod[2]);                                                        // Перевод из геодезических координат в систему WGS-84
    DT = QDateTime::currentDateTime();
     // Задание матрицы перевода из системы j2000 в WGS84
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&FrJ2000toWGS);
    inverse_m(&M_SNP,FrJ2000toWGS);                                                                                       // Задание матрицы перевода из системы WGS84 в J2000
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

    pos.posK = 0;
    pos.posT = 0;
    pos.velK = 0;
    pos.velT = 0;
    traj.currentPosition.posK = 0;
    traj.currentPosition.posT = 0;
    traj.currentPosition.velK = 0;
    traj.currentPosition.velT = 0;
    set_v(&settings.vTel,0,1,0);
    delta_phi = 0;

    cross_v(&omega_Orb,r0_j2000,v0_j2000);
    mul_vf(&omega_Orb,omega_Orb,1/(abs_v(r0_j2000)*abs_v(r0_j2000)));
    project_v(&omega_ka_nv,omega_Orb,FrJ2000toKA);
    copy_v(&omega_KA_Nm1,omega_ka_nv);
    copy_v(&omega_KA_Nm2,omega_ka_nv);
    copy_q(&FrJ2000toKA_n1,FrJ2000toKA);
    copy_q(&FrJ2000toKA_n2,FrJ2000toKA_n1);
    copy_q(&FrJ2000toKA_n3,FrJ2000toKA_n2);
}

/*
 *  InitTest - функция инициализации режима съемки
 *  Входные переменные:
 *      b - геодезическая широта начальной точки съемки
 *      l - геодезическая долгота начальной точки съемки
 *      mode - тип съемки
 *  Выходные переменные:
 *      dir_KA -
 */

void Count::InitTest(double b, double l, int mode){
    vectord dir_WGS84, dir_J2000, dir_KA;
    matrixd FrWGStoJ2000;
    // Перевод геодезических координат точки съемки в географические
    GeoToWGS84(&kadr_WGS84,b,l,0.0);
    // Нахождение вектора от КА до точки съемки
    sub_v(&dir_WGS84,kadr_WGS84,r0_wgs84);
    // Задание матрицы перевода из гринвичской с.к. в J2000
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);
    // Нахождение вектора от точки съемки до КА в J2000
    mul_mv(&dir_J2000,FrWGStoJ2000,dir_WGS84);
    // Нахождение вектора от точки съемки до КА в с.к. КА
    project_v(&dir_KA,dir_J2000,FrJ2000toKA);
    copy_v(&dir,dir_KA);
    switch(mode){
        case 1:
            traj.step = -1;
            if(MotionMode == 0){
                MotionMode = 1;
            }else{
                NextMode = 1;
            }
        break;
        case 2:
            traj.step = -1;
            if(MotionMode == 0){
                MotionMode = 2;
            }else{
                NextMode = 2;
            }
        break;
    }
    time = 0;
    start = 0;
}

void Count::KadrMotion(){
    vectord dir_WGS84, dir_J2000, dir_KA;
    matrixd FrWGStoJ2000;
    MotionDesc frs;
    // Нахождение вектора от КА до точки съемки
    sub_v(&dir_WGS84,kadr_WGS84,r0_wgs84);
    // Задание матрицы перевода из гринвичской с.к. в J2000
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);
    // Нахождение вектора от точки съемки до КА в J2000
    mul_mv(&dir_J2000,FrWGStoJ2000,dir_WGS84);
    // Нахождение вектора от точки съемки до КА в с.к. КА
    project_v(&dir_KA,dir_J2000,FrJ2000toKA);
    copy_v(&dir,dir_KA);
    vectord z_ka,z_j,y_ka,y_j;
    conj_q(&FrKAtoJ2000,FrJ2000toKA);
    set_v(&z_ka,0,0,1);
    set_v(&y_ka,0,1,0);
    project_v(&z_j,z_ka,FrKAtoJ2000);
    double scalz = dot_v(z_j,dir_J2000)/(abs_v(dir_J2000));
    double angle_dz = acos(scalz)*180/M_PI;
    project_v(&y_j,y_ka,FrKAtoJ2000);
    double scaly = dot_v(y_j,dir_J2000)/(abs_v(dir_J2000));
    double angle_dy = acos(scaly)*180/M_PI;
    vectord dir1,v;
    project_v(&v,v0_j2000,FrJ2000toKA);
    set_v(&dir1,dir[0]+v[0]/abs_v(v)*300000,dir[1]+v[1]/abs_v(v)*300000,dir[2]+v[2]/abs_v(v)*300000);
    double posK = atan(dir1[0]/dir1[1])*180/M_PI;
    switch(traj.step){
        case -1:
        if((angle_dz < 135)&&(abs_v(dir)/1000 < 2400)&&(fabs(angle_dy) < 45)){
            frs.posK = posK;
            frs.posT = 30;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,traj.currentPosition,frs,0);
            memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
            traj.step++;
            start = 1;
            emit send_pos(pos.posK,pos.posT);
        }
            break;
        case 0:
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step++;
            }
            emit send_pos(pos.posK,pos.posT);
            break;
        case 1:
            emit done();
            time = 0;
            traj.step++;
            break;
        case 2:
            frs.posK = 0;
            frs.posT = 0;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,pos,frs,0);
            traj.step++;
            emit send_pos(pos.posK,pos.posT);
            break;
        case 3:
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step++;
                start = 2;
            }
            break;
    }
    emit send_kadr_motion_status(traj.step);
    if((start == 1)||(start == 2)){time += 0.02;}
    if(start == 2){
        time = 0;
        MotionMode = NextMode;
    }
    emit send_pos(pos.posK,pos.posT);
    omega_upr[0] = pos.velT;
    omega_upr[2] = pos.velK;
}

void Count::PloshadMotion(){
    vectord dir_WGS84, dir_J2000, dir_KA;
    matrixd FrWGStoJ2000;
    MotionDesc frs;
    // Нахождение вектора от КА до точки съемки
    sub_v(&dir_WGS84,kadr_WGS84,r0_wgs84);
    // Задание матрицы перевода из гринвичской с.к. в J2000
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);
    // Нахождение вектора от точки съемки до КА в J2000
    mul_mv(&dir_J2000,FrWGStoJ2000,dir_WGS84);
    // Нахождение вектора от точки съемки до КА в с.к. КА
    project_v(&dir_KA,dir_J2000,FrJ2000toKA);
    copy_v(&dir,dir_KA);
    vectord z_ka,z_j,y_ka,y_j;
    conj_q(&FrKAtoJ2000,FrJ2000toKA);
    set_v(&z_ka,0,0,1);
    set_v(&y_ka,0,1,0);
    project_v(&z_j,z_ka,FrKAtoJ2000);
    double scalz = dot_v(z_j,dir_J2000)/(abs_v(dir_J2000));
    double angle_dz = acos(scalz)*180/M_PI;
    project_v(&y_j,y_ka,FrKAtoJ2000);
    double scaly = dot_v(y_j,dir_J2000)/(abs_v(dir_J2000));
    double angle_dy = acos(scaly)*180/M_PI;
    vectord dir1,v;
    project_v(&v,v0_j2000,FrJ2000toKA);
    set_v(&dir1,dir[0]+v[0]/abs_v(v)*300000,dir[1]+v[1]/abs_v(v)*300000,dir[2]+v[2]/abs_v(v)*300000);
    double posK = atan(dir1[0]/dir1[1])*180/M_PI;
    vectord dir2;
    switch(traj.step){
        case -1:
        // Расчет траектории переброса на начальную точку съемки
            if((angle_dz < 135)&&(abs_v(dir)/1000 < 2400)&&(fabs(angle_dy) < 45)&&(dir1[0]/1000 < 670)){
                frs.posK = posK;
                frs.posT = 30;
                frs.velK = 0;
                frs.velT = 0;
                traj.relocationTraj.calcSpline(2,1.2,traj.currentPosition,frs,0);
                memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
                traj.step++;
                start = 1;
                copy_v(&dir2,dir1);
                line_number = 0;
            }
          //  emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 0:
        // Переброс на начальную точку съемки
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step++;
            }
            //emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 1:
        // Первая полоса площадной съемки
            posK = atan(dir2[0]/dir2[1])*180/M_PI;
            time = 0;
            frs.posK = atan((720*tan(posK)+line_number*12)/720);
            frs.posT = atan((720*tan(30*M_PI/180)-60)/720)*180/M_PI;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,pos,frs,0);
            memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
            traj.step++;
            start = 1;
           // emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 2:
        // Движение по полосе
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step++;
            }
          //  emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 3:
        // Расчет траектории возврата по полосе
            posK = atan(dir2[0]/dir2[1])*180/M_PI;
            time = 0;
            frs.posK = atan((720*tan(posK)+line_number*12)/720);
            frs.posT = 30;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,pos,frs,0);
            memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
            traj.step ++;
            start = 1;
          //  emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 4:
        // Возврат по полосе
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                if(line_number < 4){
                    traj.step = 5;
                }else{
                    traj.step = 7;
                }
            }
          //  emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 5:
            posK = atan(dir2[0]/dir2[1])*180/M_PI;
            time = 0;
            frs.posK = atan((720*tan(posK)+line_number*12)/720);
            frs.posT = 30;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,pos,frs,0);
            memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
            traj.step++;
            line_number++;
            start = 1;
         //   emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 6:
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step = 1;
            }
         //   emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 7:
        // Окончание движения, возврат в нулевое положение
            time = 0;
            frs.posK = 0;
            frs.posT = 0;
            frs.velK = 0;
            frs.velT = 0;
            traj.relocationTraj.calcSpline(2,1.2,pos,frs,0);
            memcpy(&pos, &traj.currentPosition, sizeof(MotionDesc));
            traj.step ++;
            start = 1;
//            emit send_geod_point(pos.posK,pos.posT,traj.step);
            break;
        case 8:
            traj.relocationTraj.getTrajectory(time,&pos);
            if(time>=traj.relocationTraj.getRelocationTime()){
                traj.step++;
                start = 2;
                emit done();
            }
            break;
    }
  //  emit send_motion_status(traj.step);
    if((start == 1)||(start == 2)){time += 0.02;}
    if(start == 2){
        time = 0;
        MotionMode = NextMode;
    }
   // emit send_geod_point(30 - atan((720*tan(30*M_PI/180)-60)/720)*180/M_PI,pos.posT,time);
    omega_upr[0] = pos.velT;
    omega_upr[2] = pos.velK;
}

void  Count::StereoMotion(){

}

/*
 *  currentDirection -
 *  функция класса Trajectory, выдающая значение
 *  геодезических координат точки пересечения
 *  линии визирования и поверхности земного эллипсоида
 */

void Count::currentDirection(ModeDesc *data){
         // Присваиваем текущий вектор источника исходному вектору состояния КА в WGS84
         vectord SrcWGS84;
         copy_v(&SrcWGS84,r0_wgs84);

         quaterniond FrJ2000toWGSq;
         quaterniond_m(&FrJ2000toWGSq,FrJ2000toWGS);

         quaterniond FrKAtoWGS;
         conj_q(&FrKAtoJ2000,FrJ2000toKA);
         mul_q(&FrKAtoWGS,FrKAtoJ2000,FrJ2000toWGSq);
         norm_q(&FrKAtoWGS,FrKAtoWGS);

         vectord vTel_ka,vTel_wgs;
         copy_v(&vTel_ka,settings.vTel);
         project_v(&vTel_wgs,vTel_ka,FrKAtoWGS);

         // Ищем точки пересечения оси визирования телескопа с поверхностью Земли на определенной высоте

         double e = 1/298.257223563;
         double polR = 6378137.0*(1-e);
         double equR = 6378137.0;
         double polR2 = polR * polR;
         double equR2 = equR * equR;
         // Вычисление коэффициентов квадратного уравнения
         double t=0;
         double c = (SrcWGS84[0]*SrcWGS84[0] + SrcWGS84[1]*SrcWGS84[1])/equR2 + (SrcWGS84[2]*SrcWGS84[2])/polR2 - 1.0;
         double b = 2.0 * (SrcWGS84[0]*vTel_wgs[0]/equR2 + SrcWGS84[1]*vTel_wgs[1]/equR2 + SrcWGS84[2]*vTel_wgs[2]/polR2);
         double a = vTel_wgs[0]*vTel_wgs[0] / equR2 + vTel_wgs[1]*vTel_wgs[1] / equR2 + vTel_wgs[2]*vTel_wgs[2] / polR2;
         double D = b*b - 4.0*a*c;
         if(D<0.0)
         {
             t=0;
         }
         else if(D==0.0)
         {
            t = -b/(2.0*a);
         }
         else
         {
            double t1,t2;
            t1 = (-b+sqrt(D))/(2.0*a);
            t2 = (-b-sqrt(D))/(2.0*a);
            // Среди двух решений выбираем минимальное по модулю
            if(fabs(t1)>fabs(t2))
                t = t2;
            else
                t = t1;
         }

         // Получаем координаты точки пересечения линии визирования с поверхностью эллипсоида
         vectord pointCoord={SrcWGS84[0] + t*vTel_wgs[0], SrcWGS84[1] + t*vTel_wgs[1], SrcWGS84[2] + t*vTel_wgs[2]};
         // Переводим декартовы координаты в геодезические
         vectord geodes;
         WGS84ToGeo(&geodes,pointCoord[0],pointCoord[1],pointCoord[2]);

//         data.md = dynModeEarth;
         data->l = geodes[0];
         data->b = geodes[1];
         data->h = geodes[2];

         /*QTextStream ts(&file);
         ts << r0_wgs84[0]/1000  <<";"<< r0_wgs84[1]/1000 <<";"<< r0_wgs84[2]/1000 <<";"<<  tau.toString("hh:mm:ss") <<"\n";

         QTextStream ts1(&file1);
         ts1 << vTel_wgs[0]/1000  <<";"<< vTel_wgs[1]/1000 <<";"<< vTel_wgs[2]/1000 <<";"<<  tau.toString("hh:mm:ss") <<"\n";

         QTextStream ts2(&file2);
         ts2 << FrKAtoWGS[0]  <<";"<< FrKAtoWGS[1] <<";"<< FrKAtoWGS[2] <<";"<< FrKAtoWGS[3] <<";"<<  tau.toString("hh:mm:ss") <<"\n";*/
      }
