#include "count.h"
#include <windows.h>
#include<QCoreApplication>
#include <QTextStream>

Count::Count(Estar *parent):Estar(parent)
{
    parent1 = parent;
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(SetStart()));
    connect(parent->StartButton,SIGNAL(clicked()),this,SLOT(ResultMotion()));
    connect(parent->StopButton,SIGNAL(clicked()),this,SLOT(SetStop()));
    connect(parent->PauseButton,SIGNAL(clicked()),this,SLOT(SetPause()));
    connect(parent->SpeedUpButton,SIGNAL(clicked()),this,SLOT(speedup()));
    connect(parent->SpeedDownButton,SIGNAL(clicked()),this,SLOT(speeddown()));

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

    file.setFileName("F:/projects/Geod.txt");
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
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
    inverse_m(&M_SNP,FrJ2000toWGS);

    // Расчет невозмущенного движения методом Рунге-Кутты четвортого порядка
    DiffRungKutt(&r_gamma,&v_gamma,r_gamma,v_gamma,0.2);
    copy_v(&r0_wgs84,r_gamma);
    set_v(&v0_wgs84,v_gamma[0]+omegaEarth*r_gamma[1],v_gamma[1]-omegaEarth*r_gamma[0],v_gamma[2]);
    WGS84ToGeo(&r0_geod,r0_wgs84[0],r0_wgs84[1],r0_wgs84[2]);
    mul_mv(&r0_j2000,M_SNP,r0_wgs84);                        //перевод Радиус-вектора КА из системы WGS-84 в систему j2000
    mul_mv(&v0_j2000,M_SNP,v0_wgs84);

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
        if(speed > 4){speed = 4;}
        else{
            if(speed <0){
                speed = 0;
            }
            else{
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
                 }
            }
        }
        DT=DT.addMSecs(200);
        tau=tau.addMSecs(200);
        NevozMotion();
        add_v(&omega_KA,omega_ka_nv,omega_upr);
        omega_KA[0] = omega_KA[0]*M_PI/180;
        omega_KA[1] = omega_KA[1]*M_PI/180;
        omega_KA[2] = omega_KA[2]*M_PI/180;

        intgr_qt(&FrJ2000toKA,FrJ2000toKA,omega_KA,0.2);

        omega_KA[0] = omega_KA[0]*180/M_PI;
        omega_KA[1] = omega_KA[1]*180/M_PI;
        omega_KA[2] = omega_KA[2]*180/M_PI;

        project_v(&omega_pr,omega_KA,FrKAtoPr);
        mul_m1v(&Ik,FrPrtoIK,omega_pr);

        emit send_nv(r0_j2000[0]/1000.0,r0_j2000[1]/1000.0,r0_j2000[2]/1000.0,
                omega_KA[0],omega_KA[1],omega_KA[2],
                tau);
       if(j%5 == 0){
        emit send_graphIK(Ik[0],Ik[1],Ik[2],Ik[3],j/5);
        emit send_graph_KA(omega_KA[0],omega_KA[1],omega_KA[2],j/5);
        emit send_graph_PR(omega_pr[0],omega_pr[1],omega_pr[2],j/5);

       /* QTextStream ts(&file);

        ts << r0_geod[0]*180/M_PI  <<";"<< r0_geod[1]*180/M_PI <<";"<< r0_geod[2]/1000 <<";"<<  tau.toString("hh:mm:ss") <<"\n";*/
       }
        j++;
        emit send_pr(omega_pr[0],omega_pr[1],omega_pr[2]);
        emit send_ik(Ik[0],Ik[1],Ik[2],Ik[3]);
        emit send_geod(r0_geod[0]*180/M_PI,r0_geod[1]*180/M_PI,r0_geod[2]/1000);

        traj.setCurrentPos(pos);

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
    j2000::IS_GS(DT.date().year(),DT.date().month(),DT.date().day(),DT.time().hour(),DT.time().minute(),DT.time().second(),&FrJ2000toWGS);              // Задание матрицы перевода из системы j2000 в WGS84
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

    traj.currentPosition.posK = 0;
    traj.currentPosition.posT = 0;
    traj.currentPosition.velK = 0;
    traj.currentPosition.velT = 0;

    pos.posK = 0;
    pos.posT = 0;
    pos.velK = 0;
    pos.velT = 0;
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

void Count::InitTest(double b, double l, int mode){\
    vectord kadr_WGS84;
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
    SightingPoint(dir_KA);
   // traj.createTrajectory(&pos,settings);
}

/*
 * SightingPoint - функция расчета углов до точки съемки
 * Входные переменные:
 *      dir_KA - Вектор от точки съемки до КА в с.к. КА
 * Выходные переменные:
 *      pos -
*/
void Count::SightingPoint(vectord dir_KA){
    vectord Tel_wgs84, Tel_KA_WGS,Tel_KA, Tel_KA_J2000;
      matrixd FrWGStoJ2000;
    // Задание матрицы перевода из гринвичской с.к. в J2000
    inverse_m(&FrWGStoJ2000,FrJ2000toWGS);
    // Вычисление точки пересечения линии визирования
    // с земной поверхности в геодезических координатах
    traj.currentDirection(settings,pos.posK,pos.posT,&mode_desc,r0_wgs84,v0_wgs84);
    // Перевод полученной точки из геодезических координат в гринвичские
    GeoToWGS84(&Tel_wgs84,mode_desc.b,mode_desc.l,mode_desc.h);
    // Нахождение вектора от точки визирования до КА в гринвичской КА
    sub_v(&Tel_KA_WGS,Tel_wgs84,r0_wgs84);
    // Перевод полученного вектора из гринвичской с.к. в J2000
    mul_mv(&Tel_KA_J2000,FrWGStoJ2000,Tel_KA_WGS);
    // И последующий его перевод в с.к. КА
    project_v(&Tel_KA,Tel_KA_J2000,FrJ2000toKA);
    // Определение углов крена и тангажа для перехода к точке
    traj.directionToAngles(&pos,dir_KA,Tel_KA,1);
   // emit send_geod_point(pos.posK,pos.posT,pos.velK);
}
