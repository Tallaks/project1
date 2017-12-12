#include "settings.h"

Settings::Settings()
{
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
