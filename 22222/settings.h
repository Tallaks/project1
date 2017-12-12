#ifndef SETTINGS_H
#define SETTINGS_H

#include <QDateTime>

#include "emath.h"
#include "j2000.h"

class Settings
{
public:
    Settings();
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
};

#endif // SETTINGS_H
