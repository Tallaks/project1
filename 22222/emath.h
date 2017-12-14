#ifndef EMATH_H
#define EMATH_H

/*****************************************************************************
 Система : Общие математические функции
 Автор   :
******************************************************************************/
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<mem.h>

/* Включение стандартного math.h */
#define M_PI 3.14159265358979323846

extern double Mu;
extern double omegaEarth;
/* Описание базовых типов данных */
typedef double vectord[3];
typedef double quaterniond[4];
typedef double matrixd[3][3];
typedef double matrixd1[4][3];
typedef double RungKutt[6]; //Шестимерный вектор, используемый для метода Рунге-Кутты

#define M_TO_RAD M_PI/180.0
#define M_TO_GRD 180.0/M_PI

#define TO_RAD(x) ((x)*M_TO_RAD)
#define TO_GRD(x) ((x)*M_TO_GRD)

/*--------------------------------------------------------------------------*/
/* Результатом функции является первый аргумент.                            */
/* Например: sub_v(&a,b,c); эквивалентно a=b-c                              */
/*--------------------------------------------------------------------------*/
/*--- Операции над векторами                                             ---*/
/*--------------------------------------------------------------------------*/
/* set_v        Присвоение                  */
/* copy_v       Перезапись                  */
/* add_v        Сложение                    */
/* sub_v        Вычитание                   */
/* mul_vf       Умножение на число          */
/* dot_v        Умножение скалярное         */
/* cross_v      Умножение векторное         */
/* abs_v        Норма                       */
/* norm_v       Нормировка                  */
/* angle_v      Угол между векторами        */
/* quaterniond_v Преобразование в кватернион */
/*------------------------------------------*/
extern vectord ORT_X;
extern vectord ORT_Y;
extern vectord ORT_Z;
extern vectord ORT_mY;
extern vectord ORT_mZ;

void set_v( vectord *a , double x , double y , double z );
void copy_v( vectord *a , vectord b );
void add_v( vectord *a , vectord b , vectord c );
void sub_v( vectord *a , vectord b , vectord c );
void mul_vf( vectord *a , vectord b , double f );
double dot_v( vectord b , vectord c );
void cross_v( vectord *c , vectord a , vectord b );
double abs_v( vectord a );
void norm_v( vectord *a , vectord b );
double angle_v( vectord a , vectord b );
void quaterniond_v( quaterniond *q , vectord a );
/*--------------------------------------------------------------------------*/
/*--- Операции над матрицами                                             ---*/
/*--------------------------------------------------------------------------*/
/* set_mi       Единичная матрица           */
/* copy_m       Перезапись                  */
/* add_m        Сложение                    */
/* sub_m        Вычитание                   */
/* mul_mf       Умножение на число          */
/* mul_mv       Умножение на вектор         */
/* mul_m        Умножение                   */
/* inverse_m    Обратная матрица            */
/* transp_m     Транспонирование            */
/* matrixd_q    Получение матрицы поворота  */
/*------------------------------------------*/
void set_mi( matrixd *m );
void copy_m( matrixd *a , matrixd b );
void add_m( matrixd *a , matrixd b , matrixd c );
void sub_m( matrixd *a , matrixd b , matrixd c );
void mul_mf( matrixd *a , matrixd b , double f );
void mul_mv( vectord *a , matrixd m , vectord b );
void mul_m( matrixd *a , matrixd b , matrixd c );
void inverse_m( matrixd *a , matrixd b );
void transp_m( matrixd *a , matrixd b );
void matrixd_q( matrixd *m , quaterniond q );
/*--------------------------------------------------------------------------*/
/*--- Операции над кватернионами                                         ---*/
/*--------------------------------------------------------------------------*/
/* set_q            Присвоение                          */
/* set_qi           Единичный кватернион                */
/* copy_q           Перезапись                          */
/* mul_qf           Умножение на число                  */
/* mul_q            Умножение                           */
/* mul_ql           Умножение на сопряжённый слева      */
/* mul_qr           Умножение на сопряжённый справа     */
/* conj_q           Сопряжённый кватернион              */
/* abs_q            Норма                               */
/* norm_q           Нормировка                          */
/* project_v        Перепроектирование вектора          */
/* project_q        Перепроектирование кватерниона      */
/* quaterniond_m    Получение кватерниона из матрицы    */
/* vectord_q        Преобразование в вектор             */
/* intgr_q,intgr_qt Интегрирование кватерниона          */
/*------------------------------------------------------*/
void set_q( quaterniond *q , double x0 ,double x1 ,double x2 ,double x3 );
void set_qi( quaterniond *q );
void copy_q( quaterniond *a , quaterniond b );
void mul_qf( quaterniond *a , quaterniond b , double f );
void mul_q( quaterniond *a , quaterniond b , quaterniond c );
void mul_ql( quaterniond *a , quaterniond b , quaterniond c );
void mul_qr( quaterniond *a , quaterniond b , quaterniond c );
void conj_q( quaterniond *a , quaterniond b );
double abs_q( quaterniond a );
void norm_q( quaterniond *a , quaterniond b );
void project_v( vectord *a , vectord b , quaterniond q );
void project_q( quaterniond *a , quaterniond b , quaterniond q );
void quaterniond_m( quaterniond *q , matrixd m );
void vectord_q( vectord *v , quaterniond q);
void intgr_q( quaterniond *q, quaterniond a, vectord v);
void intgr_qt(quaterniond *L1, quaterniond L0, vectord w, double t);
/*--------------------------------------------------------------------------*/
/*--- Тригонометрия и прочие функции                                     ---*/
/*--------------------------------------------------------------------------*/
/* norm_a   Нормализовать угол (+/-180)                   */
/* to_rad   Из градусов в радианы                         */
/* to_grad  Из радианов в градусы                         */
/* sqrt2    sqrt( a*a+b*b );                              */
/* sqrt3    sqrt( a*a+b*b+c*c );                          */
/*--------------------------------------------------------*/
double norm_a( double a );
double to_rad( double g );
double to_grad( double r );
double sqrt2( double a , double b );
double sqrt3( double a , double b , double c );

void WGS84ToGeo( vectord *a, double X, double Y, double Z);

void F(RungKutt *f, RungKutt x);
void sum_rk(RungKutt *c,RungKutt a,RungKutt b);
void mul_rk(RungKutt *c,RungKutt a,double b);
void DiffRungKutt(vectord *r, vectord *v, vectord R0, vectord V0, double t);
void GeoToWGS84( vectord *a, double B, double L, double H);
void Povorot0(quaterniond *L, vectord rj2000, vectord vj2000);

void mul_m1v(quaterniond *IK, matrixd1 m, vectord b);
#endif // EMATH_H
