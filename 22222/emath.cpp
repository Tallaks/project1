#include "emath.h"

double Mu = 398602000000000.0;
double p0 = 398600441800000.0;
double p1 = -66063460006.0;
double p2 = 2502300000000000000000.0;
double omegaEarth = 7.29211e-5;
/*---------------------------------------------------------------------------*/
/*--- Операции над векторами                                              ---*/
/*---------------------------------------------------------------------------*/
vectord ORT_X = {1.0, 0.0, 0.0};
vectord ORT_Y = {0.0, 1.0, 0.0};
vectord ORT_Z = {0.0, 0.0, 1.0};
vectord ORT_mY = {0.0, -1.0, 0.0};
vectord ORT_mZ = {0.0, 0.0, -1.0};

/* Присвоение */
void set_v( vectord *a , double x , double y , double z ) {
    (*a)[0]=x;
    (*a)[1]=y;
    (*a)[2]=z;
}
/* Перезапись */
void copy_v( vectord *a , vectord b ) {
    (*a)[0]=b[0];
    (*a)[1]=b[1];
    (*a)[2]=b[2];
}
/* Сложение */
void add_v( vectord *a , vectord b , vectord c ) {
    (*a)[0]=b[0]+c[0];
    (*a)[1]=b[1]+c[1];
    (*a)[2]=b[2]+c[2];
}
/* Вычитание */
void sub_v( vectord *a , vectord b , vectord c ) {
    (*a)[0]=b[0]-c[0];
    (*a)[1]=b[1]-c[1];
    (*a)[2]=b[2]-c[2];
}
/* Умножение на число */
void mul_vf( vectord *a , vectord b , double f ) {
    (*a)[0]=b[0]*f;
    (*a)[1]=b[1]*f;
    (*a)[2]=b[2]*f;
}
/* Умножение скалярное */
double dot_v( vectord b , vectord c ) {
double r;
    r = b[0]*c[0] + b[1]*c[1] + b[2]*c[2];
    return (r);
}
/* Умножение векторное */
void cross_v( vectord *a , vectord b , vectord c ) {
vectord A;
    A[0] = b[1] * c[2] - b[2] * c[1];
    A[1] = b[2] * c[0] - b[0] * c[2];
    A[2] = b[0] * c[1] - b[1] * c[0];
    copy_v(a,A);
}
/* Норма */
double abs_v( vectord a ) {
double ABS_V;
    ABS_V=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    return (ABS_V);
}
/* Нормировка */
void norm_v( vectord *a , vectord b ) {
double ABS_V;
    ABS_V=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
    if(ABS_V)
    {
        (*a)[0]=b[0]/ABS_V;
        (*a)[1]=b[1]/ABS_V;
        (*a)[2]=b[2]/ABS_V;
    }
}
/* Угол между векторами */
double angle_v( vectord a , vectord b ) {
double abs_a,abs_b;
    abs_a=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    abs_b=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
    return (acos(dot_v(a,b)/abs_a/abs_b));
}
/* Преобразование в кватернион */
void quaterniond_v( quaterniond *q , vectord a ) {
    (*q)[0]=0;
    (*q)[1]=a[0];
    (*q)[2]=a[1];
    (*q)[3]=a[2];
}
/*---------------------------------------------------------------------------*/
/*--- Операции над матрицами                                              ---*/
/*---------------------------------------------------------------------------*/
/* Единичная матрица */
void set_mi( matrixd *m ) {
    (*m)[0][0]=1.0;  (*m)[0][1]=0.0;  (*m)[0][2]=0.0;
    (*m)[1][0]=0.0;  (*m)[1][1]=1.0;  (*m)[1][2]=0.0;
    (*m)[2][0]=0.0;  (*m)[2][1]=0.0;  (*m)[2][2]=1.0;
}
/* Перезапись */
void copy_m( matrixd *a , matrixd b ) {
    memcpy((void*)(*a),(void*)b,sizeof(matrixd));
}
/* Сложение */
void add_m( matrixd *a , matrixd b , matrixd c ) {
    (*a)[0][0]=b[0][0]+c[0][0]; (*a)[0][1]=b[0][1]+c[0][1]; (*a)[0][2]=b[0][2]+c[0][2];
    (*a)[1][0]=b[1][0]+c[1][0]; (*a)[1][1]=b[1][1]+c[1][1]; (*a)[1][2]=b[1][2]+c[1][2];
    (*a)[2][0]=b[2][0]+c[2][0]; (*a)[2][1]=b[2][1]+c[2][1]; (*a)[2][2]=b[2][2]+c[2][2];
}
/* Вычитание */
void sub_m( matrixd *a , matrixd b , matrixd c ) {
    (*a)[0][0]=b[0][0]-c[0][0]; (*a)[0][1]=b[0][1]-c[0][1]; (*a)[0][2]=b[0][2]-c[0][2];
    (*a)[1][0]=b[1][0]-c[1][0]; (*a)[1][1]=b[1][1]-c[1][1]; (*a)[1][2]=b[1][2]-c[1][2];
    (*a)[2][0]=b[2][0]-c[2][0]; (*a)[2][1]=b[2][1]-c[2][1]; (*a)[2][2]=b[2][2]-c[2][2];
}
/* Умножение на число */
void mul_mf( matrixd *a , matrixd b , double f ) {
    (*a)[0][0]=b[0][0]*f; (*a)[0][1]=b[0][1]*f; (*a)[0][2]=b[0][2]*f;
    (*a)[1][0]=b[1][0]*f; (*a)[1][1]=b[1][1]*f; (*a)[1][2]=b[1][2]*f;
    (*a)[2][0]=b[2][0]*f; (*a)[2][1]=b[2][1]*f; (*a)[2][2]=b[2][2]*f;
}
/* Умножение на вектор */
void mul_mv( vectord *a , matrixd m , vectord b ) {
vectord A;
    A[0]=m[0][0]*b[0]+m[0][1]*b[1]+m[0][2]*b[2];
    A[1]=m[1][0]*b[0]+m[1][1]*b[1]+m[1][2]*b[2];
    A[2]=m[2][0]*b[0]+m[2][1]*b[1]+m[2][2]*b[2];
    copy_v(a,A);
}
/* Умножение */
void mul_m( matrixd *a , matrixd b , matrixd c ) {
matrixd A;
    A[0][0]=b[0][0]*c[0][0]+b[0][1]*c[1][0]+b[0][2]*c[2][0];
    A[1][0]=b[1][0]*c[0][0]+b[1][1]*c[1][0]+b[1][2]*c[2][0];
    A[2][0]=b[2][0]*c[0][0]+b[2][1]*c[1][0]+b[2][2]*c[2][0];
    A[0][1]=b[0][0]*c[0][1]+b[0][1]*c[1][1]+b[0][2]*c[2][1];
    A[1][1]=b[1][0]*c[0][1]+b[1][1]*c[1][1]+b[1][2]*c[2][1];
    A[2][1]=b[2][0]*c[0][1]+b[2][1]*c[1][1]+b[2][2]*c[2][1];
    A[0][2]=b[0][0]*c[0][2]+b[0][1]*c[1][2]+b[0][2]*c[2][2];
    A[1][2]=b[1][0]*c[0][2]+b[1][1]*c[1][2]+b[1][2]*c[2][2];
    A[2][2]=b[2][0]*c[0][2]+b[2][1]*c[1][2]+b[2][2]*c[2][2];
    copy_m(a,A);
}
/* Обратная матрица */
void inverse_m( matrixd *a , matrixd b ) {
matrixd M;
double d;


    d=+b[0][0]*b[1][1]*b[2][2]-b[0][0]*b[2][1]*b[1][2]
      +b[0][1]*b[1][2]*b[2][0]-b[0][1]*b[2][2]*b[1][0]
      +b[0][2]*b[1][0]*b[2][1]-b[0][2]*b[2][0]*b[1][1];

    M[0][0]= (b[1][1]*b[2][2]-b[2][1]*b[1][2])/d;
    M[1][0]=-(b[1][0]*b[2][2]-b[2][0]*b[1][2])/d;
    M[2][0]= (b[1][0]*b[2][1]-b[2][0]*b[1][1])/d;
    M[0][1]=-(b[0][1]*b[2][2]-b[2][1]*b[0][2])/d;
    M[1][1]= (b[0][0]*b[2][2]-b[2][0]*b[0][2])/d;
    M[2][1]=-(b[0][0]*b[2][1]-b[2][0]*b[0][1])/d;
    M[0][2]= (b[0][1]*b[1][2]-b[1][1]*b[0][2])/d;
    M[1][2]=-(b[0][0]*b[1][2]-b[1][0]*b[0][2])/d;
    M[2][2]= (b[0][0]*b[1][1]-b[1][0]*b[0][1])/d;

    copy_m(a,M);
}
/* Транспонирование */
void transp_m( matrixd *a , matrixd b ) {
matrixd M;
    M[0][0]=b[0][0]; M[0][1]=b[1][0]; M[0][2]=b[2][0];
    M[1][0]=b[0][1]; M[1][1]=b[1][1]; M[1][2]=b[2][1];
    M[2][0]=b[0][2]; M[2][1]=b[1][2]; M[2][2]=b[2][2];
    copy_m(a,M);
}
/* Получение матрицы поворота */
void matrixd_q( matrixd *m , quaterniond q ) {
double P[4][4];
int    i,j;

    for( i=0; i<=3; i++ ) for( j=0; j<=3; j++ ) P[i][j] = q[i] * q[j];

    (*m)[0][0] = P[0][0] + P[1][1] - P[2][2] - P[3][3];
    (*m)[0][1] = 2.0 * ( P[1][2] + P[0][3] );
    (*m)[0][2] = 2.0 * ( P[1][3] - P[0][2] );
    (*m)[1][0] = 2.0 * ( P[1][2] - P[0][3] );
    (*m)[1][1] = P[0][0] + P[2][2] - P[1][1] - P[3][3];
    (*m)[1][2] = 2.0 * ( P[2][3] + P[0][1] );
    (*m)[2][0] = 2.0 * ( P[1][3] + P[0][2] );
    (*m)[2][1] = 2.0 * ( P[2][3] - P[0][1] );
    (*m)[2][2] = P[0][0] + P[3][3] - P[1][1] - P[2][2];
}
/*---------------------------------------------------------------------------*/
/*--- Операции над кватернионами                                          ---*/
/*---------------------------------------------------------------------------*/
/* Присвоение */
void set_q( quaterniond *q , double x0 ,double x1 ,double x2 ,double x3 ) {
    (*q)[0]=x0;
    (*q)[1]=x1;
    (*q)[2]=x2;
    (*q)[3]=x3;
}
/* Единичный кватернион */
void set_qi( quaterniond *q ) {
    (*q)[0]=1.0;
    (*q)[1]=0.0;
    (*q)[2]=0.0;
    (*q)[3]=0.0;
}
/* Перезапись */
void copy_q( quaterniond *a , quaterniond b ) {
    (*a)[0]=b[0];
    (*a)[1]=b[1];
    (*a)[2]=b[2];
    (*a)[3]=b[3];
}
/* Умножение на число */
void mul_qf( quaterniond *a , quaterniond b , double f ) {
    (*a)[0]=b[0]*f;
    (*a)[1]=b[1]*f;
    (*a)[2]=b[2]*f;
    (*a)[3]=b[3]*f;
}
/* Умножение : a=b*c*/
void mul_q( quaterniond *a , quaterniond b , quaterniond c ) {
quaterniond M;
    M[0] = b[0]*c[0] - b[1]*c[1] - b[2]*c[2] - b[3]*c[3];
    M[1] = b[0]*c[1] + b[1]*c[0] + b[2]*c[3] - b[3]*c[2];
    M[2] = b[0]*c[2] + b[2]*c[0] + b[3]*c[1] - b[1]*c[3];
    M[3] = b[0]*c[3] + b[3]*c[0] + b[1]*c[2] - b[2]*c[1];
    copy_q(a,M);
}
/* Умножение на сопряжённый слева : a=b^*c */
void mul_ql( quaterniond *a , quaterniond b , quaterniond c ) {
quaterniond M;
    M[0] = b[0]*c[0] + b[1]*c[1] + b[2]*c[2] + b[3]*c[3];
    M[1] = b[0]*c[1] - b[1]*c[0] - b[2]*c[3] + b[3]*c[2];
    M[2] = b[0]*c[2] - b[2]*c[0] - b[3]*c[1] + b[1]*c[3];
    M[3] = b[0]*c[3] - b[3]*c[0] - b[1]*c[2] + b[2]*c[1];
    copy_q(a,M);
}
/* Умножение на сопряжённый справа : a=b*c^ */
void mul_qr( quaterniond *a , quaterniond b , quaterniond c ) {
quaterniond M;
    M[0] =  b[0]*c[0] + b[1]*c[1] + b[2]*c[2] + b[3]*c[3];
    M[1] = -b[0]*c[1] + b[1]*c[0] - b[2]*c[3] + b[3]*c[2];
    M[2] = -b[0]*c[2] + b[2]*c[0] - b[3]*c[1] + b[1]*c[3];
    M[3] = -b[0]*c[3] + b[3]*c[0] - b[1]*c[2] + b[2]*c[1];
    copy_q(a,M);
}
/* Сопряжённый кватернион */
void conj_q( quaterniond *a , quaterniond b ) {
    (*a)[0]= b[0];
    (*a)[1]=-b[1];
    (*a)[2]=-b[2];
    (*a)[3]=-b[3];
}
/* Норма */
double abs_q( quaterniond a ) {
    return (sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3]));
}
/* Нормировка */
void norm_q( quaterniond *a , quaterniond b ) {
double ABS_Q;
    ABS_Q=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]+b[3]*b[3]);
    (*a)[0]=b[0]/ABS_Q;
    (*a)[1]=b[1]/ABS_Q;
    (*a)[2]=b[2]/ABS_Q;
    (*a)[3]=b[3]/ABS_Q;
}
/* Перепроектирование вектора */
void project_v( vectord *a , vectord b , quaterniond q ) {
    quaterniond P;

    P[0] = -b[0]*q[1] - b[1]*q[2] - b[2]*q[3];             /* P = B * Q*/
    P[1] =  b[0]*q[0] + b[1]*q[3] - b[2]*q[2];
    P[2] =  b[1]*q[0] + b[2]*q[1] - b[0]*q[3];
    P[3] =  b[2]*q[0] + b[0]*q[2] - b[1]*q[1];

    (*a)[0] =  q[0]*P[1] - q[1]*P[0] - q[2]*P[3] + q[3]*P[2]; /* q = <Q>* P */
    (*a)[1] =  q[0]*P[2] - q[2]*P[0] - q[3]*P[1] + q[1]*P[3];
    (*a)[2] =  q[0]*P[3] - q[3]*P[0] - q[1]*P[2] + q[2]*P[1];

}
/* Перепроектирование кватерниона */
void project_q( quaterniond *a , quaterniond b , quaterniond q ) { /*  a=q^ * b * q  */
quaterniond P;

    mul_ql(&P,q,b);               /*  P=q^ * b  */
    mul_q(a,P,q);                 /*  a=P  * q  */
}
/* Получение кватерниона из матрицы */
void quaterniond_m( quaterniond *q , matrixd m ) {
quaterniond r;
double Tr,r_max,r_max2;
int    i,i_max;

    Tr=m[0][0]+m[1][1]+m[2][2];
    r[0]=1.0+Tr;
    r[1]=1.0+2.0*m[0][0]-Tr;
    r[2]=1.0+2.0*m[1][1]-Tr;
    r[3]=1.0+2.0*m[2][2]-Tr;

    i_max=0;
    for (i=1;i<=3;i++) if (r[i]>r[i_max]) i_max = i;
    r_max=sqrt(r[i_max]);
    r_max2=2.0*r_max;

    if (i_max == 0) {
        (*q)[0]=r_max/2;
        (*q)[1]=(m[1][2]-m[2][1])/r_max2;
        (*q)[2]=(m[2][0]-m[0][2])/r_max2;
        (*q)[3]=(m[0][1]-m[1][0])/r_max2;
    };
    if (i_max == 1) {
        (*q)[0]=(m[1][2]-m[2][1])/r_max2;
        (*q)[1]=r_max/2;
        (*q)[2]=(m[1][0]+m[0][1])/r_max2;
        (*q)[3]=(m[0][2]+m[2][0])/r_max2;
    };
    if (i_max == 2) {
        (*q)[0]=(m[2][0]-m[0][2])/r_max2;
        (*q)[1]=(m[1][0]+m[0][1])/r_max2;
        (*q)[2]=r_max/2;
        (*q)[3]=(m[2][1]+m[1][2])/r_max2;
    };
    if (i_max == 3) {
        (*q)[0]=(m[0][1]-m[1][0])/r_max2;
        (*q)[1]=(m[0][2]+m[2][0])/r_max2;
        (*q)[2]=(m[2][1]+m[1][2])/r_max2;
        (*q)[3]=r_max/2;
    };
}
/* Преобразование в вектор */
void vectord_q( vectord *v, quaterniond q) {
    (*v)[0]=q[1];
    (*v)[1]=q[2];
    (*v)[2]=q[3];
}
/* Инегрирование кинематического уравнения */
/* На входе : a - начальный кватернион; v - поворот */
void intgr_qt( quaterniond *q, quaterniond a, vectord w, double t)
{
    quaterniond R,P;
    double SR,SP;

    copy_q(&R,a);
    P[1] = w[0]*t*0.5;
    P[2] = w[1]*t*0.5;
    P[3] = w[2]*t*0.5;

    SP = P[1]*P[1] + P[2]*P[2] + P[3]*P[3];
    SR = R[0]*R[0] + R[1]*R[1] + R[2]*R[2] + R[3]*R[3];

    P[0] = (3.0 - SP - SR)*0.5;
    (*q)[0] = R[0]*P[0] - R[1]*P[1] - R[2]*P[2] - R[3]*P[3];
    (*q)[1] = R[0]*P[1] + R[1]*P[0] + R[2]*P[3] - R[3]*P[2];
    (*q)[2] = R[0]*P[2] + R[2]*P[0] + R[3]*P[1] - R[1]*P[3];
    (*q)[3] = R[0]*P[3] + R[3]*P[0] + R[1]*P[2] - R[2]*P[1];
}
/* Инегрирование кинематического уравнения */
/* На входе : a - начальный кватернион;
              w - угловая скорость
              t - время */
void intgr_qt_Euler_2_norm( quaterniond *L1, quaterniond L0, vectord w, double t)
{
    quaterniond l0,l1,l2;
    vectord teta;
    double teta2,L2;
    mul_vf(&teta,w,t);

    quaterniond_v(&l0,teta);
    mul_q(&l0,L0,l0);
    mul_qf(&l0,l0,0.5);

    teta2 = abs_v(teta)*abs_v(teta);
    mul_qf(&l1,L0,-0.125);
    mul_qf(&l1,L0,teta2);

    L2 = abs_q(L0)*abs_q(L0);
    mul_qf(&l2,L0,0.5*(1-L2*L2));

    (*L1)[0] = L0[0]+ l0[0] + l1[0] + l2[0];
    (*L1)[1] = L0[1]+ l0[1] + l1[1] + l2[1];
    (*L1)[2] = L0[2]+ l0[2] + l1[2]+ l2[2];
    (*L1)[3] = L0[3]+ l0[3] + l1[3] + l2[3];

}

void intgr_qt_meanv3( quaterniond *L1, quaterniond L0, vectord w,vectord prevw, double t)
{
    vectord theta,theta2;
    vectord phi,delta2,dd;
    quaterniond N;

    mul_vf(&theta,w,t);
    mul_vf(&theta2,prevw,t);
    sub_v(&delta2,theta,theta2);
    mul_vf(&delta2,delta2,1/t);
    cross_v(&dd,theta,delta2);
    mul_vf(&dd,dd,1/12.0);

    add_v(&phi,theta,dd);
    mul_vf(&phi,phi,1/2.0);

    N[0] = cos(abs_v(phi));
    N[1] = (phi[0]/abs_v(phi))*sin(abs_v(phi));
    N[2] = (phi[1]/abs_v(phi))*sin(abs_v(phi));
    N[3] = (phi[2]/abs_v(phi))*sin(abs_v(phi));

    mul_q(L1,L0,N);
}

void intgr_qt_Euler_1( quaterniond *L1, quaterniond L0, vectord w, double t)
{
    quaterniond l0;
    vectord teta;
    mul_vf(&teta,w,0.5*t);

    quaterniond_v(&l0,teta);
    mul_q(&l0,L0,l0);

    (*L1)[0] = L0[0]+ l0[0];
    (*L1)[1] = L0[1]+ l0[1];
    (*L1)[2] = L0[2]+ l0[2];
    (*L1)[3] = L0[3]+ l0[3];

}

void intgr_qt_stiltyes( quaterniond *LN, quaterniond LN1, quaterniond LN2, quaterniond LN3, quaterniond LN4, vectord w, double t){
    vectord theta_v;
    quaterniond theta_q,delta_L1,delta_L2;
    quaterniond d0,d1,d2,d3,dd1,dd2,dd;


    mul_vf(&theta_v,w,0.5*t);
    quaterniond_v(&theta_q,theta_v);

    mul_q(&delta_L1,LN1,theta_q);

    d1[0] = (LN1[0]-LN2[0])/t;
    d1[1] = (LN1[1]-LN2[1])/t;
    d1[2] = (LN1[2]-LN2[2])/t;
    d1[3] = (LN1[3]-LN2[3])/t;

    dd1[0] = (LN2[0]-LN3[0])/t;
    dd1[1] = (LN2[1]-LN3[1])/t;
    dd1[2] = (LN2[2]-LN3[2])/t;
    dd1[3] = (LN2[3]-LN3[3])/t;

    d2[0] = (d1[0]-dd1[0])/t;
    d2[1] = (d1[1]-dd1[1])/t;
    d2[2] = (d1[2]-dd1[2])/t;
    d2[3] = (d1[3]-dd1[3])/t;

    dd2[0] = (LN3[0]-LN4[0])/t;
    dd2[1] = (LN3[1]-LN4[1])/t;
    dd2[2] = (LN3[2]-LN4[2])/t;
    dd2[3] = (LN3[3]-LN4[3])/t;

    dd[0] = (dd1[0]-dd2[0])/t;
    dd[1] = (dd1[1]-dd2[1])/t;
    dd[2] = (dd1[2]-dd2[2])/t;
    dd[3] = (dd1[3]-dd2[3])/t;

    d3[0] = (d2[0]-dd[0])/t;
    d3[1] = (d2[1]-dd[1])/t;
    d3[2] = (d2[2]-dd[2])/t;
    d3[3] = (d2[3]-dd[3])/t;

    d0[0] = d1[0] + d2[0] + d3[0];
    d0[1] = d1[1] + d2[1] + d3[1];
    d0[2] = d1[2] + d2[2] + d3[2];
    d0[3] = d1[3] + d2[3] + d3[3];

    mul_qf(&d0,d0,0.5);

    mul_q(&delta_L2,d0,theta_q);

    (*LN)[0] = delta_L1[0]+delta_L2[0];
    (*LN)[1] = delta_L1[1]+delta_L2[1];
    (*LN)[2] = delta_L1[2]+delta_L2[2];
    (*LN)[3] = delta_L1[3]+delta_L2[3];
}
/*---------------------------------------------------------------------------*/
/*--- Тригонометрия и прочие функции                                      ---*/
/*---------------------------------------------------------------------------*/
/* Нормализовать угол */
double norm_a( double a ) {
return a;
}
/* Из градусов в радианы */
double to_rad( double g ) {
    return (g*M_PI/180.0);
}
/* Из радианов в градусы */
double to_grad( double r ) {
    return (r*180.0/M_PI);
}
/* sqrt( a*a+b*b ) */
double sqrt2( double a , double b ) {
    return (sqrt(a*a+b*b));
}
/* sqrt( a*a+b*b+c*c ) */
double sqrt3( double a , double b , double c ) {
    return (sqrt(a*a+b*b+c*c));
}

/* Из геодезических в WGS-84 */
void GeoToWGS84( vectord *a, double B, double L, double H){
    vectord A;
    double N;
    double e = 1/298.257223563;
    N = 6378137.0/sqrt(1 - (2*e-e*e)*sin(to_rad(B))*sin(to_rad(B)));
    A[0] = (N + H)*cos(to_rad(B))*cos(to_rad(L));
    A[1] = (N + H)*cos(to_rad(B))*sin(to_rad(L));
    A[2] = ((1-e)*(1-e)*N + H)*sin(to_rad(B));
    copy_v(a,A);
}

void WGS84ToGeo( vectord *a, double X, double Y, double Z){
    vectord A;
    double ae = 6378137.0;
    double ee = 1/298.257223563;
    double be = ae*(1-ee);
    double e2 = 2*ee-ee*ee;
    double e12 = e2/(1-e2);
    double c = ae/(1-ee);
    double R = sqrt(X*X+Y*Y+Z*Z);
    double t;
    double p,n;
    p = sqrt(X*X + Y*Y);
    if(p == 0){
        A[0] = Z>0 ? 90 : -90;
        A[1] = 0;
        A[2] = fabs(Z) - be;
    }
    else{
        t = Z / p * (1 + e12*be / R);
        A[0] = atan2(Y,X);
        A[1] = atan(t);
        n = c/sqrt(1+e12*pow(cos(A[1]),2));
        if(fabs(t)<=1){
            A[2] = p / cos(A[1]) - n;
        }
        else{
            A[2] = Z / sin(A[1]) - n*(1-e2);
        }
    }
    copy_v(a,A);
}

/*Функция f(x,t) для метода Рунге-Кутты*/
void F(RungKutt *f,RungKutt x){
    double r;
    r = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
    (*f)[0] = x[3];
    (*f)[1] = x[4];
    (*f)[2] = x[5];
    (*f)[3] = -x[0]*(p0/(r*r*r)*(1+p1/(r*r)*(5*(x[2]*x[2])/(r*r)-1)+p2/(r*r*r*r)*(63*(x[2]*x[2]*x[2]*x[2])/(r*r*r*r)-42*(x[2]*x[2])/(r*r)+3)));
    (*f)[4] = -x[1]*(p0/(r*r*r)*(1+p1/(r*r)*(5*(x[2]*x[2])/(r*r)-1)+p2/(r*r*r*r)*(63*(x[2]*x[2]*x[2]*x[2])/(r*r*r*r)-42*(x[2]*x[2])/(r*r)+3)));
    (*f)[5] = -x[2]*(p0/(r*r*r)*(1+p1/(r*r)*(5*(x[2]*x[2])/(r*r)-3)+p2/(r*r*r*r)*(63*(x[2]*x[2]*x[2]*x[2])/(r*r*r*r)-70*(x[2]*x[2])/(r*r)+15)));
}

/*Суммирование векторов, используемых в методе Рунге-Кутты*/
void sum_rk(RungKutt *c,RungKutt a,RungKutt b){
    (*c)[0] = a[0]+b[0];
    (*c)[1] = a[1]+b[1];
    (*c)[2] = a[2]+b[2];
    (*c)[3] = a[3]+b[3];
    (*c)[4] = a[4]+b[4];
    (*c)[5] = a[5]+b[5];
}

/*Уммножение вектора, используемого в методе Рунге-Кутты, на скаляр*/
void mul_rk(RungKutt *c,RungKutt a,double b){
    (*c)[0] = a[0]*b;
    (*c)[1] = a[1]*b;
    (*c)[2] = a[2]*b;
    (*c)[3] = a[3]*b;
    (*c)[4] = a[4]*b;
    (*c)[5] = a[5]*b;
}

/*Решение системы дифференциальных уравнений методом Рунге-Кутты*/
void DiffRungKutt(vectord *r, vectord *v, vectord R0, vectord V0,double t){
    RungKutt x,k1,k2,k3,k4,x0,xx,xxx,s1;
    x0[0] = R0[0]; x0[1] = R0[1]; x0[2] = R0[2];
    x0[3] = V0[0]; x0[4] = V0[1]; x0[5] = V0[2];
    F(&k1,x0); mul_rk(&xxx,k1,t/2.0); sum_rk(&xx,xxx,x0);
    F(&k2,xx); mul_rk(&xxx,k2,t/2.0); sum_rk(&xx,xxx,x0);
    F(&k3,xx); mul_rk(&xxx,k3,t); sum_rk(&xx,xxx,x0);
    F(&k4,xx);
    sum_rk(&xx,k1,k4); mul_rk(&xxx,k2,2.0); sum_rk(&s1,xx,xxx); mul_rk(&xxx,k3,2.0);sum_rk(&xx,s1,xxx);
    mul_rk(&s1,xx,t/6.0);
    sum_rk(&x,x0,s1);
    x[0] = x[0]*cos(omegaEarth*t)+x[1]*sin(omegaEarth*t);
    x[1] = x[1]*cos(omegaEarth*t)-x[0]*sin(omegaEarth*t);
    x[3] = x[3]*cos(omegaEarth*t)+x[4]*sin(omegaEarth*t);
    x[4] = x[4]*cos(omegaEarth*t)-x[3]*sin(omegaEarth*t);
    (*r)[0] = x[0]; (*r)[1] = x[1]; (*r)[2] = x[2];
    (*v)[0] = x[3]; (*v)[1] = x[4]; (*v)[2] = x[5];
}


void Povorot0(quaterniond *L, vectord rj2000, vectord vj2000){
    matrixd A;
    vectord x,y,z,nr,nv,vnov;
    quaterniond l;

    norm_v(&nr,rj2000);

    y[0] = -nr[0];
    y[1] = -nr[1];
    y[2] = -nr[2];


    vnov[0] = -nr[0]*dot_v(y,vj2000);
    vnov[1] = -nr[1]*dot_v(y,vj2000);
    vnov[2] = -nr[2]*dot_v(y,vj2000);

    sub_v(&vnov,vnov,vj2000);
    norm_v(&nv,vnov);

    z[0] = nv[0];
    z[1] = nv[1];
    z[2] = nv[2];

    cross_v(&x,y,z);
    norm_v(&x,x);
    A[0][0] = x[0]; A[0][1] = y[0]; A[0][2] = z[0];
    A[1][0] = x[1]; A[1][1] = y[1]; A[1][2] = z[1];
    A[2][0] = x[2]; A[2][1] = y[2]; A[2][2] = z[2];
    inverse_m(&A,A);
    quaterniond_m(&l,A);
    copy_q(L,l);
}

void mul_m1v(quaterniond *IK, matrixd1 m, vectord b){
    quaterniond A;
        A[0]=m[0][0]*b[0]+m[0][1]*b[1]+m[0][2]*b[2];
        A[1]=m[1][0]*b[0]+m[1][1]*b[1]+m[1][2]*b[2];
        A[2]=m[2][0]*b[0]+m[2][1]*b[1]+m[2][2]*b[2];
        A[3]=m[3][0]*b[0]+m[3][1]*b[1]+m[3][2]*b[2];
        copy_q(IK,A);
}

double Mistake_Kinemtic_Euler_1(vectord w, double t){
    return (abs_v(w)*t)*(abs_v(w)*t)/8;
}

double Mistake_Kinemtic_Euler_2(vectord w, double t){
    return (abs_v(w)*t)*(abs_v(w)*t)*(abs_v(w)*t)*(abs_v(w)*t)/128;
}

double Mistake_Kinemtic_MeanV3(vectord w,vectord prev_w,vectord prev_prev_w,double t){
    vectord delta21,delta22,delta3,delta_rez;
    vectord theta1,theta2,theta3;

    mul_vf(&theta1,w,t);
    mul_vf(&theta2,prev_w,t);
    mul_vf(&theta3,prev_prev_w,t);

    sub_v(&delta21,theta1,theta2);
    mul_vf(&delta21,delta21,1/t);

    sub_v(&delta22,theta2,theta3);
    mul_vf(&delta22,delta22,1/t);

    sub_v(&delta3,delta21,delta22);
    mul_vf(&delta3,delta3,1/t);

    cross_v(&delta_rez,delta21,delta3);
    mul_vf(&delta_rez,delta_rez,(-1/48.0));
    return abs_v(delta_rez);
}

