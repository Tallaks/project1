#include "j2000.h"

j2000::j2000()
{

}

void j2000::IS_GS(int    NG,          /* I: year */
           int    NM,          /* I: month */
           int    N,           /* I: day */
           int JT,          /* I: hour */
           int JM,          /* I: minute */
           int JC,          /* I: second */
           matrixd *M_SNP) /* O: transition matrix J2000 - WGS-84 */
{
 double SI;         /* true sidereal time */

 /* расчет юлианской даты */
 double DM;      /* DM-всемирное время UT1 рассматриваемой даты, выраженное в долях суток */
 double L;
 double JD;      /* JD-целая часть юлианской даты */
 double J;
 int KK;
 double DJD;     /* DJD-дробная часть юлианской даты в долях суток */
 double D;       /* (D) - интервал времени от эпохи J2000.0  до эпохи T в средних солнечных сутках */

 /* SCP - расчет среднего звездного времени */
 double TAY;     /* интервал времени от эпохи J2000 в юлианских столетиях */
 double KSC;
 double SC;      /* SC-гринвичское среднее звездное время, выраженное в радианах */

 /* Расчет нутации */
 double TAY2,TAY3;
 double EPCO;    /* EPC0 - средний наклон эклиптики к экватору, выраженный в радианах */
 double FL1;     /* средняя аномалия Солнца */
 double F;       /* средний аргумент широты Луны */
 double D1;      /* разность средних долгот Луны и Солнца */
 double OM;      /* средняя долгота восходящего узла орбиты Луны на эклиптике */
 double SOM,COM; /* sin и cos OM */
 double HPSI;    /* нутация в долготе */
 double HEPC;    /* нутация в наклоне */
 double EPC;     /* истинный наклон эклиптики к экватору */

 /* Расчет звездного времени */
 double HALFA;
 matrixd M_S;   /* матрица учета звездного времени */

 double J_A,Z_A,Q_A; /* прецессионные параметры */
 matrixd M_P;   /* матрица прецессии */

 double CE,SE,SEO,CEO,SHPCI,CHPCI;
 matrixd M_N;   /* матрица нутации */

 matrixd M_NP;

 /* расчет юлианской даты */
 DM=JT/24.+JM/1440.+JC/86400.;

 if(NM==2) N+=31;
 if(NM==3) N+=59;
 if(NM==4) N+=90;
 if(NM==5) N+=120;
 if(NM==6) N+=151;
 if(NM==7) N+=181;
 if(NM==8) N+=212;
 if(NM==9) N+=243;
 if(NM==10) N+=273;
 if(NM==11) N+=304;
 if(NM==12) N+=334;

 L=NG-1900.;
 KK=floor(L/4.); /* Округление до меньшего целого */
 J=KK*4.;
 if((NM>2)&&(J==L)&&(L!=0.))
  N+=1;
 J=floor((L-1.)/4.);
 JD=N+J+L*365.+2415019.;
 if(DM<0.5)
  {
   DJD=DM+0.5;
  }
 else
  {
   DJD=DM-0.5;
   JD+=1;
  }
 D=JD+DJD-2451545.0;

 /* SCP - расчет среднего звездного времени */

 TAY=D/36525.;
 SC=1.7533685592+0.0172027918051*D+6.28318530718*DM+6.777071394*1e-6*TAY*TAY-
     4.50876723*1e-10*TAY*TAY*TAY;
 KSC=SC/6.28318530718;
 SC=(SC-floor(KSC)*6.28318530718);

 /* Расчет нутации */

 TAY2=TAY*TAY;
 TAY3=TAY2*TAY;

 EPCO=0.4090928042-0.2269655*1e-3*TAY-0.29*1e-8*TAY2+0.88*1e-8*TAY3;

 FL1=6.24003594+628.30195602*TAY-2.7974*1e-6*TAY2-5.82*1.e-8*TAY3;

 F=1.62790193+8433.46615831*TAY-6.42717*1e-5*TAY2+5.33*1e-8*TAY3;

 D1=5.19846951+7771.37714617*TAY-3.34085*1e-5*TAY2+9.21*1e-8*TAY3;

 OM=2.182438624-33.757045936*TAY+3.61429*1e-5*TAY2+3.88*1e-8*TAY3;

 SOM=sin(OM);
 COM=cos(OM);
 HPSI=-0.83386*1e-4*SOM+1.9994*1e-6*2*COM*SOM-0.63932*1e-5*sin(2.*(F-D1+OM))+0.6913*1e-6*sin(FL1)-0.11024*1e-5*sin(2.*(F+OM));
 HEPC=0.44615*1e-4*COM+0.27809*1e-5*cos(2.*(F-D1+OM))+0.474*1e-6*cos(2.*(F+OM));
 EPC=EPCO+HEPC;

 /* Матрица нутации */
 SE=sin(EPC);
 CE=cos(EPC);
 SEO=sin(EPCO);
 CEO=cos(EPCO);
 SHPCI=sin(HPSI);

 CHPCI=cos(HPSI);

 M_N[0][0]=CHPCI;
 M_N[0][1]=-SHPCI*CEO;
 M_N[0][2]=-SHPCI*SEO;
 M_N[1][0]=SHPCI*CE;
 M_N[1][1]=CHPCI*CE*CEO+SE*SEO;
 M_N[1][2]=CHPCI*CE*SEO-SE*CEO;
 M_N[2][0]=SHPCI*SE;
 M_N[2][1]=CHPCI*SE*CEO-CE*SEO;
 M_N[2][2]=CHPCI*SE*SEO+CE*CEO;

 /* Расчет истинного звездного времени */
 HALFA=HPSI*cos(EPC);
 SI=SC+HALFA;

 /* Расчет матрицы суточного вращения Земли */
 M_S[0][0]=cos(SI);
 M_S[0][1]=sin(SI);
 M_S[0][2]=0.;
 M_S[1][0]=-sin(SI);
 M_S[1][1]=cos(SI);
 M_S[1][2]=0.;
 M_S[2][0]=0.;
 M_S[2][1]=0.;
 M_S[2][2]=1.;

 /* Расчет матрицы прецессии */
 J_A=0.0111808609*TAY+0.146356*1e-5*TAY2+0.872*1e-7*TAY3;
 Z_A=0.0111808609*TAY+0.53072*1e-5*TAY2+0.883*10e-7*TAY3;
 Q_A=0.97171735*1e-2*TAY-0.20685*1e-5*TAY2-0.2028*1e-6*TAY3;

 M_P[0][0]=cos(J_A)*cos(Z_A)*cos(Q_A)-sin(J_A)*sin(Z_A);
 M_P[0][1]=-sin(J_A)*cos(Z_A)*cos(Q_A)-cos(J_A)*sin(Z_A);
 M_P[0][2]=-cos(Z_A)*sin(Q_A);
 M_P[1][0]=cos(J_A)*sin(Z_A)*cos(Q_A)+sin(J_A)*cos(Z_A);
 M_P[1][1]=-sin(J_A)*sin(Z_A)*cos(Q_A)+cos(J_A)*cos(Z_A);
 M_P[1][2]=-sin(Z_A)*sin(Q_A);
 M_P[2][0]=cos(J_A)*sin(Q_A);
 M_P[2][1]=-sin(J_A)*sin(Q_A);
 M_P[2][2]=cos(Q_A);


// Перемножение матрицы нутации и матрицы прецессии
mul_m(&M_NP,M_N,M_P);
// Перемножение матрицы суточного вращения Земли и произведения матриц нутации и прецессии
mul_m(M_SNP,M_S,M_NP);
}
