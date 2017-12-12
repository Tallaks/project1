#ifndef TRAJ_H
#define TRAJ_H


class Traj
{
public:
    Traj();
    /*bool calcSpline(double effVel, double acc, &currPos,  const DpnMotionDesc &frsPos, unsigned char angFlag)
    *        {
     *           double ac = acc;
      *          double v = effVel;
                double a0 = currPos.posA,
                       at = frsPos.posA,
                       va0 = currPos.velA,
                       vat = frsPos.velA;
                double b0 = currPos.posB,
                       bt = frsPos.posB,
                       vb0 = currPos.velB,
                       vbt = frsPos.velB;
                /* Проверяем переданные значения *//*
     *           if(v<=0 || ac<=0 || (fabs(at-a0)<0.001 && fabs(bt-b0)<0.001))
                    return false;

                 double T1, T2, T3, Ta, Tb, T;
                 /* Вычисляем времена переброса по каналам Альфа и Бета */
              //   if(angFlag==0 || angFlag==1)
             /*    {
                    T1 = fabs(at - a0) / v;
                    T2 = fabs(va0 - (at - a0)/T1)/ac;
                    T3 = fabs((at - a0)/T1 - vat)/ac;
                    Ta = T1 + T2 + T3;
                 }
                 if(angFlag==0 || angFlag==2)
                 {
                    T1 = fabs(bt - b0) / v;
                    T2 = fabs(vb0 - (bt - b0)/T1)/ac;
                    T3 = fabs((bt - b0)/T1 - vbt)/ac;
                    Tb = T1 + T2 + T3;
                 }
                 if(angFlag==0)
                    T = Ta>Tb?Ta:Tb;
                 if(angFlag==0)
                    _relocationTime = T;
                 else if(angFlag==1)
                    _relocationTime = Ta;
                 else if(angFlag==2)
                    _relocationTime = Tb;
                 else
                    return false;

                 T = _relocationTime;
                 double a, b;
                 /* Рассчитываем коэффициенты сплайнов для приводов Альфа и Бета */
              /*   a = at - va0*T - a0;
                 b = vat - va0;
                 _aSpline[0] = (T*b - 2.0*a)/(pow(T,3));
                 _aSpline[1] = (3.0*a - T*b)/(pow(T,2));
                 _aSpline[2] = va0;
                 _aSpline[3] = a0;

                 a = bt - vb0*T - b0;
                 b = vbt - vb0;
                 _bSpline[0] = (T*b - 2.0*a)/(pow(T,3));
                 _bSpline[1] = (3.0*a - T*b)/(pow(T,2));
                 _bSpline[2] = vb0;
                 _bSpline[3] = b0;

                 return true;
            }*/
};

#endif // TRAJ_H
