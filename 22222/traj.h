#ifndef TRAJ_H
#define TRAJ_H

#include "emath.h"

#define TACT_H 0.2
#define VEL_MAX 3.0
#define VEL_MIN -3.0
#define MAX_ANGLE 180.0
#define MAX_FORESEEN_TIME10 180.0
#define DPN_Q_1ST_STEP 0.2
#define DPN_VEC_1ST_STEP 0.2

namespace Trajectory{

    enum DynaErrors{dynErrNumTraj,dynErrBadAngle,dynErrBadTime,dynErrOk};

    enum DynaModes{dynModeConst,dynModeEarth,dynModeStars,dynModeMoon,dynModeSun,dynModeTest};

    class DynError{
    public:
        unsigned Value;

    };

   class MotionDesc
   {
   public:
       MotionDesc();
       double posK;
       double posT;
       double velK;
       double velT;
   };

   class ModeDesc{
   public:
       double h;
       double b;
       double l;
       DynaModes md;
       unsigned char angFlag;
   };

   class DpnRelocationPars{
   public:
       double redCoeff;
       double maxVel;
       double maxAcc;
       double relaxTime;
   };

   class Settings
   {
   public:
      Settings()
      {
        // Задаем разрешенные зоны приводов альфа и бета по умолчанию
        Kren[0] = -45.0;
        Kren[1] = 45.0;
        Tang[0] = -30.0;
        Tang[1] = 30.0;
        // Задаем по умолчанию выбор решения из "задней" полусферы
        _solChoise = 0;
        // Значения коэффициенттов фильтра высоких частот по умолчанию
        _lpFltCoeff[0] = 0.111635212;
        _lpFltCoeff[1] = 0.776729577;
        _lpfTimeConstant = TACT_H*0.5*(1.0-_lpFltCoeff[0])/_lpFltCoeff[0];
        // Обнуляем значение дробной части липсекунд
        _leapSecFrac = 0.0;
      }

      void initRelocParams(Settings * par)   { RelocParams = par->RelocParams; }

      quaterniond                      _qSm2DpnP0;             ///< Установочный кватернион ДПНП0 (СМ->ДПНП0)
      quaterniond                      _qSm2Dpn;               ///< Установочный кватернион ДПН   (СМ->ДПН)
      quaterniond                      _qTelescope;            ///< Установочный кватернион телескопа (Телескоп->ДПН подвижная)
      quaterniond                      _qCnt2Prf;              ///< Кватернион доворота при наблюдении за Солнцем или Луной (Телескоп->ДПН подвижная)
      vectord                          _vTel;                  ///< Вектор направления оптической оси телескопа в системе координат ДПН0
      vectord                          _vTelB;                 ///< Вектор направления оси визирования телескопа в связанной системе координат телескока
      MotionDesc                       _telPos;                ///< Углы направления оси визирования телескопа в системе координат ДПН0
      unsigned char                    _solChoise;             ///< Настройка, отвечающая за выбор решения углов наведения ДПН (0-решение из "задней" полусферы, 1-решение из "передней" полусферы, 2-автоматич. выбор решения из условий ограничений")
      double                           Kren[2];
      double                           Tang[2];
      double                           _lpFltCoeff[2];         ///< Коэффициенты фильтра упругих колебаний
      double                           _lpfTimeConstant;       ///< Постоянная времени фильтра нижних частот
      double                           _leapSecFrac;           ///< Дробная часть липсекунд
      char                             _relocType;             ///< Тип постороения траектории
      const DpnRelocationPars *        RelocParams;           ///< Структура, описывающая параметры управляемого переброса платформы
   };

   class RelocationTraj
   {
      public:
        bool calcSpline(double effVel, double effAcc, const MotionDesc &currPos,  const MotionDesc &frsPos, unsigned char angFlag)
        {
            double ac = effAcc;
            double v = effVel;
            double a0 = currPos.posK,
                   at = frsPos.posK,
                   va0 = currPos.velK,
                   vat = frsPos.velK;
            double b0 = currPos.posT,
                   bt = frsPos.posT,
                   vb0 = currPos.velT,
                   vbt = frsPos.velT;
            /* Проверяем переданные значения */
            if(v<=0 || ac<=0 || (fabs(at-a0)<0.001 && fabs(bt-b0)<0.001))
                return false;

             double T1, T2, T3, Ta, Tb, T;
             /* Вычисляем времена переброса по крену и тангажу */
             if(angFlag==0 || angFlag==1)
             {
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
                relocationTime = T;
             else if(angFlag==1)
                relocationTime = Ta;
             else if(angFlag==2)
                relocationTime = Tb;
             else
                return false;

             T = relocationTime;
             double a, b;
             /* Рассчитываем коэффициенты сплайнов для приводов Альфа и Бета */
             a = at - va0*T - a0;
             b = vat - va0;
             kSpline[0] = (T*b - 2.0*a)/(pow(T,3));
             kSpline[1] = (3.0*a - T*b)/(pow(T,2));
             kSpline[2] = va0;
             kSpline[3] = a0;

             a = bt - vb0*T - b0;
             b = vbt - vb0;
             TSpline[0] = (T*b - 2.0*a)/(pow(T,3));
             TSpline[1] = (3.0*a - T*b)/(pow(T,2));
             TSpline[2] = vb0;
             TSpline[3] = b0;

             return true;
        }

        bool getTrajectory(const double T, MotionDesc *motionDesc)
        {
            motionDesc->posK = kSpline[0]*T*T*T + kSpline[1]*T*T + kSpline[2]*T + kSpline[3];
            motionDesc->posT = TSpline[0]*T*T*T + TSpline[1]*T*T + TSpline[2]*T + TSpline[3];
            motionDesc->velK = 3.0*kSpline[0]*T*T + 2.0*kSpline[1]*T + kSpline[2];
            motionDesc->velT = 3.0*TSpline[0]*T*T + 2.0*TSpline[1]*T + TSpline[2];
            return true;
        }

        bool loPassFilter(const MotionDesc &currPos, MotionDesc *fltMotionDesc, const double *fltCoeff)
        {
            double alpha = fltCoeff[0];
            double beta =  fltCoeff[1];
            double a = prevFltTraj.posK,
                   b = prevFltTraj.posT,
                   va = prevFltTraj.velK,
                   vb = prevFltTraj.velT;
            fltMotionDesc->posK = alpha*(currPos.posK + prevTrajPos.posK) + beta*a;
            fltMotionDesc->posT = alpha*(currPos.posT + prevTrajPos.posT) + beta*b;
            fltMotionDesc->velK = alpha*(currPos.velK + prevTrajPos.velK) + beta*va;
            fltMotionDesc->velT = alpha*(currPos.velT + prevTrajPos.velT) + beta*vb;
            memcpy(&prevTrajPos, &currPos, sizeof(MotionDesc));
            memcpy(&prevFltTraj, fltMotionDesc, sizeof(MotionDesc));
            return true;
        }

        double getRelocationTime(void) {return relocationTime;}

        void setPreFltTraj(const MotionDesc &currPos) {
           memcpy(&prevFltTraj, &currPos, sizeof(MotionDesc));
           memcpy(&prevTrajPos, &currPos, sizeof(MotionDesc));
        }

      private:
        double            kSpline[4];        ///< К-ты разгонного сплайна переброса по крену
        double            TSpline[4];        ///< К-ты тормозного сплайна переброса по тангажу
        double            relocationTime;    ///< Время, необходимое для переброса
        MotionDesc        prevTrajPos;       ///< Предыдущее положение для фильтра НЧ
        MotionDesc        prevFltTraj;       ///< Предыдущее фильтрованная траектория
   };

   class ITrajectory
   {
   public:

      void set(double fi0= -5.0, double vc1= 180.0, double vc2= 60.0, double vc3= 4.0, double it = 60.0, double rt = 60.0, double wt = 10.0)
            {
                // Инициализация настроек параметров движения
               _angle0 = fi0;
               _velConst1 = (fabs(vc1) / 60.0 <= VEL_MAX && fabs(vc1) / 60.0 >= VEL_MIN) ? fabs(vc1) / 60.0 : VEL_MAX;
               _velConst2 = (fabs(vc2) / 60.0 <= VEL_MAX && fabs(vc2) / 60.0 >= VEL_MIN) ? fabs(vc2) / 60.0 : VEL_MAX;
               _velConst3 = (fabs(vc3) / 60.0 <= VEL_MAX && fabs(vc3) / 60.0 >= VEL_MIN) ? fabs(vc3) / 60.0 : VEL_MIN;
               _initTime = fabs(it);
               _relxTime = fabs(rt);
               _waitTime = fabs(wt);
               // Обнуление текущих параметров движения
               _time = 0.0;
               _step = 0;
               _angle = 0.0;
               _velosity = 0.0;
            };

      DynaErrors init(const ModeDesc& data)
             {
                if(data.b < 0.0 || data.b > 4.0)
                   return dynErrNumTraj;
                if(data.l < -MAX_ANGLE || data.l > MAX_ANGLE)
                   return dynErrBadAngle;
                if(data.h < 0 && data.b<3.5)
                   return dynErrBadTime;

                memcpy(&modeDesc, &data, sizeof(ModeDesc));

                set(data.l, 120.0, 60.0, 1.0, 30, 60.0, 10.0);
                initTrajectory((uint8_t) modeDesc.b);
                deltaFi = data.h;
                return dynErrOk;
             }

      void initTrajectory(unsigned char mode)
           {
              _step = -1;
              _time = 0;
              _angle = _angle0;
              switch(mode)
                 {
                 case 0:
                    _velosity = _velConst1;
                    deltaFi = 90;
                    break;
                 case 1:
                    _velosity = _velConst2;
                    deltaFi = 90;
                    break;
                 case 2:
                    _velosity = _velConst3;
                    deltaFi = 10;
                    break;
                 case 3:
                    _velosity = 0;
                    break;
                 default:
                    break;
                 }
           };

      bool createTrajectory(unsigned char mode, MotionDesc *pos, const Settings &settings)
          {
             double Vel = 0;
             double VelN = Vel;
             short trajType = 0;
             double waitTime = 0;
             bool retValue = true;
             double bppMaxAcc = settings.RelocParams->maxAcc;
             double bppMaxVel = settings.RelocParams->maxVel;
             MotionDesc frsPos;
             bool relocation = false;

             /* В соответствии с типом траектории подбираем скорость и задержки*/
             switch(mode)
             {
             case 0:
                Vel = _velConst1;
                trajType = 1;
                waitTime = _relxTime;
                break;
             case 1:
                Vel = _velConst2;
                trajType = 1;
                waitTime = _relxTime;
                break;
             case 2:
                Vel = _velConst3;
                trajType = 1;
                waitTime = _relxTime;
                break;
             case 3:
                trajType = 2;
                waitTime = _waitTime;
                break;
             case 4:
                trajType = 3;
                waitTime = _angle0;
                break;
             default:
                retValue = false;
                break;
             }

                 /* Случай 1: Движение с постоянной скоростью */
                 if(trajType == 1)
                 {
                     /* Шаг 1: Переброс платформы */
                     switch(_step)
                     {
                        // Ожидание начала движения либо расчет траектории переброса
                        case -1:
                            if(settings._relocType == 0)
                                 if(_time > _initTime)
                                     _step = 1;
                                 else
                                     Vel = 0.0;
                            else
                            {
                                frsPos.posK = _angle0;
                                frsPos.posK = _angle0;
                                frsPos.velK = Vel;
                                frsPos.velT = Vel;
                                relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                _step++;
                                relocation = true;
                            }
                            break;
                        // Переброс платформы из текущего положения в необходимое для начала движения
                        case 0:
                            relocationTraj.getTrajectory(_time, pos);
                            if(_time >= relocationTraj.getRelocationTime())
                                _step++;
                            relocation = true;
                            break;
                        /* Шаг 2: Разгон с постоянной скоростью + движение */
                        case 1:
                            if(_angle < _angle0 + deltaFi)
                            {
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                            }
                            else
                            {
                                _step++;
                                _stopTime = _time;
                                if(settings._relocType > 0)
                                {
                                    relocation = true;
                                    frsPos.posK = _angle + Vel*Vel/bppMaxAcc - Vel*Vel/(2.0*bppMaxAcc);
                                    frsPos.posK = frsPos.posK;
                                    frsPos.velK = 0.0;
                                    frsPos.velT = 0.0;
                                    _angle = frsPos.posK;
                                    relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                    memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                    relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                }
                            }
                            break;

                        /* Шаг 3: Ожидание успокоения платформы */
                        case 2:
                            if(_time < _stopTime + waitTime)
                            {
                                Vel=0.0;
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                                if(settings._relocType > 0 && _time <= _stopTime + relocationTraj.getRelocationTime())
                                {
                                    relocationTraj.getTrajectory(_time - _stopTime, pos);
                                    relocation = true;
                                }
                            }
                            else
                            {
                                _step ++;
                                if(settings._relocType > 0)
                                {
                                    relocation = true;
                                    frsPos.posK = _angle - Vel*Vel/(2.0*bppMaxAcc);
                                    frsPos.posK = frsPos.posK;
                                    frsPos.velK = -Vel;
                                    frsPos.velT = -Vel;
                                    _angle = frsPos.posK;
                                    relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                    memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                    relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                }
                            }
                            Vel=0.0;
                            break;
                        /* Шаг 4: Движение в обратную сторону */
                        case 3:
                            if(settings._relocType > 0 && _time <= _stopTime + waitTime + relocationTraj.getRelocationTime())
                            {
                                relocationTraj.getTrajectory(_time - _stopTime - waitTime, pos);
                                relocation = true;
                            }
                            else
                            {
                                Vel=-Vel;
                                if(_angle>=_angle0)
                                {
                                    VelN = Vel;
                                    _angle += (Vel+VelN)/2.0*TACT_H;
                                }
                                else
                                {
                                    if(settings._relocType > 0)
                                    {
                                        relocation = true;
                                        frsPos.posK = _angle - Vel*Vel/bppMaxAcc + Vel*Vel/(2.0*bppMaxAcc);
                                        frsPos.posK = frsPos.posK;
                                        frsPos.velK = 0;
                                        frsPos.velT = 0;
                                        _angle = frsPos.posK;
                                        relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                        memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                        relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                        _stopTime = _time;
                                        _step++;
                                    }
                                    else
                                        _step = 5;
                                }
                            }
                            break;
                        case 4:
                            if(settings._relocType > 0 && _time <= _stopTime + relocationTraj.getRelocationTime())
                            {
                                relocationTraj.getTrajectory(_time - _stopTime, pos);
                                relocation = true;
                            }
                            else
                            {
                                _step++;
                                Vel = 0;
                            }
                            break;
                        case 5:
                            memcpy(pos, &currentPosition, sizeof(MotionDesc));
                            initTrajectory(mode);
                            relocation = true;
                            break;
                     }
                }

                 /* Случай 2: Движение с переменной скоростью */
                 if(trajType == 2)
                 {
                     Vel = _velosity;
                     /* Шаг 1: Переброс платформы */
                     switch(_step)
                     {
                        // Ожидание начала движения либо расчет траектории переброса
                        case -1:
                            if(settings._relocType == 0)
                                 if(_time > _initTime)
                                     _step = 1;
                                 else
                                     Vel = 0.0;
                            else
                            {
                                frsPos.posK = _angle0;
                                frsPos.posK = _angle0;
                                frsPos.velK = Vel;
                                frsPos.velT = Vel;
                                relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                _step++;
                                relocation = true;
                            }
                            break;
                        // Переброс платформы из текущего положения в необходимое для начала движения
                        case 0:
                            relocationTraj.getTrajectory(_time, pos);
                            if(_time + TACT_H >= relocationTraj.getRelocationTime())
                                _step++;
                            relocation = true;
                            break;
                        /* Шаг 2: Разгон с постоянной скоростью + движение */
                        case 1:
                            if(_velosity<VEL_MAX)
                            {
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                                Vel += VEL_MIN;
                            }
                            else
                            {
                                _step++;
                                _stopTime = _time;
                            }
                            break;
                        /* Шаг 3: Ожидание успокоения платформы */
                        case 2:
                            if(_time<=_stopTime+waitTime)
                            {
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                            }
                            else
                                _step++;
                            break;
                         /* Шаг 4: Движение в обратную сторону */
                        case 3:
                            if(Vel>-VEL_MAX)
                            {
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                                Vel -= VEL_MIN;
                            }
                            else
                                _step++;
                            break;
                        /* Шаг 5: Движение в обратную сторону */
                        case 4:
                            if(Vel<0)
                            {
                                VelN = Vel;
                                _angle += (Vel+VelN)/2.0*TACT_H;
                                Vel += VEL_MIN;
                            }
                            else
                            {
                                _step++;
                                Vel = 0;
                            }
                            break;
                        case 5:
                            memcpy(pos, &currentPosition, sizeof(MotionDesc));
                            initTrajectory(mode);
                                relocation = true;
                            break;
                    }
                    _velosity = Vel;
                }

                 /* Случай 3: Наблюдение за точкой на поверхности Земли */
                if(trajType==3)
                {
                    double trackA, trackB, velK, velT;
                    double Altitude_km = 400;
                    double Velocity_kms = 7.707;
                    double Ground_Track_Offset_km = deltaFi;//-150;
                    double Time_Before_Nadir = -(waitTime/2.0 + settings.RelocParams->relaxTime);
                    double Nadir_posK = -95.0;
                    double Nadir_posT = 135.0;
                    switch(_step)
                    {
                        case -1:
                            if(settings._relocType == 0)
                            {
                                pos->posK = (Nadir_posK + 180/M_PI * atan(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km));
                                trackB = sqrt(pow(Altitude_km,2) + pow(Velocity_kms*Time_Before_Nadir,2));
                                pos->posK = (Nadir_posK - 180/M_PI * atan(-1.0*Ground_Track_Offset_km/trackB));
                                pos->velK = 0.0;
                                pos->velT = 0.0;

                                if(_time + TACT_H > _initTime)
                                     _step = 1;
                            }
                            else
                            {
                                trackA = (Nadir_posK + 180/M_PI * atan(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km));
                                velK = - 180/M_PI * Velocity_kms / (1.0 + pow(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km,2)) / Altitude_km;
                                trackB = sqrt(pow(Altitude_km,2) + pow(Velocity_kms*Time_Before_Nadir,2));
                                velT = -180/M_PI*Ground_Track_Offset_km*Velocity_kms*Time_Before_Nadir/(pow(trackB,3)*(1.0+pow(Ground_Track_Offset_km/trackB,2)));
                                trackB = (Nadir_posK - 180/M_PI * atan(-1.0*Ground_Track_Offset_km/trackB));
                                frsPos.posK = trackA;
                                frsPos.posK = trackB;
                                frsPos.velK = velK;
                                frsPos.velT = velT;
                                relocationTraj.calcSpline(settings.RelocParams->redCoeff * settings.RelocParams->maxVel, settings.RelocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings._lpFltCoeff);
                                _step++;
                            }
                            break;
                        case 0:
                            relocationTraj.getTrajectory(_time, pos);
                            //setCurrentPos(_currentPosition);
                            if(_time + TACT_H >= relocationTraj.getRelocationTime())
                            {
                                if(settings._relocType>0)
                                    _time-=relocationTraj.getRelocationTime();
                                _step++;
                            }
                            break;
                        case 1:
                            if(settings._relocType == 0)
                                Time_Before_Nadir = _time - _initTime - (waitTime/2.0 + settings.RelocParams->relaxTime);
                            else
                                Time_Before_Nadir = _time - (waitTime/2.0 + settings.RelocParams->relaxTime);
                            /* Расчет траектории движения */
                            trackA = (Nadir_posK + 180/M_PI * atan(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km));
                            velK = - 180/M_PI * Velocity_kms / (1.0 + pow(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km,2)) / Altitude_km;
                            trackB = sqrt(pow(Altitude_km,2) + pow(Velocity_kms*Time_Before_Nadir,2));
                            velT = -180/M_PI*Ground_Track_Offset_km*Velocity_kms*Time_Before_Nadir/(pow(trackB,3)*(1.0+pow(Ground_Track_Offset_km/trackB,2)));
                            trackB = (Nadir_posK - 180/M_PI * atan(-1.0*Ground_Track_Offset_km/trackB));
                            pos->posK = trackA;
                            pos->posK = trackB;
                            pos->velK = velK;
                            pos->velT = velT;
                            if(_time + TACT_H >= relocationTraj.getRelocationTime() + settings.RelocParams->relaxTime + waitTime)
                               _step++;
                            break;
                        case 2:
                            memcpy(pos, &currentPosition, sizeof(MotionDesc));
                            initTrajectory(mode);
                            break;
                    }

                }

                 _time+=TACT_H;

                 // Проверка корректности полученных параметров
                 if(fabs(_angle) > MAX_ANGLE)
                 {
                     retValue = false;
                     Vel = 0;
                     _angle = _angle>0 ? MAX_ANGLE : -MAX_ANGLE;
                 }
                 if(trajType!=3 && !relocation)
                 {
                    pos->velK = Vel;
                    pos->posK = _angle;
                    pos->posK = pos->posK;
                    pos->velT = pos->velK;
                 }

                 if(modeDesc.angFlag == 1)
                 {
                    pos->posK = currentPosition.posK;
                    pos->velT = 0;
                }
                else if(modeDesc.angFlag == 2)
                {
                    pos->posK = currentPosition.posK;
                    pos->velK = 0;
                }

                 return retValue;
         }

      bool calculate(const Settings& settings, MotionDesc *pos, double& dT)
            {
               double minKren = settings.Kren[0]+5, minTang = settings.Tang[0]+5, maxKren = settings.Kren[1]-5, maxTang = settings.Tang[1]-5;
               MotionDesc notFilteredTraj;
               if(_step==-1 && settings._relocType==2)
                  memcpy(pos, &currentPosition, sizeof(MotionDesc));

               bool retValue = createTrajectory((uint8_t)modeDesc.b, &notFilteredTraj, settings);
               if(!(notFilteredTraj.posK >= minKren && notFilteredTraj.posK <= maxKren))
                  notFilteredTraj.velK = 0.0;
               if(notFilteredTraj.posK < minKren)
                  notFilteredTraj.posK = minKren;
               if(notFilteredTraj.posK > maxKren)
                  notFilteredTraj.posK = maxKren;

               if(!(notFilteredTraj.posT >= minTang && notFilteredTraj.posT <= maxTang))
                  notFilteredTraj.velT = 0.0;
               if(notFilteredTraj.posT < minTang)
                  notFilteredTraj.posT = minTang;
               if(notFilteredTraj.posT > maxTang)
                  notFilteredTraj.posT = maxTang;

               // Если нужно, то проводим фильтрацию созданной траектории
               if(settings._relocType==2)
                   relocationTraj.loPassFilter(notFilteredTraj, pos, settings._lpFltCoeff);
               else
                  memcpy(pos, &notFilteredTraj, sizeof(MotionDesc));

               return retValue;
            }

      bool currentDirection(const Settings& settings, double Alpha, double Beta, DynError& err, ModeDesc& data)
      {
         int mode = (int)modeDesc.h;
         if(mode>=0)
            return currentHeocentricDirection(settings,Alpha,Beta,err,data);
         else
            return currentHeliocentricDirection(settings,Alpha,Beta,err,data);
      }

      DynaModes mode() const
      {
         return modeDesc.md;
      }

      void initParams() {
         //инициализируем начальными значениями
         _MAX_FORESEEN_TIME = MAX_FORESEEN_TIME10;
         _dpnQuatFrsStep = DPN_Q_1ST_STEP;
         _dpnVectFrsStep = DPN_VEC_1ST_STEP;
      }

     static DynError::Value vectorsToAngles(const SudnLib::Vector &aAxis,
                                               const SudnLib::Vector &bAxis,
                                               const SudnLib::Vector &zeroDir,
                                               const SudnLib::Vector &trgtDir,
                                               const SudnLib::Vector &nodeDir,
                                               double &Alpha,
                                               double &Beta)
      {
         // 0.0 Инициализируем необходимые переменные
         double sina=0, sinb=0, cosa=0, cosb=0;
         DynError::Value retValue = DynError::DpnCalcOk;
         double sProd;              // Скалярное произведение векторов
         vectord vProd;     // Векторное произведение векторов
         // 1.0 Вычисляем синусы и косинусы углов Альфа и Бета
         // 1.1 Бета - это угол между перпендикулярами к оси Бета из линии узлов
         //     и из нулевого положения
         sProd = nodeDir*bAxis; // Это косинус угла м-ду линией узлов и осью (по идее длина проекции в-ра на ось)
         SudnLib::Vector nodeProj = nodeDir - bAxis*sProd;
         if(nodeProj.absValue()<SudnLib::CALC_THRESHOLD)
            return DynError::DpnCalcError;
         nodeProj = nodeProj/nodeProj.absValue();
         sProd = zeroDir*bAxis; // Это косинус угла м-ду нулевым положением ДПНп и осью (по идее длина проекции в-ра на ось)
         SudnLib::Vector zeroProj = zeroDir - bAxis*sProd;
         if(zeroProj.absValue()<SudnLib::CALC_THRESHOLD)
            return DynError::DpnCalcError;
         zeroProj = zeroProj/zeroProj.absValue();
         cosb = zeroProj*nodeProj;
         vProd = zeroProj|nodeProj;
         sProd = vProd*bAxis;   // cos угла м-ду векторм и осью, если 1, то сонаправлены, если -1, то противоположно направлены
         sinb = sProd>=0.0 ? vProd.absValue() : -vProd.absValue();

         // 1.2 Альфа - это угол между перпендикулярами к оси Альфа из линии узлов
         //     и из направления цели
         sProd = nodeDir*aAxis; // Это косинус угла м-ду линией узлов и осью (по идее длина проекции в-ра на ось)
         nodeProj = nodeDir - aAxis*sProd;
         if(nodeProj.absValue()<SudnLib::CALC_THRESHOLD)
            return DynError::DpnCalcError;
         nodeProj = nodeProj/nodeProj.absValue();
         sProd = trgtDir*aAxis; // Это косинус угла м-ду направлением на цель и осью (по идее длина проекции в-ра на ось)
         SudnLib::Vector trgtProj = trgtDir - aAxis*sProd;
         if(trgtProj.absValue()<SudnLib::CALC_THRESHOLD)
            return DynError::DpnCalcError;
         trgtProj = trgtProj/trgtProj.absValue();
         cosa = nodeProj*trgtProj;
         vProd = nodeProj|trgtProj;
         sProd = vProd*aAxis;   // cos угла м-ду векторм и осью, если 1, то сонаправлены, если -1, то противоположно направлены
         sina = sProd>=0.0 ? vProd.absValue() : -vProd.absValue();


         // 2.0 Вычисляем из синусов и косинусов углы аьфа и бета
         // Переводим синусы и косинусы углов в углы поворота платформы
         if(fabs(sinb)<=1.0)
            Beta = cosb>=0 ? asin(sinb)*180/M_PI :
            ((sinb>=0 ? 180.0 : -180.0) - asin(sinb)*180/M_PI);
         else
            retValue = DynError::DpnCalcError;

         if(fabs(sina)<=1.0)
            Alpha = cosa>=0 ? asin(sina)*180/M_PI :
            ((sina>=0 ? 180.0 : -180.0) - asin(sina)*180/M_PI);
         else
            retValue = DynError::DpnCalcError;

         return retValue;

      }

      static DynError::Value directionToAngles(MotionDesc *pos,
                                                  const SudnLib::Vector& dir,
                                                  const SudnLib::Vector &vTel,
                                                  const char solType,
                                                  const double *zoneA,
                                                  const double *zoneB)
      {
         DynError::Value retValue = DynError::DpnCalcOk;
         double cos5 = cos(M_PI/180*5.0),
                sin5 = sin(M_PI/180*5.0);

         // 1.0 Находим линии пересечения конусов вращения по альфа и бета
         SudnLib::Vector bAxis(1.0,0.0,0.0);
         SudnLib::Vector aAxis(0.0,-cos5,sin5);
         SudnLib::Vector trgtDir, zeroDir;
         trgtDir = dir/dir.absValue();
         zeroDir = vTel/vTel.absValue();

         double k1 = (aAxis[1]*trgtDir[1] + aAxis[2]*trgtDir[2])/aAxis[2],
                k2 = -aAxis[1]/aAxis[2];
         double a = 1.0+k2*k2,
                b = 2.0*k1*k2,
                c = zeroDir[0]*zeroDir[0] - 1.0 + k1*k1;
         double D = b*b-4.0*a*c;
         if(D<0)
            return DynError::DpnCalcError;
         /*double x0,x01,x02,
                y0,y01,y02,
                z0,z01,z02;*/

         double x01, x02,
                y01, y02,
                z01, z02;

         y01 = (-b+sqrt(D))/(2.0*a);
         y02 = (-b-sqrt(D))/(2.0*a);
         z01 = k1 + y01*k2;
         z02 = k1 + y02*k2;
         x01 = zeroDir[0];
         x02 = zeroDir[0];

         SudnLib::Vector nodeDir[3];
         nodeDir[0] = SudnLib::Vector(x01,y01,z01); // Просто инициализируем чем-нибудь
         nodeDir[1] = SudnLib::Vector(x01,y01,z01);
         nodeDir[2] = SudnLib::Vector(x02,y02,z02);

        /* SudnLib::Vector nodeDir2(x02,y02,z02);
         SudnLib::Vector nodeDir;*/

         // 2.0 Выбираем среди двух найденных линий пересечения вариант с минимальным углом Бета
         // Угол бета - угол м-ду вектором нулевого положения ДПНп и линией пересечения конусов вращения
         // double sProd, sProd1, sProd2;
         double sProd1, sProd2;
         SudnLib::Vector vProd;
         unsigned short nodeIndex = 0;
         sProd1 = zeroDir*nodeDir[1]; // Это косинус бета
         sProd2 = zeroDir*nodeDir[2];

         // 3.0 Сортируем получившиеся решения на решение из "задней полусферы и из "передней" полусферы
         // Вариант с малым углом Бета это решение из передней полусферы
         if(sProd1<0.0 && sProd2>=0.0)
            nodeIndex = 2;
         else if(sProd2<0.0 && sProd1>=0.0)
            nodeIndex = 1;
         else if(sProd2>=0.0 && sProd1>=0.0)    // В этом случаем берем минимальный угол Бета
         {
            if(sProd2>=sProd1)
                nodeIndex = 2;
            else
                nodeIndex = 1;
         }

         // 4.0 Выбираем индекс узла исходя из настройки solType
         double alpha[3], beta[3];
         double minAlpha = zoneA[0], minBeta = zoneB[0], maxAlpha = zoneA[1], maxBeta = zoneB[1];
         switch(solType)
         {
            case 0:     // Решение из "задней" полусферы
                nodeIndex = nodeIndex==1?2:1;
                retValue = vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir[nodeIndex], alpha[nodeIndex], beta[nodeIndex]);
                break;
            case 1:     // Решение из "передней" полусферы
                retValue = vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir[nodeIndex], alpha[nodeIndex], beta[nodeIndex]);
                break;
            case 2:     // Выбор решения исходя из зон разрешенных углов c приоритетом "передней" полусферы
            case 3:     // Выбор решения исходя из зон разрешенных углов c приоритетом "задней" полусферы
                {
                    bool node1Valid = false, node2Valid = false;
                    DynError::Value retValue1, retValue2;
                    // Находим величины углов для каждой линии узлов
                    retValue1 = vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir[1], alpha[1], beta[1]);
                    retValue2 = vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir[2], alpha[2], beta[2]);
                    retValue = (retValue1==DynError::DpnCalcOk && retValue2==DynError::DpnCalcOk)?DynError::DpnCalcOk:DynError::DpnCalcError;
                    if(retValue==DynError::DpnCalcError)
                        return DynError::DpnCalcError;
                    /* Проверяем какой из узлов удовлетворяет заданным ограничениям */
                    if(alpha[1]>=minAlpha && alpha[1]<=maxAlpha &&
                       beta[1]>=minBeta && beta[1]<=maxBeta)
                        node1Valid = true;
                    if(alpha[2]>=minAlpha && alpha[2]<=maxAlpha &&
                       beta[2]>=minBeta && beta[2]<=maxBeta)
                        node2Valid = true;
                    if(node1Valid && node2Valid)
                    {
                        if(solType==3)
                            nodeIndex = nodeIndex==1?2:1;
                        break;
                    }
                    else if(node1Valid && !node2Valid)
                        nodeIndex = 1;
                    else if(!node1Valid && node2Valid)
                        nodeIndex = 2;
                    else if(!node1Valid && !node2Valid)
                        return DynError::DpnDeniedZnError;
                }
                break;
            default:
                return DynError::DpnSolTypeBError;
         }

         // Определяем выбранные значения углово поворота ДПН, соответствующие индексу
         // выбранного узла
         if(nodeIndex == 1)
         {
            pos->posK = alpha[1];
            pos->posK = beta[1];
         }
         else if(nodeIndex == 2)
         {
            pos->posK = alpha[2];
            pos->posK = beta[2];
         }
         else
            return DynError::DpnCalcError;

         // Если ранее не проверяли, то проверяем, что углы внутри допустимого диапазона
         if(solType==0 || solType==1)
         {
            if((retValue == DynError::DpnCalcOk) && !(pos->posK >= minAlpha && pos->posK <= maxAlpha))
                retValue = DynError::DpnUporAError;
            if((retValue == DynError::DpnCalcOk) && !(pos->posK >= minBeta && pos->posK <= maxBeta))
                retValue = DynError::DpnUporBError;
         }

         return retValue;
      }

      void setCurrentPos(const MotionDesc &currPos)
      {
        double va = (currPos.posK - currentPosition.posK) / TACT_H;
        double vb = (currPos.posK - currentPosition.posK) / TACT_H;
        memcpy(&previousPosition, &currentPosition, sizeof(MotionDesc));
        memcpy(&currentPosition, &currPos, sizeof(MotionDesc));
        currentPosition.velK = va;
        currentPosition.velT = vb;
      }

      void zeroCounter(void){ _startCounter = 0;};
      void increaseCounter(void) { _startCounter++;};
      uint32_t getCounter(void) {return _startCounter;};
      void zeroLpfParams(const MotionDesc &currPos) {relocationTraj.setPreFltTraj(currPos);
                                                    };

   protected:
      ModeDesc                      modeDesc;              ///< Уставки на текущий режим наведения
      MotionDesc                    currentPosition;
      MotionDesc                    previousPosition;
      RelocationTraj                   relocationTraj;
      uint32_t                         _startCounter;          ///< Счетчик тактов при рестарете траектории
      int16_t                          _step;                  ///< Текущее положение на траектории
      double                           _MAX_FORESEEN_TIME;     ///< Максммальное время прогноза вектора состояния и ориентации
      double                           _dpnQuatFrsStep;        ///< Шаг интегрирования кватерниона, с
      double                           _dpnVectFrsStep;        ///< Шаг интегрирования вектора состояния, с
      double                           _relocStartTime;         ///< Время начала переброса из начального положения на траектории движения

      bool currentHeliocentricDirection(const Settings& settings, double Alpha, double Beta, DynError & err,ModeDesc& data)
      {
         // Присваиваем текущий вектор источника исходному вектору состояния станции в WGS84
         vectord SrcWGS84;

         // Прогнозируем ориентацию станции(ДПН) в системе J2000 на текущий момент времени
         quaterniond qA;
         vectord vW;
         quaterniond A0;
         // Разворот ДПН подвижной, относительно J2000
         mul_q(&A0,A0,settings._qSm2Dpn);
         // Разворот относительно нулей ДПН подвижной на заданные углы
         quaterniond qDpn2Dpnp(  /* 1.0,0.0,0.0,0.0*/
          cos(M_PI/180*(175.0-Alpha)/2.0)*cos(M_PI/180*(175.0-Beta)/2.0),
         -cos(M_PI/180*(175.0-Alpha)/2.0)*sin(M_PI/180*(175.0-Beta)/2.0),
         -sin(M_PI/180*(175.0-Alpha)/2.0)*cos(M_PI/180*(175.0-Beta)/2.0),
         -sin(M_PI/180*(175.0-Alpha)/2.0)*sin(M_PI/180*(175.0-Beta)/2.0)
         );
         A0 = A0 * qDpn2Dpnp;
         // Разворот телескопа ДПН подвижной берем из настроек
         quaterniond qDpnp2Tel = settings._qTelescope;
         qDpnp2Tel = ~qDpnp2Tel;
         A0 = A0 * qDpnp2Tel;
         mul_q(&A0,A0,qDpnp2Tel);
         conj_q(&A0,A0)

         double Mtel2J2000[9];       // Матрица перехода от телескопа к J2000
         A0.toMatrix(Mtel2J2000);
         vectord vTel = settings._vTelB;
         vectord vView = vTel.mulToMatR(Mtel2J2000);   // Вектор направления телескопа в J2000
         vView = vView / vView.absValue();

         // Получаем гелиоцентрические координаты линии визирования
         double projEqat = sqrt(vView[0] * vView[0] + vView[1] * vView[1]);
         double _l,_b = asin(vView[2])*180/M_PI;
         if(fabs(projEqat) < SudnLib::CALC_THRESHOLD)
            _l = 0.0;
         else
            _l = acos(vView[0]/projEqat) * 180/M_PI;
         if(vView[1]<0.0)
            _l = -_l;

//         data.md = dynModeStars;
         data.l = _l;
         data.b = _b;
         data.h = 0.0;

         return true;
      }


      bool currentHeocentricDirection(const Settings& settings, double Alpha, double Beta, DynError & err,ModeDesc& data)
      {
         double dT = 0.0;
         // Присваиваем текущий вектор источника исходному вектору состояния станции в WGS84
         SudnLib::StateVector SrcWGS84(_2hzData.GTIDB_X);
         // Формируем времена, на которые будем строить прогнозы
         Mpc::Core::TimeServer::OnboardTime cTime;
         uint32_t currSec;
         uint8_t currNSec;
         cTime.ccsdsUnsegmentedTime(currSec,currNSec);
         // Время прогнозирование ориентации
         double  deltaTO = (double)currSec + (double)currNSec/256.0 -
            (_5hzData.F_Unseg_Time_Coarse + _5hzData.F_Unseg_Time_Fine/65536.0) + dT;
         // Время прогнозирования навигации
         double deltaTN = (double)currSec + (double)currNSec/256.0 - _2hzData.GTIDB_TG -
            (double)_2hzData.F_ASN_toCorrection_LeapSec + dT;

         // Проверяем, что задержки не сильно большие
         if(deltaTO > _MAX_FORESEEN_TIME || deltaTO<0) {
            err.set(DynError::DpnAttError, deltaTO, deltaTN);
            return false;
         }
         if(deltaTN > _MAX_FORESEEN_TIME || deltaTN<0) {
            err.set(DynError::DpnNavError, deltaTO, deltaTN);
            return false;
         }

         // Прогнозируем ориентацию станции(ДПН) в системе J2000 на текущий момент времени
         quaterniond qA(_5hzData.GTIFQ_A);
         vectord vW(_5hzData.GTIFV_W);
         quaterniond A0 = SudnLib::foreseen_Q_We(qA, vW, deltaTO, _dpnQuatFrsStep);
         // Разворот ДПН подвижной, относительно J2000
         A0 = A0 * settings._qSm2Dpn;
         // Разворот относительно нулей ДПН подвижной на заданные углы
         quaterniond qDpnp2Dpnp(  /* 1.0,0.0,0.0,0.0 */
          cos(M_PI/180*(175.0-Alpha)/2.0)*cos(M_PI/180*(175.0-Beta)/2.0),
         -cos(M_PI/180*(175.0-Alpha)/2.0)*sin(M_PI/180*(175.0-Beta)/2.0),
         -sin(M_PI/180*(175.0-Alpha)/2.0)*cos(M_PI/180*(175.0-Beta)/2.0),
         -sin(M_PI/180*(175.0-Alpha)/2.0)*sin(M_PI/180*(175.0-Beta)/2.0)            /* */
         );
         A0 = A0 * qDpnp2Dpnp;
         // Разворот телескопа ДПН подвижной берем из настроек
         SudnLib::Quaternion qDpnp2Tel = settings._qTelescope;
         qDpnp2Tel = ~qDpnp2Tel;
         A0 = A0 * qDpnp2Tel;

         A0 = ~A0;
         // Матрица перехода от J2000 к WGS84
         double GREEN[9];
         SudnLib::j2000ToGreenvich(GREEN, (double)currSec + (double)currNSec/256.0 - (double)_2hzData.F_ASN_toCorrection_LeapSec + dT);
         SudnLib::Quaternion qJ20002Wgs84 = SudnLib::quaternionFromMatrix(GREEN);
         qJ20002Wgs84 = ~qJ20002Wgs84;
         A0 = A0*qJ20002Wgs84;

         double Mtel2Wgs84[9];       // Матрица перехода от телескопа к WGS84
         A0.toMatrix(Mtel2Wgs84);
         SudnLib::Vector vTel = settings._vTelB;
         SudnLib::Vector vView = vTel.mulToMatR(Mtel2Wgs84);   // Вектор направления телескопа в WGS84

         // Прогнозируем вектор состояния на текущий момент времени
         SrcWGS84 = SudnLib::foreseen_VsWGS84(SrcWGS84, deltaTN, 0, _dpnVectFrsStep);

         // Ищем точки пересечения оси визирования телескопа с поверхностью Земли на определенной высоте
         double polR = SudnLib::earthMajorSemiaxis * (1.0 - SudnLib::earthEllipticity) + modeDesc.h;    // Полярный радиус эллипсоида
         double equR = SudnLib::earthMajorSemiaxis + modeDesc.h;                                        // Экваториальный радиус эллипсоида
         double polR2 = polR * polR;
         double equR2 = equR * equR;
         // Вычисление коэффициентов квадратного уравнения
         double t=0;
         double c = (SrcWGS84[0]*SrcWGS84[0] + SrcWGS84[1]*SrcWGS84[1])/equR2 + (SrcWGS84[2]*SrcWGS84[2])/polR2 - 1.0;
         double b = 2.0 * (SrcWGS84[0]*vView[0]/equR2 + SrcWGS84[1]*vView[1]/equR2 + SrcWGS84[2]*vView[2]/polR2);
         double a = vView[0]*vView[0] / equR2 + vView[1]*vView[1] / equR2 + vView[2]*vView[2] / polR2;
         double D = b*b - 4.0*a*c;
         if(D<0.0)
         {
            return false;
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
         SudnLib::Vector pointCoord(SrcWGS84[0] + t*vView[0], SrcWGS84[1] + t*vView[1], SrcWGS84[2] + t*vView[2]);
         // Переводим декартовы координаты в геодезические
         SudnLib::Vector geodes = SudnLib::XYZTolbh(pointCoord);

//         data.md = dynModeEarth;
         data.l = geodes[0];
         data.b = geodes[1];
         data.h = geodes[2];

         return true;
      }
       double      deltaFi;      ///< Угол поворота платформы при движении с постоянной скоростью
   private:
            double      _time;         ///< Текущее время на траектории движения
            double      _stopTime;     ///< Время остановки платфомы / время выхода на максимальную скорость
            double      _angle0;       ///< Начальный угол поворота платформы, от которого начинается отработка траектории
            double      _angle;        ///< Текущий угол поворота платформы
            double      _velosity;     ///< Текущая скорость движения платформы
            double      _velConst1;    ///< Постоянная скорость движения платформы в первом тесте
            double      _velConst2;    ///< Постоянная скорость движения платформы во втором тесте
            double      _velConst3;    ///< Постоянная скорость движения платформы в третьем тесте
            double      _initTime;     ///< Время успокоения платформы после перехода в начальную точку траектории
            double      _relxTime;     ///< Время успокоения платформы при движении с постоянной скоростью
            double      _waitTime;     ///< Задержка при измененнии знака ускорения при движении с переменной скоростью
            uint16_t    _repeatNum;    ///< Кол-во повторений тестовой траектории
   };

}

#endif //TRAJ_H
