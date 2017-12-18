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

   class RelocationPars{
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
        solChoise = 0;
        // Значения коэффициенттов фильтра высоких частот по умолчанию
        FltCoeff[0] = 0.111635212;
        FltCoeff[1] = 0.776729577;
        FTimeConstant = TACT_H*0.5*(1.0-FltCoeff[0])/FltCoeff[0];
        // Обнуляем значение дробной части липсекунд
        leapSecFrac = 0.0;
      }

      void initRelocParams(Settings * par)   { RelocParams = par->RelocParams; }

      quaterniond                      _qSm2DpnP0;             ///< Установочный кватернион ДПНП0 (СМ->ДПНП0)
      quaterniond                      _qSm2Dpn;               ///< Установочный кватернион ДПН   (СМ->ДПН)
      quaterniond                      _qTelescope;            ///< Установочный кватернион телескопа (Телескоп->ДПН подвижная)
      quaterniond                      _qCnt2Prf;              ///< Кватернион доворота при наблюдении за Солнцем или Луной (Телескоп->ДПН подвижная)
      vectord                          _vTel;                  ///< Вектор направления оптической оси телескопа в системе координат ДПН0
      vectord                          _vTelB;                 ///< Вектор направления оси визирования телескопа в связанной системе координат телескока
      MotionDesc                       _telPos;                ///< Углы направления оси визирования телескопа в системе координат ДПН0
      unsigned char                    solChoise;             ///< Настройка, отвечающая за выбор решения углов наведения ДПН (0-решение из "задней" полусферы, 1-решение из "передней" полусферы, 2-автоматич. выбор решения из условий ограничений")
      double                           Kren[2];
      double                           Tang[2];
      double                           FltCoeff[2];         ///< Коэффициенты фильтра упругих колебаний
      double                           FTimeConstant;       ///< Постоянная времени фильтра нижних частот
      double                           leapSecFrac;           ///< Дробная часть липсекунд
      char                             relocType;             ///< Тип постороения траектории
      const RelocationPars *        RelocParams;           ///< Структура, описывающая параметры управляемого переброса платформы
   };

   class RelocationTraj
   {
      public:
        bool calcSpline(double effVel, double effAcc, const MotionDesc &currPos,  const MotionDesc &frsPos, unsigned char angFlag)
        {
            double ac = effAcc;
            double v = effVel;
            double kren0 = currPos.posK,
                   krent = frsPos.posK,
                   VKren0 = currPos.velK,
                   VKrent = frsPos.velK;
            double tang0 = currPos.posT,
                   tangt = frsPos.posT,
                   VTang0 = currPos.velT,
                   VTangt = frsPos.velT;
            /* Проверяем переданные значения */
            if(v<=0 || ac<=0 || (fabs(krent-kren0)<0.001 && fabs(tangt-tang0)<0.001))
                return false;

             double T1, T2, T3, Tkren, Ttang, T;
             /* Вычисляем времена переброса по крену и тангажу */
             if(angFlag==0 || angFlag==1)
             {
                T1 = fabs(krent - kren0) / v;
                T2 = fabs(VKren0 - (krent - kren0)/T1)/ac;
                T3 = fabs((krent - kren0)/T1 - VKrent)/ac;
                Tkren = T1 + T2 + T3;
             }
             if(angFlag==0 || angFlag==2)
             {
                T1 = fabs(tangt - tang0) / v;
                T2 = fabs(VTang0 - (tangt - tang0)/T1)/ac;
                T3 = fabs((tangt - tang0)/T1 - VTangt)/ac;
                Ttang = T1 + T2 + T3;
             }
             if(angFlag==0)
                T = Tkren>Ttang?Tkren:Ttang;
             if(angFlag==0)
                relocationTime = T;
             else if(angFlag==1)
                relocationTime = Tkren;
             else if(angFlag==2)
                relocationTime = Ttang;
             else
                return false;

             T = relocationTime;
             double a, b;
             /* Рассчитываем коэффициенты сплайнов для приводов Альфа и Бета */
             a = krent - VKren0*T - kren0;
             b = VKrent - VKren0;
             kSpline[0] = (T*b - 2.0*a)/(pow(T,3));
             kSpline[1] = (3.0*a - T*b)/(pow(T,2));
             kSpline[2] = VKren0;
             kSpline[3] = kren0;

             a = tangt - VTang0*T - tang0;
             b = VTangt - VTang0;
             TSpline[0] = (T*b - 2.0*a)/(pow(T,3));
             TSpline[1] = (3.0*a - T*b)/(pow(T,2));
             TSpline[2] = VTang0;
             TSpline[3] = tang0;

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
            double c1 = fltCoeff[0];
            double c2 =  fltCoeff[1];
            double kren = prevFltTraj.posK,
                   tang = prevFltTraj.posT,
                   vkren = prevFltTraj.velK,
                   vtang = prevFltTraj.velT;
            fltMotionDesc->posK = c1*(currPos.posK + prevTrajPos.posK) + c2*kren;
            fltMotionDesc->posT = c1*(currPos.posT + prevTrajPos.posT) + c2*tang;
            fltMotionDesc->velK = c1*(currPos.velK + prevTrajPos.velK) + c2*vkren;
            fltMotionDesc->velT = c1*(currPos.velT + prevTrajPos.velT) + c2*vtang;
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
               angle0 = fi0;
               velConst1 = (fabs(vc1) / 60.0 <= VEL_MAX && fabs(vc1) / 60.0 >= VEL_MIN) ? fabs(vc1) / 60.0 : VEL_MAX;
               velConst2 = (fabs(vc2) / 60.0 <= VEL_MAX && fabs(vc2) / 60.0 >= VEL_MIN) ? fabs(vc2) / 60.0 : VEL_MAX;
               velConst3 = (fabs(vc3) / 60.0 <= VEL_MAX && fabs(vc3) / 60.0 >= VEL_MIN) ? fabs(vc3) / 60.0 : VEL_MIN;
               initTime = fabs(it);
               relxTime = fabs(rt);
               waitTime = fabs(wt);
               // Обнуление текущих параметров движения
               time = 0.0;
               step = 0;
               angle = 0.0;
               velosity = 0.0;
            }

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
              step = -1;
              time = 0;
              angle = angle0;
              switch(mode)
                 {
                 case 0:
                    velosity = velConst1;
                    deltaFi = 90;
                    break;
                 case 1:
                    velosity = velConst2;
                    deltaFi = 90;
                    break;
                 case 2:
                    velosity = velConst3;
                    deltaFi = 10;
                    break;
                 case 3:
                    velosity = 0;
                    break;
                 default:
                    break;
                 }
           }

      bool createTrajectory(MotionDesc *pos, const Settings &settings)
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

             trajType = 3;
             waitTime = angle0;
             double trackA, trackB, velK, velT;
             double Altitude_km = 400;
             double Velocity_kms = 7.707;
             double Ground_Track_Offset_km = deltaFi;//-150;
             double Time_Before_Nadir = -(waitTime/2.0 + settings.RelocParams->relaxTime);
             double Nadir_posK = -95.0;
             double Nadir_posT = 135.0;
             switch(step)
             {
                case -1:
                    if(settings.relocType == 0)
                    {
                        pos->posK = (Nadir_posK + 180/M_PI * atan(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km));
                        trackB = sqrt(pow(Altitude_km,2) + pow(Velocity_kms*Time_Before_Nadir,2));
                        pos->posK = (Nadir_posK - 180/M_PI * atan(-1.0*Ground_Track_Offset_km/trackB));
                        pos->velK = 0.0;
                        pos->velT = 0.0;

                        if(time + TACT_H > initTime)
                                     step = 1;
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
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings.FltCoeff);
                                step++;
                            }
                            break;
                        case 0:
                            relocationTraj.getTrajectory(time, pos);
                            //setCurrentPos(_currentPosition);
                            if(time + TACT_H >= relocationTraj.getRelocationTime())
                            {
                                if(settings.relocType>0)
                                    time-=relocationTraj.getRelocationTime();
                                step++;
                            }
                            break;
                        case 1:
                            if(settings.relocType == 0)
                                Time_Before_Nadir = time - initTime - (waitTime/2.0 + settings.RelocParams->relaxTime);
                            else
                                Time_Before_Nadir = time - (waitTime/2.0 + settings.RelocParams->relaxTime);
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
                            if(time + TACT_H >= relocationTraj.getRelocationTime() + settings.RelocParams->relaxTime + waitTime)
                               step++;
                            break;
                        case 2:
                            memcpy(pos, &currentPosition, sizeof(MotionDesc));
                            break;
                    }



                 time+=TACT_H;

                 // Проверка корректности полученных параметров
                 if(fabs(angle) > MAX_ANGLE)
                 {
                     retValue = false;
                     Vel = 0;
                     angle = angle>0 ? MAX_ANGLE : -MAX_ANGLE;
                 }
                 if(trajType!=3 && !relocation)
                 {
                    pos->velK = Vel;
                    pos->posK = angle;
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
               if(step==-1 && settings.relocType==2)
                  memcpy(pos, &currentPosition, sizeof(MotionDesc));

               bool retValue = createTrajectory(&notFilteredTraj, settings);
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
               if(settings.relocType==2)
                   relocationTraj.loPassFilter(notFilteredTraj, pos, settings.FltCoeff);
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

     void vectorsToAngles(const vectord &aAxis,
                                               const vectord &bAxis,
                                               const vectord &zeroDir,
                                               const vectord &trgtDir,
                                               const vectord &nodeDir,
                                               double &Alpha,
                                               double &Beta)
      {
         // 0.0 Инициализируем необходимые переменные
         double sina=0, sinb=0, cosa=0, cosb=0;
         double sProd;              // Скалярное произведение векторов
         vectord vProd;     // Векторное произведение векторов
         // 1.0 Вычисляем синусы и косинусы углов Альфа и Бета
         // 1.1 Бета - это угол между перпендикулярами к оси Бета из линии узлов
         //     и из нулевого положения
         sProd = nodeDir*bAxis; // Это косинус угла м-ду линией узлов и осью (по идее длина проекции в-ра на ось)
         vectord nodeProj = nodeDir - bAxis*sProd;
         if(nodeProj.absValue()<SudnLib::CALC_THRESHOLD)
            return DynError::DpnCalcError;
         nodeProj = nodeProj/nodeProj.absValue();
         sProd = zeroDir*bAxis; // Это косинус угла м-ду нулевым положением ДПНп и осью (по идее длина проекции в-ра на ось)
         vectord zeroProj = zeroDir - bAxis*sProd;
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

      void directionToAngles(MotionDesc *pos,
                                                  const vectord& dir,
                                                  const vectord &vTel,
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

      void zeroCounter(void){ _startCounter = 0;}
      void increaseCounter(void) { _startCounter++;}
      uint32_t getCounter(void) {return _startCounter;}
      void zeroLpfParams(const MotionDesc &currPos) {relocationTraj.setPreFltTraj(currPos);
                                                    }

   protected:
      ModeDesc                      modeDesc;              ///< Уставки на текущий режим наведения
      MotionDesc                    currentPosition;
      MotionDesc                    previousPosition;
      RelocationTraj                   relocationTraj;
      uint32_t                         _startCounter;          ///< Счетчик тактов при рестарете траектории
      int16_t                          step;                  ///< Текущее положение на траектории
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
         conj_q(&A0,A0);

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
            double      time;         ///< Текущее время на траектории движения
            double      stopTime;     ///< Время остановки платфомы / время выхода на максимальную скорость
            double      angle0;       ///< Начальный угол поворота платформы, от которого начинается отработка траектории
            double      angle;        ///< Текущий угол поворота платформы
            double      velosity;     ///< Текущая скорость движения платформы
            double      velConst1;    ///< Постоянная скорость движения платформы в первом тесте
            double      velConst2;    ///< Постоянная скорость движения платформы во втором тесте
            double      velConst3;    ///< Постоянная скорость движения платформы в третьем тесте
            double      initTime;     ///< Время успокоения платформы после перехода в начальную точку траектории
            double      relxTime;     ///< Время успокоения платформы при движении с постоянной скоростью
            double      waitTime;     ///< Задержка при измененнии знака ускорения при движении с переменной скоростью
            uint16_t    repeatNum;    ///< Кол-во повторений тестовой траектории
   };

}

#endif //TRAJ_H
