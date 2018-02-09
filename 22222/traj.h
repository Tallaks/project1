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

   class MotionDesc
   {
   public:
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
        Kren[0] = -180.0;
        Kren[1] = 180.0;
        Tang[0] = -180.0;
        Tang[1] = 180.0;
        // Задаем по умолчанию выбор решения из "задней" полусферы
        solChoise = 0;
        // Значения коэффициенттов фильтра высоких частот по умолчанию
        FltCoeff[0] = 0.111635212;
        FltCoeff[1] = 0.776729577;
        FTimeConstant = TACT_H*0.5*(1.0-FltCoeff[0])/FltCoeff[0];
        // Обнуляем значение дробной части липсекунд
        set_v(&vTel,0.0,1.0,0.0);
        leapSecFrac = 0.0;
      }

      void initRelocParams(Settings * par)   { RelocParams = par->RelocParams; }

      quaterniond                      _qSm2DpnP0;             ///< Установочный кватернион ДПНП0 (СМ->ДПНП0)
      quaterniond                      _qSm2Dpn;               ///< Установочный кватернион ДПН   (СМ->ДПН)
      quaterniond                      _qCnt2Prf;              ///< Кватернион доворота при наблюдении за Солнцем или Луной (Телескоп->ДПН подвижная)
      vectord                          vTel;                  ///< Вектор направления оптической оси телескопа в системе координат ДПН0
      MotionDesc                       _telPos;                ///< Углы направления оси визирования телескопа в системе координат ДПН0
      unsigned char                    solChoise;             ///< Настройка, отвечающая за выбор решения углов наведения ДПН (0-решение из "задней" полусферы, 1-решение из "передней" полусферы, 2-автоматич. выбор решения из условий ограничений")
      double                           Kren[2];
      double                           Tang[2];
      double                           FltCoeff[2];         ///< Коэффициенты фильтра упругих колебаний
      double                           FTimeConstant;       ///< Постоянная времени фильтра нижних частот
      double                           leapSecFrac;           ///< Дробная часть липсекунд
      char                             relocType;             ///< Тип постороения траектории
      RelocationPars *        RelocParams;           ///< Структура, описывающая параметры управляемого переброса платформы
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

        bool getTrajectory(double T, MotionDesc *motionDesc)
        {
            motionDesc->posK = kSpline[0]*T*T*T + kSpline[1]*T*T + kSpline[2]*T + kSpline[3];
            motionDesc->posT = TSpline[0]*T*T*T + TSpline[1]*T*T + TSpline[2]*T + TSpline[3];
            motionDesc->velK = 3.0*kSpline[0]*T*T + 2.0*kSpline[1]*T + kSpline[2];
            motionDesc->velT = 3.0*TSpline[0]*T*T + 2.0*TSpline[1]*T + TSpline[2];
            return true;
        }

        bool loPassFilter(MotionDesc &currPos, MotionDesc *fltMotionDesc, double *fltCoeff)
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

        void setPreFltTraj(MotionDesc &currPos) {
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

      void init(ModeDesc& data)
             {
                memcpy(&modeDesc, &data, sizeof(ModeDesc));
                set(data.l, 120.0, 60.0, 1.0, 30, 60.0, 10.0);
                initTrajectory((uint8_t) modeDesc.b);
                deltaFi = data.h;
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

      void createTrajectory(MotionDesc *pos, Settings &settings)
          {
             double waitTime = 0;
             MotionDesc frsPos;
             waitTime = 10.0;
             double trackA, trackB, velK, velT;
             double Altitude_km = 400;
             double Velocity_kms = 7.707;
             double Ground_Track_Offset_km = deltaFi;//-150;
             double Time_Before_Nadir = -(waitTime/2.0 + settings.RelocParams->relaxTime);
             double Nadir_posK = -95.0;
             switch(step)
             {
                case -1:
                    if(settings.relocType == 0)
                    {
                        pos->posK = (Nadir_posK + 180/M_PI * atan(-1.0*Velocity_kms*Time_Before_Nadir/Altitude_km));
                        trackB = sqrt(pow(Altitude_km,2) + pow(Velocity_kms*Time_Before_Nadir,2));
                        pos->posT = (Nadir_posK - 180/M_PI * atan(-1.0*Ground_Track_Offset_km/trackB));
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
                                frsPos.posT = trackB;
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
                            //setCurrentPos(currentPosition);
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
         }

      void calculate(Settings& settings, MotionDesc *pos)
            {
               double minKren = settings.Kren[0]+5, minTang = settings.Tang[0]+5, maxKren = settings.Kren[1]-5, maxTang = settings.Tang[1]-5;
               MotionDesc notFilteredTraj;
               if(step==-1 && settings.relocType==2)
                  memcpy(pos, &currentPosition, sizeof(MotionDesc));

               createTrajectory(&notFilteredTraj, settings);
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

               }

      void initParams() {
         //инициализируем начальными значениями
         _MAX_FORESEEN_TIME = MAX_FORESEEN_TIME10;
         _dpnQuatFrsStep = DPN_Q_1ST_STEP;
         _dpnVectFrsStep = DPN_VEC_1ST_STEP;
      }

     void vectorsToAngles(vectord aAxis,
                          vectord bAxis,
                          vectord zeroDir,
                          vectord trgtDir,
                          vectord nodeDir
                          )
      {
         // 0.0 Инициализируем необходимые переменные
         double sina=0, sinb=0, cosa=0, cosb=0;
         double sProd;              // Скалярное произведение векторов
         vectord vProd;     // Векторное произведение векторов
         // 1.0 Вычисляем синусы и косинусы углов Альфа и Бета
         // 1.1 Бета - это угол между перпендикулярами к оси Бета из линии узлов
         //     и из нулевого положения
         sProd = dot_v(nodeDir,bAxis); // Это косинус угла м-ду линией узлов и осью (по идее длина проекции в-ра на ось)
         vectord nodeProj,bAsP;
         mul_vf(&bAsP,bAxis,sProd);
         sub_v(&nodeProj,nodeDir,bAsP);
         norm_v(&nodeProj,nodeProj);
         // Это косинус угла м-ду нулевым положением ДПНп и осью (по идее длина проекции в-ра на ось)
         sProd = dot_v(zeroDir,bAxis);
         mul_vf(&bAsP,bAxis,sProd);
         vectord zeroProj;
         sub_v(&zeroProj,zeroDir,bAsP);
         norm_v(&zeroProj,zeroProj);
         cosb = dot_v(zeroProj,nodeProj);
         cross_v(&vProd,zeroProj,nodeProj);
         sProd = dot_v(vProd,bAxis);   // cos угла м-ду векторм и осью, если 1, то сонаправлены, если -1, то противоположно направлены
         sinb = sProd>=0.0 ? abs_v(vProd) : -abs_v(vProd);

         // 1.2 Альфа - это угол между перпендикулярами к оси Альфа из линии узлов
         //     и из направления цели
         sProd = dot_v(nodeDir,aAxis); // Это косинус угла м-ду линией узлов и осью (по идее длина проекции в-ра на ось)
         vectord aAsP;
         mul_vf(&aAsP,aAxis,sProd);
         sub_v(&nodeProj,nodeDir,aAsP);
         norm_v(&nodeProj,nodeProj);
         sProd = dot_v(trgtDir,aAxis); // Это косинус угла м-ду направлением на цель и осью (по идее длина проекции в-ра на ось)
         vectord trgtProj;
         mul_vf(&aAsP,aAxis,sProd);
         sub_v(&trgtProj,trgtDir,aAsP);
         norm_v(&trgtProj,trgtProj);
         cosa = dot_v(nodeProj,trgtProj);
         cross_v(&vProd,nodeProj,trgtProj);
         sProd = dot_v(vProd,aAxis);   // cos угла м-ду векторм и осью, если 1, то сонаправлены, если -1, то противоположно направлены
         sina = sProd>=0.0 ? abs_v(vProd) : -abs_v(vProd);


         // 2.0 Вычисляем из синусов и косинусов углы аьфа и бета
         // Переводим синусы и косинусы углов в углы поворота платформы
            Beta = cosb>=0 ? asin(sinb)*180/M_PI :
            ((sinb>=0 ? 180.0 : -180.0) - asin(sinb)*180/M_PI);
            Alpha = cosa>=0 ? asin(sina)*180/M_PI :
            ((sina>=0 ? 180.0 : -180.0) - asin(sina)*180/M_PI);
      }

      void directionToAngles(MotionDesc *pos,
                                                  vectord dir,
                                                  vectord vTel,
                                                  char solType)
      {
         // 1.0 Находим линии пересечения конусов вращения по альфа и бета
         vectord bAxis={0.0,0.0,1.0};
         vectord aAxis{1.0,0.0,0.0};
         vectord trgtDir, zeroDir;
         norm_v(&trgtDir,dir);
         norm_v(&zeroDir,vTel);

         double x01, x02,
                y01, y02,
                z01, z02;

         y01 = trgtDir[1]/(trgtDir[1]*trgtDir[1]+trgtDir[2]*trgtDir[2]);
         y02 = -trgtDir[1]/(trgtDir[1]*trgtDir[1]+trgtDir[2]*trgtDir[2]);
         z01 = trgtDir[2]/(trgtDir[1]*trgtDir[1]+trgtDir[2]*trgtDir[2]);
         z02 = -trgtDir[2]/(trgtDir[1]*trgtDir[1]+trgtDir[2]*trgtDir[2]);
         x01 = 0;
         x02 = 0;

         vectord nodeDir,nodeDir1,nodeDir2;
         set_v(&nodeDir2,x02,y02,z02);
         set_v(&nodeDir1,x01,y01,z01);

         // 2.0 Выбираем среди двух найденных линий пересечения вариант с минимальным углом Бета
         // Угол бета - угол м-ду вектором нулевого положения ДПНп и линией пересечения конусов вращения
         // double sProd, sProd1, sProd2;
         double sProd1, sProd2;
         sProd1 = dot_v(zeroDir,nodeDir1); // Это косинус бета
         sProd2 = dot_v(zeroDir,nodeDir2);

         // 3.0 Сортируем получившиеся решения на решение из "задней полусферы и из "передней" полусферы
         // Вариант с малым углом Бета это решение из передней полусферы
         if(sProd1<0.0 && sProd2>=0.0)
            copy_v(&nodeDir,nodeDir2);
         else if(sProd2<0.0 && sProd1>=0.0)
            copy_v(&nodeDir,nodeDir1);
         else if(sProd2>=0.0 && sProd1>=0.0)    // В этом случаем берем минимальный угол Бета
         {
            if(sProd2>=sProd1)
                copy_v(&nodeDir,nodeDir2);
            else
                copy_v(&nodeDir,nodeDir1);
         }

         // 4.0 Выбираем индекс узла исходя из настройки solType
         double alpha, beta;  
         switch(solType)
         {
            case 0:     // Решение из "задней" полусфер
                vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir2);
                break;
            case 1:     // Решение из "передней" полусферы
                vectorsToAngles(aAxis, bAxis, zeroDir, trgtDir, nodeDir1);
                break;
            default:
                break;
         }

         // Определяем выбранные значения углово поворота ДПН, соответствующие индексу
         // выбранного узла
            pos->posK = alpha;
            pos->posT = beta;
      }

      void setCurrentPos(MotionDesc &currPos)
      {
        double vk = (currPos.posK - currentPosition.posK) / TACT_H;
        double vt = (currPos.posT - currentPosition.posT) / TACT_H;
        memcpy(&previousPosition, &currentPosition, sizeof(MotionDesc));
        memcpy(&currentPosition, &currPos, sizeof(MotionDesc));
        currentPosition.velK = vk;
        currentPosition.velT = vt;
      }

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


      void currentDirection(Settings *settings, double Alpha, double Beta, ModeDesc *data, vectord r, vectord v)
      {
         // Присваиваем текущий вектор источника исходному вектору состояния станции в WGS84
         vectord SrcWGS84;
         copy_v(&SrcWGS84,r);

         // Прогнозируем ориентацию станции(ДПН) в системе J2000 на текущий момент времени
         quaterniond A0,WGS20ka0;

         // Разворот относительно нулей ДПН подвижной на заданные углы
         quaterniond FrKAtoKA0={   /*1.0,0.0,0.0,0.0*/
          cos(M_PI/180*(180-Alpha)/2.0)*cos(M_PI/180*(180-Beta)/2.0),
          sin(M_PI/180*(180-Alpha)/2.0)*cos(M_PI/180*(180-Beta)/2.0),
         -sin(M_PI/180*(180-Alpha)/2.0)*sin(M_PI/180*(180-Beta)/2.0),
          cos(M_PI/180*(180-Alpha)/2.0)*sin(M_PI/180*(180-Beta)/2.0)
         };
         conj_q(&FrKAtoKA0,FrKAtoKA0);
         copy_q(&A0,FrKAtoKA0);

         Povorot0(&WGS20ka0,r,v);
         conj_q(&WGS20ka0,WGS20ka0);
         mul_q(&A0,A0,WGS20ka0);

         matrixd MKA02Wgs84;       // Матрица перехода от телескопа к WGS84
         matrixd_q(&MKA02Wgs84,A0);
         vectord vTel;
         copy_v(&vTel,settings->vTel);
         vectord vView,s;
         mul_mv(&s,MKA02Wgs84,vTel);   // Вектор направления телескопа в WGS84
         copy_v(&vView,s);

         // Ищем точки пересечения оси визирования телескопа с поверхностью Земли на определенной высоте

         double e = 1/298.257223563;
         double polR = 6378137.0*(1-e);
         double equR = 6378137.0;
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
         vectord pointCoord={SrcWGS84[0] + t*vView[0], SrcWGS84[1] + t*vView[1], SrcWGS84[2] + t*vView[2]};
         // Переводим декартовы координаты в геодезические
         vectord geodes;
         WGS84ToGeo(&geodes,pointCoord[0],pointCoord[1],pointCoord[2]);

//         data.md = dynModeEarth;
         data->l = geodes[0];
         data->b = geodes[1];
         data->h = geodes[2];
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
            double Alpha;
            double Beta;
   };

}

#endif //TRAJ_H
