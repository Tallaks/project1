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
        set_v(&vTel,0.0,1.0,0.0);
        // Обнуляем значение дробной части липсекунд
        leapSecFrac = 0.0;
      }

      void initRelocParams(Settings * par)   { relocParams = par->relocParams; }

      vectord                          vTel;                  ///< Вектор направления оптической оси телескопа в системе координат ДПН0
      unsigned char                    solChoise;             ///< Настройка, отвечающая за выбор решения углов наведения ДПН (0-решение из "задней" полусферы, 1-решение из "передней" полусферы, 2-автоматич. выбор решения из условий ограничений")
      double                           Kren[2];
      double                           Tang[2];
      double                           FltCoeff[2];         ///< Коэффициенты фильтра упругих колебаний
      double                           FTimeConstant;       ///< Постоянная времени фильтра нижних частот
      double                           leapSecFrac;           ///< Дробная часть липсекунд
      char                             relocType;             ///< Тип постороения траектории
      RelocationPars *                 relocParams;           ///< Структура, описывающая параметры управляемого переброса платформы
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

      void createTrajectory(MotionDesc *pos, Settings &settings)
          {
             angle0 = 30;
             MotionDesc frsPos;
             waitTime = 10.0;
             double Vel,VelN;
             double MaxAcc = 3;
             switch(step)
             {
                  /* Шаг 1: Переброс платформы */
                  // Ожидание начала движения либо расчет траектории переброса
                  case -1:

                    break;
                    // Переброс платформы из текущего положения в необходимое для начала движения
                    case 0:
                        relocationTraj.getTrajectory(time, pos);
                        if(time >= relocationTraj.getRelocationTime())
                            step++;
                    break;
                    /* Шаг 2: Разгон с постоянной скоростью + движение */
                    case 1:
                        if(angle < angle0 + deltaFi)
                        {
                            VelN = Vel;
                            angle += (Vel+VelN)/2.0*TACT_H;
                        }
                        else
                        {
                            step++;
                            stopTime = time;
                            if(settings.relocType > 0)
                            {
                                frsPos.posK = angle + Vel*Vel/MaxAcc - Vel*Vel/(2.0*MaxAcc);
                                frsPos.posT = frsPos.posK;
                                frsPos.velK = 0.0;
                                frsPos.velT = 0.0;
                                angle = frsPos.posK;
                                relocationTraj.calcSpline(settings.relocParams->maxVel, settings.relocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings.FltCoeff);
                            }
                        }
                    break;
                    /* Шаг 3: Ожидание успокоения платформы */
                    case 2:
                        if(time < stopTime + waitTime)
                        {
                            Vel=0.0;
                            VelN = Vel;
                            angle += (Vel+VelN)/2.0*TACT_H;
                            if(settings.relocType > 0 && time <= stopTime + relocationTraj.getRelocationTime())
                            {
                                relocationTraj.getTrajectory(time - stopTime, pos);
                            }
                        }
                        else
                        {
                            step ++;
                            if(settings.relocType > 0)
                            {
                                frsPos.posK = angle - Vel*Vel/(2.0*MaxAcc);
                                frsPos.posT = frsPos.posT;
                                frsPos.velK = -Vel;
                                frsPos.velT = -Vel;
                                angle = frsPos.posK;
                                relocationTraj.calcSpline(settings.relocParams->maxVel, settings.relocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                relocationTraj.loPassFilter(currentPosition, &frsPos, settings.FltCoeff);
                            }
                        }
                        Vel=0.0;
                    break;
                    /* Шаг 4: Движение в обратную сторону */
                    case 3:
                        if(settings.relocType > 0 && time <= stopTime + waitTime + relocationTraj.getRelocationTime())
                        {
                            relocationTraj.getTrajectory(time - stopTime - waitTime, pos);
                        }
                        else
                        {
                            Vel=-Vel;
                            if(angle>=angle0)
                            {
                                VelN = Vel;
                                angle += (Vel+VelN)/2.0*TACT_H;
                            }
                            else
                            {
                                if(settings.relocType > 0)
                                {
                                    frsPos.posK = angle - Vel*Vel/MaxAcc + Vel*Vel/(2.0*MaxAcc);
                                    frsPos.posT = frsPos.posK;
                                    frsPos.velK = 0;
                                    frsPos.velT = 0;
                                    angle = frsPos.posK;
                                    relocationTraj.calcSpline(settings.relocParams->redCoeff * settings.relocParams->maxVel, settings.relocParams->maxAcc, currentPosition,  frsPos, modeDesc.angFlag);
                                    memcpy(pos, &currentPosition, sizeof(MotionDesc));
                                    relocationTraj.loPassFilter(currentPosition, &frsPos, settings.FltCoeff);
                                    stopTime = time;
                                    step++;
                                }
                                else
                                step = 5;
                            }
                    }
                    break;
                    case 4:
                        if(settings.relocType > 0 && time <= stopTime + relocationTraj.getRelocationTime())
                        {
                            relocationTraj.getTrajectory(time - stopTime, pos);
                        }
                        else
                        {
                            step++;
                            Vel = 0;
                        }
                    break;
                    case 5:
                        memcpy(pos, &currentPosition, sizeof(MotionDesc));
                        //initTrajectory(mode);
                    break;
            }
                 time+=TACT_H;
         }

      void calculate(Settings& settings, MotionDesc *pos)
            {
               MotionDesc notFilteredTraj;
               if(step==-1 && settings.relocType==2)
                  memcpy(pos, &currentPosition, sizeof(MotionDesc));

               createTrajectory(&notFilteredTraj, settings);
               // Если нужно, то проводим фильтрацию созданной траектории
               if(settings.relocType==2)
                   relocationTraj.loPassFilter(notFilteredTraj, pos, settings.FltCoeff);
               else
                  memcpy(pos, &notFilteredTraj, sizeof(MotionDesc));
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
         vectord zAxis={0.0,0.0,1.0};
         vectord xAxis{1.0,0.0,0.0};
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
                vectorsToAngles(xAxis, zAxis, zeroDir, trgtDir, nodeDir2);
                break;
            case 1:     // Решение из "передней" полусферы
                vectorsToAngles(xAxis, zAxis, zeroDir, trgtDir, nodeDir1);
                break;
            default:
                break;
         }

         // Определяем выбранные значения углово поворота ДПН, соответствующие индексу
         // выбранного узла
            pos->posK = alpha;
            pos->posT = beta;
      }

      void setCurrentPos(MotionDesc currPos)
      {
        double vk = (currPos.posK - currentPosition.posK) / TACT_H;
        double vt = (currPos.posT - currentPosition.posT) / TACT_H;
        memcpy(&previousPosition, &currentPosition, sizeof(MotionDesc));
        memcpy(&currentPosition, &currPos, sizeof(MotionDesc));
        currentPosition.velK = vk;
        currentPosition.velT = vt;
      }

      ModeDesc                      modeDesc;              // Уставки на текущий режим наведения
      MotionDesc                    currentPosition;
      MotionDesc                    previousPosition;
      RelocationTraj                relocationTraj;
      int                           step;                  ///< Текущее положение на траектории
     /* double                        _MAX_FORESEEN_TIME;     ///< Максммальное время прогноза вектора состояния и ориентации
      double                        _dpnQuatFrsStep;        ///< Шаг интегрирования кватерниона, с
      double                        _dpnVectFrsStep;        ///< Шаг интегрирования вектора состояния, с
      double                        _relocStartTime;         ///< Время начала переброса из начального положения на траектории движения*/


       double      deltaFi;      ///< Угол поворота платформы при движении с постоянной скоростью
   private:
            double      time;         ///< Текущее время на траектории движения
            double      stopTime;     ///< Время остановки платфомы / время выхода на максимальную скорость
            double      angle0;       ///< Начальный угол поворота платформы, от которого начинается отработка траектории
            double      angle;        ///< Текущий угол поворота платформы
       /*     double      velosity;     ///< Текущая скорость движения платформы
            double      velConst1;    ///< Постоянная скорость движения платформы в первом тесте
            double      velConst2;    ///< Постоянная скорость движения платформы во втором тесте
            double      velConst3;    ///< Постоянная скорость движения платформы в третьем тесте
         */ double      initTime;     ///< Время успокоения платформы после перехода в начальную точку траектории
            double      relaxTime;     ///< Время успокоения платформы при движении с постоянной скоростью
            double      waitTime;     ///< Задержка при измененнии знака ускорения при движении с переменной скоростью
          //  uint16_t    repeatNum;    ///< Кол-во повторений тестовой траектории
            double Alpha;
            double Beta;
   };

}

#endif //TRAJ_H
