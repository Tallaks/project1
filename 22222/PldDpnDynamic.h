#ifndef ARM_DEBUG
   #include <boost/lexical_cast.hpp>
   #include "Mpc/Core/Utils/Sudn/SudnUtils.h"
   #include "Target/SubSystems/Payloads/PldDpn/mpsrc/PldDpnDynamic.h"

#else
    #include "../../techno/exchange/model_bort.h"
    #include "PldDpnDynamic.h"
#endif

namespace Target { namespace SubSystems { namespace Payloads { namespace PldDpn
{
   /// Функция для начальной инициализации установочного кватерниона 
  void DpnDynamic::initSettings(Settings * sets/*, const double * deltaR, const double * visV*/)
   {
      _settings.initRelocParams(sets);    //инициализируем параметры для переброса
      _settings.initBetaProfileParams(sets);    //инициализируем параметры для ступенчатой траеткории по бета
      // Вычисление кватерниона установки телескопа относительно подвижной системы ДПН
      double Qt1[4];
      memcpy(Qt1, sets->quaternions()->telQuaternion, 4 * sizeof(double));
      _settings._qTelescope = SudnLib::Quaternion(Qt1);
      _settings._qTelescope = ~_settings._qTelescope;

      // Переход от системы координат телескопа к подвижной с.к. ДПН
      double Telescope[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.}; // Матрица перехода от телескопа к подвижной ДПН
      double fullDPNZone[2] = {-175.0, 175.0};
      _settings._qTelescope.toMatrix(Telescope);
      SudnLib::Vector vVis(sets->visVector());                   // Телескоп этой осью (OY) смотрит на объект
      _settings._vTelB = vVis;
      _settings._vTel = vVis.mulToMatR(Telescope);
      SudnLib::Vector vDpnVis(0.0,0.0,1.0);
      DpnMotionDesc tmpPos;
      if(!ITrajectory::directionToAngles(&_settings._telPos, _settings._vTel, vDpnVis, sets->settings()->flags.w.solution,fullDPNZone,fullDPNZone)) {};

      //_settings._telPos._posB -= 175.0;

      // Вычисление установочного кватерниона ДПН
      SudnLib::Quaternion Qd1(
         cos(grad2rad*(175.0/2.0)) * cos(grad2rad*(175.0/2.0)),
         -cos(grad2rad*(175.0/2.0)) * sin(grad2rad*(175.0/2.0)),
         -sin(grad2rad*(175.0/2.0)) * cos(grad2rad*(175.0/2.0)),
         -sin(grad2rad*(175.0/2.0)) * sin(grad2rad*(175.0/2.0))
         );  // Разворот ДПН подвижной, относительно связанных осей ДПН
//      SudnLib::Quaternion Qd2(0.0, 0.0, sqrt(2.0)/2.0, -sqrt(2.0)/2.0);                          // Переход от ССК СМ к СК ДПН
      memcpy(Qt1, sets->quaternions()->dpnQuaternion, 4 * sizeof(double));
      SudnLib::Quaternion Qd2(Qt1);
      _settings._qSm2Dpn = Qd2;
      _settings._qSm2DpnP0 = Qd2 * Qd1;
      //_settings._qDpn0 = Qd2;

      // Запоминаем разрешенные зоны углов ДПН и настройку, отвечающую за выбор решения
      _settings._aZone[0] = sets->settings()->minA;
      _settings._aZone[1] = sets->settings()->maxA;
      _settings._bZone[0] = sets->settings()->minB;
      _settings._bZone[1] = sets->settings()->maxB;
      _settings._solChoise = sets->settings()->flags.w.solution;
      double deltaR[2] = {0};
      memcpy(deltaR, sets->deltaR(), 2 * sizeof(double));

      // Вычисляем кватернион доворота при наблюдении за Луной или Солнцем
      SudnLib::Quaternion dqSB1(
                                cos(grad2rad*(deltaR[0]/2.0)),
                                0.0,
                                0.0,
                                sin(grad2rad*(deltaR[0]/2.0))),
                          dqSB2(
                                cos(grad2rad*(deltaR[1]/2.0)),
                                0.0,
                                sin(grad2rad*(deltaR[1]/2.0)),
                                0.0);
      _settings._qCnt2Prf = dqSB1*dqSB2;

      // Коэффициенты фильтра низких частот
      if(sets->settings()->lpFltCoeffA>0.00001 && sets->settings()->lpFltCoeffA<=1.0)
      {
        _settings._lpFltCoeff[0] =  sets->settings()->lpFltCoeffA;
        _settings._lpFltCoeff[1] =  sets->settings()->lpFltCoeffB;
        _settings._lpfTimeConstant = TACT_H*0.5*(1.0-_settings._lpFltCoeff[0])/_settings._lpFltCoeff[0];
      }
      // Тип переходного процесса
      _settings._relocType = sets->settings()->flags.w.relocationType;
      // Значение дробной части липсекунд
      _settings._leapSecFrac = 0.0;

      // Параметры использования Ступенчатого фильтра для оси Бета
      _settings._useBetaStepFilter = sets->settings()->flags.w.useBetaStepFilter;
      _settings._betaFilterStep = sets->settings()->betaFilterStep;
      _settings._betaFilterAcc = sets->settings()->betaFilterAcc;
   }

   ITrajectory* DpnDynamic::getTrajectory(DpnDynaModes mode)
   {
      switch(mode)
      {
      case dynModeConst: return &_trajConst;
      case dynModeEarth: return &_trajEarth;
      case dynModeStars: return &_trajStars;
      case dynModeMoon:  return &_trajMoon;
      case dynModeSun:   return &_trajSun;
      case dynModeTest:  return &_trajTest;
      default:           return 0;
      }
      return 0;
   }

   DpnDynaErrors DpnDynamic::selectTrajectory(const DpnModeDesc& data)
   {
      _trajectory = getTrajectory(data.md);
      _trajectory->zeroCounter();
      if (!_trajectory)
         return dynErrMode;
      _trajectory->setBnoData(_bnoClient);

      return _trajectory->init(data);
   }

   bool DpnDynamic::calcTraj(const DpnMotionDesc &currPos, DpnMotionDesc *pos, DpnDynError & err, double &T)
   {
      if (!_trajectory)
         return false;
      _trajectory->setBnoData(_bnoClient);
      _trajectory->setCurrentPos(currPos);
      if(_settings._relocType>0)
        if(_trajectory->getCounter() < 3)
        {
            _trajectory->increaseCounter();
            memcpy(pos,&currPos, sizeof(DpnMotionDesc));
            pos->velA = 0.0;
            pos->velB = 0.0;
            _trajectory->zeroLpfParams(currPos);
            T = -1;
            return true;
        }
      T = 0.0;
      return _trajectory->calculate(_settings, pos, T, err);
   }

   bool DpnDynamic::calcPos(DpnDynError & err, double Alpha, double Beta, DpnModeDesc& data)
   {
      if (!_trajectory)
         return false;
      _trajectory->setBnoData(_bnoClient);
      data.md = _trajectory->mode();
      return _trajectory->currentDirection(_settings, Alpha, Beta, err, data);
   }

   void DpnDynamic::timeDiff(DpnDynError & err) {
      if(!_trajectory) {
         err.set(DpnDynError::DpnNoTraj);
         return;
      }
      _trajectory->setBnoData(_bnoClient);
      return _trajectory->timeDiff(err);
   }
}}}}

























