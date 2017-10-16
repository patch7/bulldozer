#ifndef __CALIBRATE
#define __CALIBRATE


class Calibrate
{
public:
  Calibrate()                            = default;
  Calibrate(const Calibrate&)            = delete;
  Calibrate(Calibrate&&)                 = delete;
  Calibrate& operator=(const Calibrate&) = delete;
  Calibrate& operator=(Calibrate&&)      = delete;
  ~Calibrate()                           = default;

  void Write();
  void Read();
  void RudCalib();

private:
  uint16_t TableOtL[100] = {0};
  uint16_t TableOtR[100] = {0};
  uint16_t TableBfL[100] = {0};
  uint16_t TableBfR[100] = {0};
  uint16_t TableF[100]   = {0};
  uint16_t TableR[100]   = {0};

  uint16_t RudMin        = 0;
  uint16_t RudMax        = 0;
  uint16_t LeftMin       = 0;
  uint16_t LeftMax       = 0;
  uint16_t RightMin      = 0;
  uint16_t RightMax      = 0;
  uint16_t BrakeMin      = 0;
  uint16_t BrakeMax      = 0;
  uint16_t DecelerateMin = 0;
  uint16_t DecelerateMax = 0;

  //               Resv Resv Resv Resv Resv Decl  B   R   L  Rud  R   F  BfR BfL OtR OtL
  uint16_t state;// 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
};


#endif  __CALIBRATE