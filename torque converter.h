#ifndef __TC
#define __TC
class TC
{
public:
  enum status { RESET, UNLOCK, LOCK };
  TC()                     = default;
  TC(const TC&)            = delete;
  TC(TC&&)                 = delete;
  TC& operator=(const TC&) = delete;
  TC& operator=(TC&&)      = delete;
  ~TC()                    = default;
  void lock() const;
  void unlock() const;
  void reset() const;
  status state() const;
private:
  mutable status st = RESET;
  const uint16_t minpwm = 0;
  const uint16_t maxpwm = 500;
};

inline void TC::lock() const
{
  TIM_SetCompare3(TIM1, minpwm);//ТР
  TIM_SetCompare4(TIM2, maxpwm);//ФМ
}
inline void TC::unlock() const
{
  TIM_SetCompare4(TIM2, minpwm);
  TIM_SetCompare3(TIM1, maxpwm);
}
inline void TC::reset() const
{
  TIM_SetCompare3(TIM1, minpwm);
  TIM_SetCompare4(TIM2, minpwm);
}
inline TC::status TC::state() const { return st; }
#endif //__TC