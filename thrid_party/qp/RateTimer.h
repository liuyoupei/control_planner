#pragma once
#include <chrono>
class RateTimer {
public:
  RateTimer(double hz) : m_rateHz(hz) {}
  RateTimer() {}
  bool sleep();
  void tic();
  long tac();

private:
  double                                m_rateHz;
  std::chrono::steady_clock::time_point m_timerPoint = std::chrono::steady_clock::now();
};
