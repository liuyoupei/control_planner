#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>

class waitNotify {
private:
  waitNotify() {}

public:
  static std::shared_ptr<waitNotify> create();
  void wait();
private:
  std::mutex              m_mutex; /**< 锁*/
  std::condition_variable m_conditionVariable;
  bool                    m_isFinish{ false };
};
