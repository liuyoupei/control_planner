#include "waitNotify.h"
#include <iostream>

void waitNotify::wait() {
  std::unique_lock<std::mutex> ul(m_mutex);
  auto ret = m_conditionVariable.wait_for(ul, std::chrono::seconds{ 1 }, [&] { return m_isFinish; });
  if (!ret) {
  }
  m_isFinish = false;
}
std::shared_ptr<waitNotify> waitNotify::create() {
  return std::shared_ptr<waitNotify>(new waitNotify());
}
