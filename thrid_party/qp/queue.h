#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace qp {
template <class T> class Queue {
private:
  std::mutex                           m_mutex;
  std::condition_variable              m_cond;
  std::deque<T>                        m_list;
  std::atomic<decltype(m_list.size())> m_size{ 0 };

public:
  decltype(m_list.size()) size() {
    return m_size;
  }

  void pushBack(const T& item) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_list.push_back(item);
      m_size = m_list.size();
    }
    m_cond.notify_one();
  }

  void pushFront(const T& item) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_list.push_front(item);
      m_size = m_list.size();
    }
    m_cond.notify_one();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_list.empty()) {
      m_cond.wait(lock, [this] { return !m_list.empty(); });
    }

    auto val = m_list.front();
    m_list.pop_front();
    m_size = m_list.size();
    return val;
  }
};
}  // namespace qp
