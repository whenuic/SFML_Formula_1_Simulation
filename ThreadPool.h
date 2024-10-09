#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <functional>
#include <queue>
#include <condition_variable>
#include <mutex>

class ThreadPool {
  using Task = std::function<void()>;

 public:
  ThreadPool(int number_of_threads) { Start(number_of_threads); }

  ~ThreadPool() noexcept { Stop(); }

  void EnqueueTask(Task task) {
    {
      std::unique_lock<std::mutex> lock{event_mutex_};
      tasks_.emplace(std::move(task));
    }
    event_var_.notify_one();
  }

  void SetAvailableTasksCount(int n) {
    std::unique_lock<std::mutex> lock{event_mutex_};
    available_tasks_count_ = n;
  }

  int GetAvailableTasksCount() {
    std::unique_lock<std::mutex> lock{event_mutex_};
    return available_tasks_count_;
  }

 private:
  int available_tasks_count_ = 0;

  std::queue<Task> tasks_;
  std::vector<std::thread> threads_;

  std::condition_variable event_var_;

  std::mutex event_mutex_;

  bool is_stopped_ = false;

  void Start(int number_of_threads) {
    for (auto i = 0; i < number_of_threads; i++) {
      threads_.emplace_back([=] {
        while (true) {
          Task task;
          {
            std::unique_lock<std::mutex> lock{event_mutex_};
            event_var_.wait(lock,
                            [=] { return is_stopped_ || !tasks_.empty(); });
            if (is_stopped_ && tasks_.empty()) {
              break;
            }
            task = std::move(tasks_.front());
            tasks_.pop();
          }
          task();
          {
            std::unique_lock<std::mutex> lock{event_mutex_};
            available_tasks_count_--;
          }
        }
      });
    }
  }

  void Stop() {
    {
      std::unique_lock<std::mutex> lock{event_mutex_};
      is_stopped_ = true;
    }
    event_var_.notify_all();
    for (auto& t : threads_) {
      t.join();
    }
  }
};

#endif // THREAD_POOL_H