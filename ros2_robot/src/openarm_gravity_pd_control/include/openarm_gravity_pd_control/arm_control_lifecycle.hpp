#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace openarm_gravity_pd_control {

// One worker owns each arm from initialization through continuous control.
// A coordinator joins ALL workers before disabling either bus. It also handles
// worker exceptions without depending on a functioning ROS executor.
class ArmControlLifecycle {
public:
  using Continue = std::function<bool()>;
  struct Arm {
    std::string name;
    std::function<bool(const Continue &)> initialize;
    std::function<void()> step;
    std::function<void()> disable;
  };

  ArmControlLifecycle() = default;
  ArmControlLifecycle(const ArmControlLifecycle &) = delete;
  ArmControlLifecycle & operator=(const ArmControlLifecycle &) = delete;
  ~ArmControlLifecycle() { stop(); }

  void start(std::vector<Arm> arms, std::chrono::steady_clock::duration period,
    Continue context_ok, std::function<void(std::exception_ptr)> report_error = {})
  {
    if (started_) throw std::logic_error("arm workers already started");
    if (period <= std::chrono::steady_clock::duration::zero())
      throw std::invalid_argument("control period must be positive");
    arms_ = std::move(arms);
    period_ = period;
    context_ok_ = std::move(context_ok);
    report_error_ = std::move(report_error);
    started_ = true;
    coordinator_ = std::thread([this]() { coordinate(); });
  }

  void waitInitialized()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this]() {return ready_ == arms_.size() || error_ || stop_.load();});
    if (error_) std::rethrow_exception(error_);
    if (stop_.load()) throw std::runtime_error("arm initialization cancelled");
  }

  bool stopping() const noexcept {return stop_.load();}

  // Idempotent and safe when a shutdown callback races normal destruction.
  // Never call from initialize/step/disable/report_error callbacks.
  void stop() noexcept
  {
    std::lock_guard<std::mutex> join_lock(join_mutex_);
    requestStop();
    if (coordinator_.joinable()) coordinator_.join();
  }

private:
  void requestStop() noexcept
  {
    stop_.store(true);
    changed_.notify_all();
  }

  bool continuing()
  {
    if (stop_.load()) return false;
    if (!context_ok_()) {requestStop(); return false;}
    return !stop_.load();
  }

  void fail(std::exception_ptr error) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!error_) error_ = error;
    }
    requestStop();
  }

  void run(Arm & arm) noexcept
  {
    try {
      const Continue keep_running = [this]() {return continuing();};
      if (!keep_running()) return;
      if (!arm.initialize(keep_running)) {
        if (keep_running()) throw std::runtime_error(arm.name + " arm initialization failed");
        return;
      }
      // A peer may have failed or shutdown may have begun during homing.
      if (!keep_running()) return;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++ready_;
      }
      changed_.notify_all();
      auto next = std::chrono::steady_clock::now();
      while (keep_running()) {
        arm.step();
        next += period_;
        const auto now = std::chrono::steady_clock::now();
        if (next < now) next = now;  // Preserve the existing no-catch-up policy.
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_until(lock, next, [this]() {return stop_.load();});
      }
    } catch (...) {
      fail(std::current_exception());
    }
  }

  void coordinate() noexcept
  {
    std::vector<std::thread> workers;
    try {
      workers.reserve(arms_.size());
      for (auto & arm : arms_) {
        if (!continuing()) break;
        auto * current = &arm;
        workers.emplace_back([this, current]() {run(*current);});
      }
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait(lock, [this]() {return stop_.load();});
    } catch (...) {
      fail(std::current_exception());
    }
    requestStop();
    for (auto & worker : workers) if (worker.joinable()) worker.join();
    for (auto & arm : arms_) {
      try {arm.disable();} catch (...) {fail(std::current_exception());}
    }
    std::exception_ptr error;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error = error_;
    }
    if (error && report_error_) {
      try {report_error_(error);} catch (...) {}  // Never unwind a thread entry.
    }
  }

  std::vector<Arm> arms_;
  std::chrono::steady_clock::duration period_{};
  Continue context_ok_;
  std::function<void(std::exception_ptr)> report_error_;
  std::atomic<bool> stop_{false};
  bool started_ = false;
  std::size_t ready_ = 0;
  std::exception_ptr error_;
  std::mutex mutex_, join_mutex_;
  std::condition_variable changed_;
  std::thread coordinator_;
};
}  // namespace openarm_gravity_pd_control
