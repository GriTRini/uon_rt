#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace uon {
namespace timer {

template <typename Rep, typename Period> class Timer {
  public:
    Timer(const std::chrono::duration<Rep, Period> &interval,
          const std::function<void(void)> &function)
        : interval(interval), function(function), run(false) {}

    ~Timer() {
        if (is_running()) {
            stop();
        }
    }

  public:
    const std::chrono::duration<Rep, Period> interval;
    const std::function<void(void)> function;

  public:
    void start() noexcept {
        // atomic 변수의 안전한 상태 변경
        if (!run.exchange(true)) {
            thread = std::jthread{[this](std::stop_token st) {
                // 시작 기준 시간 측정
                auto next_time = std::chrono::steady_clock::now();
                
                while (!st.stop_requested()) {
                    // 🌟 다음 목표 시간 계산 (정확히 interval 만큼 누적)
                    next_time += interval; 
                    
                    function(); // 실제 로봇 업데이트 함수 실행
                    
                    // 🌟 목표 시간까지 남은 시간만 대기 (연산 시간 보상)
                    std::this_thread::sleep_until(next_time);
                }
            }};
        }
    }

    void stop() noexcept {
        if (run.exchange(false)) {
            thread.request_stop();
            // 스레드가 합류 가능한 상태인지 확인 후 join (에러 방지)
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    bool is_running() const noexcept { return run.load(); }

  private:
    std::jthread thread{};
    std::atomic<bool> run{false};
};

} // namespace timer
} // namespace uon