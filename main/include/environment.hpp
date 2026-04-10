#pragma once

#include <atomic>

class Environment {
  public:
    explicit Environment() {
        static std::atomic<bool> created = false;
        bool expected = false;
        if (!created.compare_exchange_strong(expected, true)) {
            abort(); // We can remove this if we want multiple environments...
        }
    }
    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;

  private:
};
