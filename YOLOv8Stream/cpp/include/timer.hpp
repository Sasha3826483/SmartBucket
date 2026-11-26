#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

class Timer {
public:
  using Clock = std::chrono::steady_clock;

  Timer() = default;

  // Задаёт одноразовый таймаут в секундах.
  // Возвращает false для отрицательного или не конечного значения.
  bool setDurationSeconds(float seconds);
  // Начинает новый отсчёт с нуля.
  void start();
  // Останавливает отсчёт, сохраняя прошедшее время.
  void stop();
  // Останавливает таймер и очищает прошедшее время, сохраняя заданный таймаут.
  void reset();
  // Проверяет истечение таймаута перед возвратом состояния.
  bool isRunning();
  // При истечении останавливает таймер до следующего start().
  bool isExpired();
  // Возвращает прошедшее время и начинает новый отсчёт.
  float restart();
  // Возвращает прошедшее время; не останавливает и не сбрасывает таймер.
  float elapsedSeconds() const;

private:
  Clock::time_point startedAt{}; // Время последнего запуска таймера
  float durationSeconds{0.0f}; // Длительность таймаута в секундах
  float stoppedElapsedSeconds{0.0f}; // Прошедшее время при остановке таймера
  bool hasDuration{false}; // true, если таймер имеет заданный таймаут
  bool running{false}; // true, если таймер запущен и отсчитывает время
  bool expired{false}; // true, если таймер истёк и не был перезапущен
};

#endif // TIMER_HPP
