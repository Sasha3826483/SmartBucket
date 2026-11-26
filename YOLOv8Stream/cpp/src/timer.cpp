#include "timer.hpp"

#include <algorithm>
#include <cmath>

bool Timer::setDurationSeconds(float seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0f) {
    // Некорректное значение не меняет текущую настройку таймера.
    return false;
  }

  durationSeconds = seconds;
  hasDuration = true;
  expired = false;
  return true;
}

void Timer::start() {
  startedAt = Clock::now();
  stoppedElapsedSeconds = 0.0f;
  running = true;
  expired = false;
}

void Timer::stop() {
  if (running) {
    // Сохраняем значение, чтобы elapsedSeconds() оставался неизменным.
    stoppedElapsedSeconds = elapsedSeconds();
    running = false;
  }
}

void Timer::reset() {
  stoppedElapsedSeconds = 0.0f;
  running = false;
  expired = false;
}

bool Timer::isRunning() {
  // Проверка состояния также останавливает таймер по истечении срока.
  isExpired();
  return running;
}

bool Timer::isExpired() {
  if (expired) {
    return true;
  }

  if (!hasDuration || !running || elapsedSeconds() < durationSeconds) {
    return false;
  }

  // Фиксируем истечение до следующего явного запуска.
  stop();
  expired = true;
  return true;
}

float Timer::restart() {
  const float seconds = elapsedSeconds();
  start();
  return seconds;
}

float Timer::elapsedSeconds() const {
  if (!running) {
    // Не учитываем время, прошедшее после остановки.
    return stoppedElapsedSeconds;
  }

  const auto now = Clock::now();
  const float seconds = std::chrono::duration<float>(now - startedAt).count();
  return std::max(seconds, 0.0f);
}
