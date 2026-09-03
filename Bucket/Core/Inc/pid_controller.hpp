#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <stdint.h>

class PIDController {
private:
    float kp;           // Пропорциональный коэффициент
    float ki;           // Интегральный коэффициент
    float kd;           // Дифференциальный коэффициент

    float integral{0.0f};     // Накопленная интегральная ошибка
    float prevError{0.0f};    // Предыдущая ошибка для вычисления производной

    float minOutput;    // Минимальный выход
    float maxOutput;    // Максимальный выход

    float maxIntegral;  // Ограничение для интеграла (anti-windup)

    // Отладочные переменные для хранения текущей уставки, обратной связи и выхода ПИД регулятора
    float targetSpeed{0.0f};  // Уставка скорости для ПИД регулятора
    float measureSpeed{0.0f}; // Измеренная скорость для ПИД регулятора
    float outputPid{0.0f};    // Выход ПИД регулятора

public:
    // Конструктор
    PIDController(float kp = 1.0f,
                  float ki = 0.0f,
                  float kd = 0.0f,
                  float minOutput = -1.0f,
                  float maxOutput = 1.0f,
                  float maxIntegral = 100.0f);
    
    // Установить коэффициенты
    void setCoefficients(float kp, float ki, float kd);
    
    // Установить ограничения выхода
    void setOutputLimits(float minVal, float maxVal);
    
    // Установить ограничение интеграла (anti-windup)
    void setIntegralLimit(float maxVal);
    
    // Обновить значение ПИД регулятора
    float update(float setpoint, float actual, float dt);
    
    // Сбросить регулятор
    void reset();

    // Получить текущую уставку скорости
    float getTargetSpeed() const;

    // Получить текущее выдаваемое значение ПИД регулятора
    float getOutputPid() const;

    // Получить текущее измеренное значение скорости
    float getMeasureSpeed() const;
};

#endif // PID_CONTROLLER_HPP
