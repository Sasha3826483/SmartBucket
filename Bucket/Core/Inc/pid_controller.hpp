#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <stdint.h>

class PIDController {
private:
    float kp;           // Пропорциональный коэффициент
    float ki;           // Интегральный коэффициент
    float kd;           // Дифференциальный коэффициент
    float integral;     // Накопленная интегральная ошибка
    float prevError;    // Предыдущая ошибка
    float minOutput;    // Минимальный выход
    float maxOutput;    // Максимальный выход
    float maxIntegral;  // Ограничение для интеграла (anti-windup)
    float targetSpeed;  // Уставка скорости для ПИД регулятора
    float measureSpeed; // Измеренная скорость для ПИД регулятора
    float outputPid;       // Выход ПИД регулятора

public:
    // Конструктор
    PIDController(float kp = 0.5f, float ki = 0.05f, float kd = 0.02f);
    
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

    int getTargetSpeed() const;
    int getOutputPid() const;
};

#endif // PID_CONTROLLER_HPP
