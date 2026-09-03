#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd) :
		kp(kp), ki(ki), kd(kd), integral(0.0f), prevError(0.0f), minOutput(
				-100.0f), maxOutput(100.0f), maxIntegral(50.0f) {
}

void PIDController::setCoefficients(float kp, float ki, float kd) {
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void PIDController::setOutputLimits(float minVal, float maxVal) {
	minOutput = minVal;
	maxOutput = maxVal;
}

void PIDController::setIntegralLimit(float maxVal) {
	maxIntegral = maxVal;
}

float clamp(float value, float minVal, float maxVal) {
	if (value > maxVal)
		return maxVal;
	if (value < minVal)
		return minVal;
	return value;
}

float PIDController::update(float setpoint, float actual, float dt)
{
	targetSpeed = setpoint;
	measureSpeed = actual;

    float error = setpoint - actual;

    // Остановка двигателя при близости к нулю
    if (setpoint == 0.0f && (actual >= -1.0f && actual <= 1.0f))
    {
        integral = 0.0f;
        prevError = 0.0f;
        return 0.0f;
    }

    // Вычисление компонентов PID
    float p = kp * error;
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;

    // Интегрирование с ограничением
    float newIntegral = clamp(
        integral + error * dt,
        -maxIntegral,
        maxIntegral
    );

    // Anti-windup: обновляем интеграл только если это помогает
    // Не обновляем, если выход насыщен и интеграл толкает его ещё сильнее в сторону насыщения
    float output = p + ki * newIntegral + d;

    if (!((output > maxOutput && error > 0.0f) || (output < minOutput && error < 0.0f))) {
        integral = newIntegral;
    }

    // Пересчитываем выход с финальным интегралом и ограничиваем
    output = p + ki * integral + d;
    output = clamp(output, minOutput, maxOutput);

    prevError = error;
    outputPid = output;

    return output;
}

int PIDController::getTargetSpeed() const {
	return static_cast<int>(targetSpeed);
}

int PIDController::getOutputPid() const {
	return static_cast<int>(outputPid);
}

void PIDController::reset() {
	integral = 0.0f;
	prevError = 0.0f;
}
