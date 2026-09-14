#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd, 
                             float minOutput, float maxOutput, 
                             float maxIntegral) :
    kp{kp}, ki{ki}, kd{kd},
    minOutput{minOutput}, maxOutput{maxOutput}, maxIntegral{maxIntegral} {
}

void PIDController::setCoefficients(float kp, float ki, float kd) {
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void PIDController::setOutputLimits(float minVal, float maxVal) {
	this->minOutput = minVal;
	this->maxOutput = maxVal;
}

void PIDController::setIntegralLimit(float maxVal) {
	this->maxIntegral = maxVal;
}

// Предварительное объявление функции clampValue для использования в методе update
extern float clampValue(float value, float minValue, float maxValue);

float PIDController::update(float setpoint, float actual, float dt)
{
	targetSpeed = setpoint;
	measureSpeed = actual;

    float error = setpoint - actual;

    // При нулевой уставке и малой фактической скорости сбрасываем накопленный интеграл,
    // чтобы он не оставался навсегда после одного ненулевого значения.
//    if ((setpoint == 0.0f) && (actual >= -0.5f && actual <= 0.5f))
//    {
//        reset();
//        outputPid = 0.0f;
//        return 0.0f;
//    }

    // Вычисление компонентов PID
    float p = kp * error;
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;

    // Интегрирование с ограничением
    float newIntegral = clampValue(
        integral + error * dt,
        -maxIntegral,
        maxIntegral
    );

    // Не обновляем, если выход насыщен и интеграл толкает его ещё сильнее в сторону насыщения
    float output = p + ki * newIntegral + d;

    if (!((output > maxOutput && error > 0.0f) || (output < minOutput && error < 0.0f))) {
        integral = newIntegral;
    }

    // Пересчитываем выход с финальным интегралом и ограничиваем
    output = p + ki * integral + d;
    output = clampValue(output, minOutput, maxOutput);
    outputPid = output;
    
    prevError = error;

    return output;
}

float PIDController::getTargetSpeed() const {
	return targetSpeed;
}

float PIDController::getMeasureSpeed() const {
	return measureSpeed;
}

float PIDController::getOutputPid() const {
	return outputPid;
}

void PIDController::reset() {
	integral = 0.0f;
	prevError = 0.0f;
	outputPid = 0.0f;
}
