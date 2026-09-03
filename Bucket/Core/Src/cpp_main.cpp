#include "cpp_main.hpp"
#include "main.h"
#include "Motor.hpp"
#include "encoder.hpp"
#include "pid_controller.hpp"

#include <stdint.h>

// PWM Timer
extern TIM_HandleTypeDef htim1;

// PID Timer
extern TIM_HandleTypeDef htim10;

// Encoders Timers (32 bit)
//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

// Encoders Timers (16 bit)
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// UART Timer
extern UART_HandleTypeDef huart2;

#define FRAME_START_1 0xAA
#define FRAME_START_2 0x55
#define FRAME_PAYLOAD_SIZE 3
#define COMMUNICATION_TIMEOUT_MS 300
#define MAX_SPEED 176

volatile uint8_t rxByte;
volatile uint8_t framePayload[FRAME_PAYLOAD_SIZE];
volatile uint8_t frameIndex = 0;
volatile uint8_t frameState = 0;
volatile bool frameReady = false;
volatile uint32_t lastValidFrameTick = 0;
static uint32_t lastControlTick = 0;

static float controlDt = 0.01f;
static constexpr float CONTROL_DT_FALLBACK = 0.01f;

// для настройки ПИД регулятора через SWD
float Kp, Ki, Kd;
int16_t setpointSpeed = 0;
int16_t outputPid = 0;
int integralLimit = 0;

// Структура для хранения команды движения
struct MotionCommand {
	int16_t vx;
	int16_t vy;
	int16_t vz;
};

MotionCommand motion { };
volatile MotionCommand pendingMotion { };

enum MotorPosition {
	MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB
};

Motor motors[] = { 
	Motor(&htim1, TIM_CHANNEL_4,
	AIN1_LF_GPIO_Port, AIN1_LF_Pin,
	AIN2_LF_GPIO_Port, AIN2_LF_Pin),

	Motor(&htim1, TIM_CHANNEL_3,
	AIN1_RF_GPIO_Port, AIN1_RF_Pin,
	AIN2_RF_GPIO_Port, AIN2_RF_Pin),

	Motor(&htim1, TIM_CHANNEL_2,
	AIN1_LB_GPIO_Port, AIN1_LB_Pin,
	AIN2_LB_GPIO_Port, AIN2_LB_Pin),

	Motor(&htim1, TIM_CHANNEL_1,
	AIN1_RB_GPIO_Port, AIN1_RB_Pin,
	AIN2_RB_GPIO_Port, AIN2_RB_Pin) 
};

Encoder encoders[] = { 
	Encoder(&htim5, 0xFFFFFFFFU), // LF
	Encoder(&htim4, 0xFFFFU),     // RF
	Encoder(&htim3, 0xFFFFU),     // LB
	Encoder(&htim5, 0xFFFFFFFFU)  // RB
};

// Массив ПИД контроллеров для каждого мотора
PIDController pidControllers[4] = { 
	PIDController(0.0f, 0.0f, 0.0f),
	PIDController(0.0f, 0.0f, 0.0f),
	PIDController(0.0f, 0.0f, 0.0f),
	PIDController(0.0f, 0.0f, 0.0f)
};

volatile float measureSpeed[4] = { };
int16_t targetSpeed[4] = { };

int16_t clampSpeed(int16_t speed) {
	if (speed > 100)
		return 100;
	if (speed < -100)
		return -100;
	return speed;
}

void applyMotorSpeed(Motor &motor, int16_t speed) {
	speed = clampSpeed(speed);
	if (speed > 0) {
		motor.setDirection(MotorDirection::FORWARD);
		motor.setSpeed((uint8_t) speed);
	} else if (speed < 0) {
		motor.setDirection(MotorDirection::BACKWARD);
		motor.setSpeed((uint8_t) -speed);
	} else {
		motor.stop();
	}
}

void applyMotion(const MotionCommand &motion) {
	int16_t targetSpeed[4] = { };
	targetSpeed[MOTOR_LF] = motion.vy + motion.vx - motion.vz;
	targetSpeed[MOTOR_RF] = motion.vy - motion.vx + motion.vz;
	targetSpeed[MOTOR_LB] = motion.vy - motion.vx - motion.vz;
	targetSpeed[MOTOR_RB] = motion.vy + motion.vx + motion.vz;

	// Инициализируем уставочные значения скорости в об/мин
	float targetSpeedForPID[4] = { };
	for (int i = 0; i < 4; ++i) {
		targetSpeedForPID[i] = (float(targetSpeed[i]) / 100) * MAX_SPEED;
	}

	float pidOuts[4] = { };
	for (int i = 0; i < 4; ++i) {
		pidOuts[i] = (pidControllers[i].update(targetSpeedForPID[i], measureSpeed[i], controlDt) / MAX_SPEED) * 100;
		applyMotorSpeed(motors[i], pidOuts[i]);
	}
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// TIM10 настроен на прерывание каждые 10 мс, используем его для расчета скорости и ПИД регулятора
	if (htim != &htim10)
		return;

	// Вычисляем дельту времени с момента последнего вызова для более точного расчета скорости И ПИД регулятора
	uint32_t now = HAL_GetTick();
	uint32_t elapsedMs = (lastControlTick == 0U) ? 10U : (now - lastControlTick);
	controlDt = (elapsedMs == 0U) ? CONTROL_DT_FALLBACK : (float(elapsedMs) / 1000.0f);
	lastControlTick = now;

	// Считываем дельты энкодеров
	int32_t encoderDeltas[4] = { };
	encoderDeltas[MOTOR_LB] = -encoders[MOTOR_LB].readDelta();
	encoderDeltas[MOTOR_RB] = encoders[MOTOR_RB].readDelta();
	encoderDeltas[MOTOR_LF] = -encoders[MOTOR_LF].readDelta();
	encoderDeltas[MOTOR_RF] = encoders[MOTOR_RF].readDelta();

	// Переводим дельты в об/мин (RPM)
	// Здесь 44 - количество импульсов на оборот (режим работы энкодера TI1 AND TI2), 56 - редуктор
	measureSpeed[MOTOR_LF] = float(encoderDeltas[MOTOR_LB]) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RF] = float(encoderDeltas[MOTOR_RF]) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_LB] = float(encoderDeltas[MOTOR_LF]) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RB] = float(encoderDeltas[MOTOR_RB]) / controlDt / 44 / 56 * 60;

	setpointSpeed = pidControllers[MOTOR_LF].getTargetSpeed();
	outputPid = pidControllers[MOTOR_LF].getOutputPid();

	applyMotion(motion);
}

// ---------------------------------------------------------
void startUartRx(void) {
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxByte, 1);
}

// ---------------------------------------------------------
// UART callback
// ---------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart2)
		return;

	uint8_t byte = rxByte;
	static uint8_t checksum = 0;

	if (frameState == 0) {
		if (byte == FRAME_START_1)
			frameState = 1;
	} else if (frameState == 1) {
		if (byte == FRAME_START_2) {
			frameState = 2;
			frameIndex = 0;
			checksum = 0;
		} else {
			frameState = 0;
		}
	} else if (frameState == 2) {
		framePayload[frameIndex++] = byte;
		checksum ^= byte;
		if (frameIndex == FRAME_PAYLOAD_SIZE)
			frameState = 3;
	} else {
		if (byte == checksum) {
			lastValidFrameTick = HAL_GetTick();
			if (!frameReady) {
				pendingMotion.vx = (int8_t) framePayload[0];
				pendingMotion.vy = (int8_t) framePayload[1];
				pendingMotion.vz = (int8_t) framePayload[2];
				frameReady = true;
			}
		}
		frameState = 0;
	}

	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxByte, 1);
}

// ---------------------------------------------------------
// Главная функция
// ---------------------------------------------------------

void cpp_main(void) {
	// Инициализация моторов и энкодеров
	for (uint8_t i = 0; i < 4; ++i) {
		motors[i].init();
		encoders[i].init();
	}

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim10);
	lastValidFrameTick = HAL_GetTick();

	startUartRx();

	for (int i = 0; i < 4; ++i) {
		pidControllers[i].PIDController::setOutputLimits(-MAX_SPEED, MAX_SPEED);
	}

	while (1) {

		// Если прошло больше 300 мс с момента последнего валидного кадра, останавливаем все моторы
		if (HAL_GetTick() - lastValidFrameTick > COMMUNICATION_TIMEOUT_MS) {
			motion.vx = 0;
			motion.vy = 0;
			motion.vz = 0;
			pendingMotion.vx = 0;
			pendingMotion.vy = 0;
			pendingMotion.vz = 0;
			frameReady = false;
			for (uint8_t i = 0; i < 4; ++i) {
				motors[i].stop();
				pidControllers[i].reset();
			}
		}
		
		// Если пришел новый кадр, применяем команду движения
		if (frameReady) {
			// Копируем данные из pendingMotion в motion с отключением прерываний
			__disable_irq();
			motion.vx = pendingMotion.vx;
			motion.vy = pendingMotion.vy;
			motion.vz = pendingMotion.vz;
			frameReady = false;
			__enable_irq();
		}
		
		// Для настройки ПИД регулятора через SWD
		pidControllers[MOTOR_LF].setCoefficients(Kp, Ki, Kd);
		pidControllers[MOTOR_LF].setIntegralLimit(integralLimit);
	}
}
