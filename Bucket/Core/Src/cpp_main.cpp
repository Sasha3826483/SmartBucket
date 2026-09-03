#include "cpp_main.hpp"
#include "main.h"
#include "Motor.hpp"
#include "encoder.hpp"
#include "pid_controller.hpp"
#include <stdint.h>

// ---------------------------------------------------------
// Глобальные объекты для управления моторами, энкодерами и ПИД регуляторами

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

// ---------------------------------------------------------
// Константы и настройки
#define FRAME_START_1 0xAA
#define FRAME_START_2 0x55
#define FRAME_PAYLOAD_SIZE 3
#define COMMUNICATION_TIMEOUT_MS 300
#define MAX_SPEED 176

// ---------------------------------------------------------
// Глобальные переменные

// Состояния приема кадра
enum class FrameState : uint8_t {
	WaitStart1, // Ожидание первого стартового байта кадра синхрометки 0xAA55
	WaitStart2, // Ожидание второго стартового байта кадра синхрометки 0xAA55
	ReceivePayload, // Ожидание полезной нагрузки кадра
	WaitChecksum // Ожидание контрольной суммы кадра
};

// Переменные для UART приема данных
volatile uint8_t rxByte; // Переменная для хранения принятого байта по UART
volatile uint8_t framePayload[FRAME_PAYLOAD_SIZE]; // Буфер для хранения полезной нагрузки кадра
volatile uint8_t frameIndex{0}; // Индекс текущего байта в кадре
volatile FrameState frameState{FrameState::WaitStart1}; // Состояние приема кадра
volatile bool frameReady{false}; // Флаг готовности кадра
volatile uint32_t lastValidFrameTick{0}; // Тик последнего действительного кадра
volatile bool controlUpdate{false}; // Флаг обновления управления

// Переменные для управления скоростью и ПИД регулятора
static float controlDt{0.01f};

// ПИД-параметры для настройки ПИД регулятора через SWD
float Kp{0.0f}, Ki{0.0f}, Kd{0.0f};

// Ограничение интеграла для ПИД регулятора, чтобы избежать windup
float integralLimit{0.0f};

// Для отладки: текущая уставка скорости и выход ПИД регулятора и измеренных 
// оборотов для вывода на график через SWD-интерфейс
float setpointSpeed{0};
float outputPid{0};
float actualSpeed{0};

// Структура для хранения команды движения
struct MotionCommand {
	int16_t vx;
	int16_t vy;
	int16_t vz;
};

// Глобальные переменные для хранения текущей и ожидаемой команды движения
MotionCommand motion{}; // Текущая команда движения, которая применяется к моторам
volatile MotionCommand pendingMotion{}; // Ожидаемая команда движения, которая будет 
										// применена при следующем цикле управления

// Перечисление для определения позиции мотора
enum MotorPosition {
	MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB
};

// Массив объектов каждого мотора
Motor motors[] {
	Motor{&htim1, TIM_CHANNEL_4,
	AIN1_LF_GPIO_Port, AIN1_LF_Pin,
		AIN2_LF_GPIO_Port, AIN2_LF_Pin},

	Motor{&htim1, TIM_CHANNEL_3,
	AIN1_RF_GPIO_Port, AIN1_RF_Pin,
		AIN2_RF_GPIO_Port, AIN2_RF_Pin},

	Motor{&htim1, TIM_CHANNEL_2,
	AIN1_LB_GPIO_Port, AIN1_LB_Pin,
		AIN2_LB_GPIO_Port, AIN2_LB_Pin},

	Motor{&htim1, TIM_CHANNEL_1,
	AIN1_RB_GPIO_Port, AIN1_RB_Pin,
		AIN2_RB_GPIO_Port, AIN2_RB_Pin}
};

// Массив объектов каждого энкодера
Encoder encoders[] {
	Encoder{&htim5, 0xFFFFFFFFU}, // LF
	Encoder{&htim4, 0xFFFFU},     // RF
	Encoder{&htim3, 0xFFFFU},     // LB
	Encoder{&htim5, 0xFFFFFFFFU}  // RB
};

// Массив ПИД контроллеров для каждого мотора
PIDController pidControllers[4] {
	PIDController{0.0f, 0.0f, 0.0f, -100.0f, 100.0f, 100.0f},
	PIDController{0.0f, 0.0f, 0.0f, -100.0f, 100.0f, 100.0f},
	PIDController{0.0f, 0.0f, 0.0f, -100.0f, 100.0f, 100.0f},
	PIDController{0.0f, 0.0f, 0.0f, -100.0f, 100.0f, 100.0f}
};

// Функция для ограничения скорости в диапазоне [-100, 100] %
int16_t clampSpeed(int16_t speed) {
	if (speed > 100)
		return 100;
	if (speed < -100)
		return -100;
	return speed;
}

// Применение скорости к мотору с учетом направления
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

// Применение команды движения к роботу (vx, vy, vz) к каждому мотору с учетом ПИД регулятора
void applyMotion() {
	float measureSpeed[4]{};

	// Переводим дельты энкодеров в обороты в минуту.
	// Здесь 44 - количество импульсов на оборот, 56 - редуктор, 60 - перевод в минуты.
	measureSpeed[MOTOR_LF] = float(-encoders[MOTOR_LB].readDelta()) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RF] = float(encoders[MOTOR_RF].readDelta()) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_LB] = float(-encoders[MOTOR_LF].readDelta()) / controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RB] = float(encoders[MOTOR_RB].readDelta()) / controlDt / 44 / 56 * 60;

	int16_t targetSpeed[4]{};
	targetSpeed[MOTOR_LF] = motion.vy + motion.vx - motion.vz;
	targetSpeed[MOTOR_RF] = motion.vy - motion.vx + motion.vz;
	targetSpeed[MOTOR_LB] = motion.vy - motion.vx - motion.vz;
	targetSpeed[MOTOR_RB] = motion.vy + motion.vx + motion.vz;

	// Инициализируем уставочные значения скорости в об/мин
	float targetSpeedForPID[4]{};
	for (uint8_t i = 0; i < 4; ++i) {
		targetSpeedForPID[i] = (float(targetSpeed[i]) / 100) * MAX_SPEED;
	}

	float pidOuts[4]{};
	for (uint8_t i = 0; i < 4; ++i) {
		// Вычисляем выход ПИД регулятора и применяем его к мотору. Пид-регулятор работает в об/мин, 
		// поэтому нормируем его к диапазону [-100, 100], т.к. моторы управляются в процентах от 
		// максимальной скорости
		pidOuts[i] = (pidControllers[i].update(targetSpeedForPID[i], measureSpeed[i], controlDt) / MAX_SPEED) * 100;
		applyMotorSpeed(motors[i], pidOuts[i]);
	}
}

// ---------------------------------------------------------
// Прерывание для расчета скорости и ПИД регулятора
// ---------------------------------------------------------
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim10)
		controlUpdate = true;
}

// ---------------------------------------------------------
// UART callback
// ---------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart2)
		return;

	uint8_t byte = rxByte;
	static uint8_t checksum = 0;

	if (frameState == FrameState::WaitStart1) {
		if (byte == FRAME_START_1)
			frameState = FrameState::WaitStart2;
	} else if (frameState == FrameState::WaitStart2) {
		if (byte == FRAME_START_2) {
			frameState = FrameState::ReceivePayload;
			frameIndex = 0;
			checksum = 0;
		} else {
			frameState = FrameState::WaitStart1;
		}
	} else if (frameState == FrameState::ReceivePayload) {
		framePayload[frameIndex++] = byte;
		checksum ^= byte;
		if (frameIndex == FRAME_PAYLOAD_SIZE)
			frameState = FrameState::WaitChecksum;
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
		frameState = FrameState::WaitStart1;
	}

	// Продолжаем принимать следующий байт по UART
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

	// Запускаем таймер для работы ШИМ
	HAL_TIM_Base_Start_IT(&htim1);
	// Запускаем таймер для обработки данных с энкодера
	HAL_TIM_Base_Start_IT(&htim10);
	// Начинаем принимать данные по UART в прерывании
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxByte, 1);

	// Инициализируем тик последнего валидного кадра, чтобы избежать ложного срабатывания таймаута
	lastValidFrameTick = HAL_GetTick();

	uint32_t lastControlTick{HAL_GetTick()}; // Тик последнего обновления управления
	while (1) {
		bool updateRequired{false}; // Флаг, указывающий, что требуется обновление управления

		// Короткая критическая секция для обмена данными с прерываниями
		__disable_irq();
		// Если пришел новый кадр, применяем команду движения до обновления управления
		if (frameReady) {
			motion.vx = pendingMotion.vx;
			motion.vy = pendingMotion.vy;
			motion.vz = pendingMotion.vz;
			frameReady = false;
		}
		// Если пришло прерывание от таймера ПИД регулятора, устанавливаем флаг 
		// обновления управления (TIM10)
		if (controlUpdate) {
			controlUpdate = false;
			updateRequired = true;
		}
		__enable_irq();

		if (updateRequired) {
			uint32_t now{HAL_GetTick()};
			controlDt = float((now - lastControlTick) > 0U ? now - lastControlTick : 1U) / 1000.0f;
			lastControlTick = now;
			
			// Применяем команду движения к моторам с учетом ПИД регулятора
			applyMotion();
		}

		// Если прошло больше 300 мс с момента последнего валидного кадра, останавливаем все моторы,
		// обнуляем команды движения и сбрасываем ПИД регуляторы, чтобы избежать накопления 
		// интегральной ошибки
		if (HAL_GetTick() - lastValidFrameTick > COMMUNICATION_TIMEOUT_MS) {
			motion.vx = 0;
			motion.vy = 0;
			motion.vz = 0;
			pendingMotion.vx = 0;
			pendingMotion.vy = 0;
			pendingMotion.vz = 0;
			frameReady = false;
			for (uint8_t i = 0; i < 4; ++i) {
				// Останавливаем мотор и сбрасываем ПИД регулятор, чтобы избежать накопления 
				// интегральной ошибки
				motors[i].stop();
				pidControllers[i].reset();
			}
		}
		
		// -----------------------------------------
		// Настройка ПИД регулятора
		
		// Для настройки ПИД регулятора через SWD
		pidControllers[MOTOR_LF].setCoefficients(Kp, Ki, Kd);
		pidControllers[MOTOR_LF].setIntegralLimit(integralLimit);
		
		// Для отладки: сохраняем текущую уставку скорости и выход ПИД регулятора для вывода
		// на график через SWD-интерфейс
		setpointSpeed = pidControllers[MOTOR_LF].getTargetSpeed();
		outputPid = pidControllers[MOTOR_LF].getOutputPid();
		actualSpeed = pidControllers[MOTOR_LF].getMeasureSpeed();	
		
		// -----------------------------------------
	}
}
