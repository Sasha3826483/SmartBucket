#include "cpp_main.hpp"
#include "main.h"
#include "Motor.hpp"
#include "encoder.hpp"
#include "pid_controller.hpp"
#include "filter.hpp"
#include <stdint.h>

// ---------------------------------------------------------
// Глобальные объекты для управления моторами, энкодерами и ПИД регуляторами

// PWM Timer
extern TIM_HandleTypeDef htim1;

// PID Timer
extern TIM_HandleTypeDef htim10;

// Encoders Timers (32 bit)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

// Encoders Timers (16 bit)
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// UART Timer
extern UART_HandleTypeDef huart2;

// ADC
extern ADC_HandleTypeDef hadc1;

// ---------------------------------------------------------
// Константы и настройки
#define g_FRAME_START_1 0xAA
#define g_FRAME_START_2 0x55
#define g_FRAME_PAYLOAD_SIZE 3
#define g_FRAME_TYPE_TELEMETRY 0x20
#define g_FRAME_TELEMETRY_PAYLOAD_SIZE 9
#define g_TELEMETRY_FRAME_SIZE 14
#define g_COMMUNICATION_TIMEOUT_MS 300
#define g_MAX_SPEED 176
#define g_MAX_MEASURED_SPEED 300.0f

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
volatile uint8_t g_rxByte; // Переменная для хранения принятого байта по UART
volatile uint8_t g_framePayload[g_FRAME_PAYLOAD_SIZE]; // Буфер для хранения полезной нагрузки кадра
volatile uint8_t g_frameIndex{0}; // Индекс текущего байта в кадре
volatile FrameState g_frameState{FrameState::WaitStart1}; // Состояние приема кадра
volatile bool g_frameReady{false}; // Флаг готовности кадра
volatile uint32_t g_lastValidFrameTick{0}; // Тик последнего действительного кадра
volatile bool g_controlUpdate{false}; // Флаг обновления управления
volatile bool g_telemetryUpdate{false};
volatile bool g_resetPid{false}; // Флаг сброса ПИД регулятора

// Переменные для управления скоростью и ПИД регулятора
static float g_controlDt{0.01f};

MovingAverageFilter<10> g_voltageAverageFilter;
IFilter* g_VoltageFilter = &g_voltageAverageFilter;

MedianFilter<3> g_speedMedianFilters[4];
IFilter* g_speedFilters[4] {
	&g_speedMedianFilters[0],
	&g_speedMedianFilters[1],
	&g_speedMedianFilters[2],
	&g_speedMedianFilters[3]
};

// ПИД-параметры для настройки ПИД регулятора через SWD
float g_Kp{1.0f}, g_Ki{15.0f}, g_Kd{0.01f};

// Ограничение интеграла для ПИД регулятора, чтобы избежать windup
float g_integralLimit{0.0f};

// Для отладки: текущая уставка скорости и выход ПИД регулятора и измеренных 
// оборотов для вывода на график через SWD-интерфейс
float g_setpointSpeed{0};
float g_outputPid{0};
float g_actualSpeed{0};

int32_t g_dt{0};

// Структура для хранения команды движения
struct MotionCommand {
	int16_t vx;
	int16_t vy;
	int16_t vz;
};

// Глобальные переменные для хранения текущей и ожидаемой команды движения
MotionCommand g_motion{}; // Текущая команда движения, которая применяется к моторам
volatile MotionCommand g_pendingMotion{}; // Ожидаемая команда движения, которая будет 

// Перечисление для определения позиции мотора
enum MotorPosition {
	MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB
};

// Массив объектов каждого мотора
Motor g_motors[] {
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
Encoder g_encoders[] {
	Encoder{&htim3, 0xFFFFU}, // LF
	Encoder{&htim5, 0xFFFFFFFFU},     // RF
	Encoder{&htim2, 0xFFFFFFFFU},     // LB
	Encoder{&htim4, 0xFFFFU}  // RB
};

// Массив ПИД контроллеров для каждого мотора
PIDController g_pidControllers[4] {
	 PIDController{g_Kp, g_Ki, g_Kd, -176.0f, 176.0f, 100.0f},
	 PIDController{g_Kp, g_Ki, g_Kd, -176.0f, 176.0f, 100.0f},
	 PIDController{g_Kp, g_Ki, g_Kd, -176.0f, 176.0f, 100.0f},
	 PIDController{g_Kp, g_Ki, g_Kd, -176.0f, 176.0f, 100.0f}
};

// Функция для ограничения значения в заданном диапазоне
float clampValue(float value, float minValue, float maxValue) {
	if (value > maxValue)
		return maxValue;
	if (value < minValue)
		return minValue;
	return value;
}

// Применение скорости к мотору с учетом направления
void applyMotorSpeed(Motor &motor, int16_t speed) {
	speed = clampValue(speed, int16_t{-100}, int16_t{100});
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
	measureSpeed[MOTOR_LF] = float(-g_encoders[MOTOR_LF].readDelta()) / g_controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RF] = float(g_encoders[MOTOR_RF].readDelta()) / g_controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_LB] = float(-g_encoders[MOTOR_LB].readDelta()) / g_controlDt / 44 / 56 * 60;
	measureSpeed[MOTOR_RB] = float(g_encoders[MOTOR_RB].readDelta()) / g_controlDt / 44 / 56 * 60;

	for (uint8_t i = 0; i < 4; ++i) {
		measureSpeed[i] = clampValue(measureSpeed[i],
				-g_MAX_MEASURED_SPEED, g_MAX_MEASURED_SPEED);
		measureSpeed[i] = g_speedFilters[i]->update(measureSpeed[i]);
	}

	// Вычисляем уставочные скорости для каждого мотора на основе команды движения (vx, vy, vz)
	int16_t targetSpeed[4]{};
	targetSpeed[MOTOR_LF] = g_motion.vy + g_motion.vx - g_motion.vz;
	targetSpeed[MOTOR_RF] = g_motion.vy - g_motion.vx + g_motion.vz;
	targetSpeed[MOTOR_LB] = g_motion.vy - g_motion.vx - g_motion.vz;
	targetSpeed[MOTOR_RB] = g_motion.vy + g_motion.vx + g_motion.vz;

	// Инициализируем уставочные значения скорости в об/мин
	float targetSpeedForPID[4]{};
	for (uint8_t i = 0; i < 4; ++i) {
		targetSpeedForPID[i] = (float(targetSpeed[i]) / 100) * g_MAX_SPEED;
		if (targetSpeedForPID[i] == 0.0f)
			g_speedFilters[i]->reset();
	}

	float pidOuts[4]{};
	
	// Вычисляем выход ПИД регулятора и применяем его к мотору. Пид-регулятор работает в об/мин, 
	// поэтому нормируем его к диапазону [-100, 100], т.к. моторы управляются в процентах от 
	// максимальной скорости
	pidOuts[MOTOR_LF] = (g_pidControllers[MOTOR_LF].update(targetSpeedForPID[MOTOR_LF], measureSpeed[MOTOR_LF], g_controlDt) / g_MAX_SPEED) * 100;
	pidOuts[MOTOR_RF] = (g_pidControllers[MOTOR_RF].update(targetSpeedForPID[MOTOR_RF], measureSpeed[MOTOR_RF], g_controlDt) / g_MAX_SPEED) * 100;
	pidOuts[MOTOR_LB] = (g_pidControllers[MOTOR_LB].update(targetSpeedForPID[MOTOR_LB], measureSpeed[MOTOR_LB], g_controlDt) / g_MAX_SPEED) * 100;
	pidOuts[MOTOR_RB] = (g_pidControllers[MOTOR_RB].update(targetSpeedForPID[MOTOR_RB], measureSpeed[MOTOR_RB], g_controlDt) / g_MAX_SPEED) * 100;
	for (uint8_t i = 0; i < 4; ++i)
		applyMotorSpeed(g_motors[i], pidOuts[i]);
}

// ---------------------------------------------------------
// Прерывание для расчета скорости и ПИД регулятора
// ---------------------------------------------------------
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim10){

		// Устанавливаем флаг обновления управления в каждом прерывании таймера ПИД регулятора (TIM10)
		g_controlUpdate = true;

		static uint16_t telemetryCycleNumber{0};
		// Каждые 10 циклов (примерно 100 мс) устанавливаем флаг обновления телеметрии для отправки данных на ESP32
		if (++telemetryCycleNumber >= 10) {
			g_telemetryUpdate = true;
			telemetryCycleNumber = 0;
		}

		// Сбрасываем PID, если уставка ноль и обратная связь тоже ноль (чтобы избежать накопления интегральной ошибки)
		static uint16_t pidResetCycleNumber{0};
		if (++pidResetCycleNumber >= 200) {
			g_resetPid = true;
			pidResetCycleNumber = 0;
		}
	}
}

// ---------------------------------------------------------
// UART callback. Прием байт
// ---------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != &huart2)
		return;

	uint8_t byte = g_rxByte;
	static uint8_t checksum = 0;

	if (g_frameState == FrameState::WaitStart1) {
		if (byte == g_FRAME_START_1)
			g_frameState = FrameState::WaitStart2;
	} else if (g_frameState == FrameState::WaitStart2) {
		if (byte == g_FRAME_START_2) {
			g_frameState = FrameState::ReceivePayload;
			g_frameIndex = 0;
			checksum = 0;
		} else {
			g_frameState = FrameState::WaitStart1;
		}
	} else if (g_frameState == FrameState::ReceivePayload) {
		g_framePayload[g_frameIndex++] = byte;
		checksum ^= byte;
		if (g_frameIndex == g_FRAME_PAYLOAD_SIZE)
			g_frameState = FrameState::WaitChecksum;
	} else {
		if (byte == checksum) {
			g_lastValidFrameTick = HAL_GetTick();
			if (!g_frameReady) {
				g_pendingMotion.vx = (int8_t) g_framePayload[0];
				g_pendingMotion.vy = (int8_t) g_framePayload[1];
				g_pendingMotion.vz = (int8_t) -g_framePayload[2];
				g_frameReady = true;
			}
		}
		g_frameState = FrameState::WaitStart1;
	}

	// Продолжаем принимать следующий байт по UART
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &g_rxByte, 1);
}

// Переменная для хранения состояния передачи телеметрии по UART
volatile bool g_txBusy = false;
// Буфер для передачи телеметрии по UART
uint8_t g_txBuff[g_TELEMETRY_FRAME_SIZE];

// Функция для отправки телеметрии на ESP32 через UART
void sendTelemetryFrame() {
	// Если передача телеметрии уже идет или мы находимся в процессе приема кадра, выходим
	if (g_txBusy || g_frameState == FrameState::ReceivePayload)
		return;

	uint8_t* p = g_txBuff; // Указатель на текущую позицию в буфере передачи
	
	// Формируем стартовый кадр телеметрии: синхрометка, тип кадра, длина полезной нагрузки
	*p++ = 0xAA;
	*p++ = 0x55;
	*p++ = g_FRAME_TYPE_TELEMETRY;
	*p++ = g_FRAME_TELEMETRY_PAYLOAD_SIZE;

	// Лямбда-функция для добавления 16-битного значения в буфер передачи
	auto appendInt16 = [&](int16_t value) {
		*p++ = static_cast<uint8_t>(value & 0xFF);
		*p++ = static_cast<uint8_t>((value >> 8) & 0xFF);
	};

	// Добавляем измеренные скорости каждого мотора в буфер передачи
	for (uint8_t i = 0; i < 4; ++i) {
		appendInt16(static_cast<int16_t>(g_pidControllers[i].getMeasureSpeed()));
	}

	// Добавляем статус робота в буфер передачи (например, 0x01 для нормального состояния)
	*p++ = 0x01;

	// Вычисляем контрольную сумму для кадра телеметрии и добавляем ее в буфер передачи
	uint8_t checksum = 0;
	for (uint8_t i = 2; i < static_cast<uint8_t>(p - g_txBuff); ++i)
		checksum ^= g_txBuff[i];
	
	// Добавляем контрольную сумму в буфер передачи
	*p++ = checksum;

	g_txBusy = true;
	
	HAL_UART_Transmit_IT(&huart2, g_txBuff, static_cast<uint16_t>(p - g_txBuff));
}

// ---------------------------------------------------------
// Callback для завершения передачи телеметрии по UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) g_txBusy = false;
}

// ---------------------------------------------------------
// Callback для завершения преобразования АЦП
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1) {
		// Получаем значение напряжения батареи из АЦП
		uint32_t adcValue = HAL_ADC_GetValue(hadc);
		// Преобразуем значение АЦП в напряжение (в вольтах)
		float batteryVoltage = (adcValue / 4095.0f) * 3.3f;
		// С учетом резистивного делителя за 100% считаем напряжение 2.7V
		batteryVoltage = (batteryVoltage / 2.7f) * 100.0f;
	}
}

// ---------------------------------------------------------
// Главная функция
// ---------------------------------------------------------

void cpp_main(void) {
	// Инициализация моторов и энкодеров
	for (uint8_t i = 0; i < 4; ++i) {
		g_motors[i].init();
		g_encoders[i].init();
	}

	// Запускаем таймер для работы ШИМ
	HAL_TIM_Base_Start_IT(&htim1);
	// Запускаем таймер для обработки данных с энкодера
	HAL_TIM_Base_Start_IT(&htim10);
	// Начинаем принимать данные по UART в прерывании
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &g_rxByte, 1);
	// Запускаем АЦП для измерения напряжения батареи
	HAL_ADC_Start(&hadc1);

	// Инициализируем тик последнего валидного кадра, чтобы избежать ложного срабатывания таймаута
	g_lastValidFrameTick = HAL_GetTick();

	// int32_t lastValidFrameTickTelemetry = HAL_GetTick();

	uint32_t lastControlTick{HAL_GetTick()}; // Тик последнего обновления управления
	while (1) {
		bool updateRequired{false}; // Флаг, указывающий, что требуется обновление управления
		bool updateTelemetry{false}; // Флаг для обновления данных телеметрии в esp32

		// Короткая критическая секция для обмена данными с прерываниями
		__disable_irq();
		// Если пришел новый кадр, применяем команду движения до обновления управления
		if (g_frameReady) {
			g_motion.vx = g_pendingMotion.vx;
			g_motion.vy = g_pendingMotion.vy;
			g_motion.vz = g_pendingMotion.vz;
			g_frameReady = false;
		}

		// Если пришло прерывание от таймера ПИД регулятора, устанавливаем флаг 
		// обновления управления (TIM10)
		if (g_controlUpdate) {
			g_controlUpdate = false;
			updateRequired = true;
		}
		
		if (g_telemetryUpdate) {
			// uint32_t nowTelemetry{HAL_GetTick()};
			// dt = nowTelemetry - lastValidFrameTickTelemetry;
			// lastValidFrameTickTelemetry = nowTelemetry;
			g_telemetryUpdate = false;
			updateTelemetry = true;
		}

		__enable_irq();
		
		if (updateTelemetry) sendTelemetryFrame();

		if (updateRequired) {
			uint32_t now{HAL_GetTick()};
			g_controlDt = float((now - lastControlTick) > 0U ? now - lastControlTick : 10U) / 1000.0f;
			lastControlTick = now;
			
			// Применяем команду движения к моторам с учетом ПИД регулятора
			applyMotion();
		}

		// Если прошло больше 300 мс с момента последнего валидного кадра, останавливаем все моторы,
		// обнуляем команды движения и сбрасываем ПИД регуляторы, чтобы избежать накопления 
		// интегральной ошибки
		if (HAL_GetTick() - g_lastValidFrameTick > g_COMMUNICATION_TIMEOUT_MS) {
			g_motion.vx = 0;
			g_motion.vy = 0;
			g_motion.vz = 0;
			g_pendingMotion.vx = 0;
			g_pendingMotion.vy = 0;
			g_pendingMotion.vz = 0;
			g_frameReady = false;
//			}
		}

		if (g_resetPid &&
				g_motion.vx == 0 &&
				g_motion.vy == 0 &&
				g_motion.vz == 0 &&
				g_encoders[MOTOR_LF].readDelta() == 0 &&
				g_encoders[MOTOR_RF].readDelta() == 0 &&
				g_encoders[MOTOR_LB].readDelta() == 0 &&
				g_encoders[MOTOR_RB].readDelta() == 0) {
			g_resetPid = false;
			for (uint8_t i = 0; i < 4; ++i) {
				// Сбрасываем ПИД регулятор, чтобы избежать накопления интегральной ошибки
				g_pidControllers[i].reset();
			}
		}
	}
}
