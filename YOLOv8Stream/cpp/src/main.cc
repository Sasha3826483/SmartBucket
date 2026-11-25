#include "mjpeg_server.h" // HTTP-сервер для передачи обработанных кадров в формате MJPEG.
#include "yolov8.h"  // Интерфейс проекта для RKNN, модели и постобработки YOLO.
#include <algorithm> // std::max для ограничения координаты подписи.
#include <chrono>    // Замер времени чтения кадра.
#include <cmath>     // std::lround для преобразования угла в байт.
#include <opencv2/core/core.hpp> // Базовые типы OpenCV: Mat, Point, Scalar.
#include <opencv2/highgui/highgui.hpp> // Высокоуровневый ввод-вывод OpenCV, включая захват видео.
#include <opencv2/imgproc/imgproc.hpp> // Обработка изображений: resize, rectangle, putText.
#include <stdint.h> // Типы фиксированной разрядности, например uint8_t.
#include <stdio.h>  // Вывод сообщений и результатов в консоль.
#include <stdlib.h> // Общие функции C и управление памятью.
#include <string.h> // Работа с C-строками и памятью.
#include <string>   // Строка подписи для OpenCV.

#define MODEL_INPUT_SIZE 640
#define CAMERA_DEV 11

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SPI_DEVICE_PATH "/dev/spidev0.0"

float factY{};
float factX{};
float target{640.0 / 2};
int angleBase{0};
float errorY{0};
float errorX{0};
constexpr float deadBandPixels{20.0f};
constexpr float maxServoSpeedDegreesPerSecond{30.0f};
constexpr float servoSpeedGain{0.2f};
constexpr int8_t maxPlatformSpeed{50};
constexpr float platformSpeedGain{0.2f};
float commandedServoAngle{0.0f};
std::chrono::steady_clock::time_point lastServoUpdate{};

uint8_t N{4};

typedef struct control {
  uint8_t angle;
  int8_t Vx;
  int8_t Vy;
  int8_t Vz;
};

control control = {.angle = 0, .Vx = 0, .Vy = 0, .Vz = 0};

float g_controlDt{};

int spi_push() {

  int spi_file;
  uint8_t tx_buffer[4] = {control.angle, control.Vx, control.Vy, control.Vz};
  uint8_t rx_buffer[4] = {};

  // Open the SPI device
  if ((spi_file = open(SPI_DEVICE_PATH, O_RDWR)) < 0) {
    perror("Failed to open SPI device");
    return -1;
  }

  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (ioctl(spi_file, SPI_IOC_WR_MODE, &mode) < 0) {
    perror("Failed to set SPI mode");
    close(spi_file);
    return -1;
  }

  struct spi_ioc_transfer transfer = {
      .tx_buf = (unsigned long)tx_buffer,
      .rx_buf = (unsigned long)rx_buffer,
      .len = sizeof(tx_buffer),
      .speed_hz = 100000, // SPI speed in Hz
      .delay_usecs = 0,
      .bits_per_word = 8,
  };

  if (ioctl(spi_file, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    perror("Failed to perform SPI transfer");
    close(spi_file);
    return -1;
  }

  close(spi_file);
  return 0;
}

float getServoUpdateInterval() {
  const auto now = std::chrono::steady_clock::now();
  float dt = std::chrono::duration<float>(now - lastServoUpdate).count();
  lastServoUpdate = now;
  return std::max(dt, 0.0f);
}

void moveServo(float angularVelocity) {
  float dt = getServoUpdateInterval();
  angularVelocity =
      std::max(-maxServoSpeedDegreesPerSecond,
               std::min(angularVelocity, maxServoSpeedDegreesPerSecond));
  commandedServoAngle += angularVelocity * dt;
  commandedServoAngle = std::max(0.0f, std::min(commandedServoAngle, 90.0f));
  control.angle = commandedServoAngle;
}

void returnServoToBase() {
  float maxStep = maxServoSpeedDegreesPerSecond * getServoUpdateInterval();
  float difference = static_cast<float>(angleBase) - commandedServoAngle;
  float angleChange = std::max(-maxStep, std::min(difference, maxStep));
  commandedServoAngle += angleChange;
  control.angle = static_cast<uint8_t>(std::lround(commandedServoAngle));
}

void moveMotor(float speed) {
  speed = std::max(static_cast<float>(-maxPlatformSpeed),
                   std::min(speed, static_cast<float>(maxPlatformSpeed)));
  control.Vz = static_cast<int8_t>(std::lround(speed));
}

// Возвращает стабильный различимый цвет для любого из 80 ID классов COCO.
static cv::Scalar classColor(int class_id) {
  if (class_id < 0 || class_id >= OBJ_CLASS_NUM) {
    return cv::Scalar(255, 255, 255);
  }

  return cv::Scalar(50 + (class_id * 71) % 206, 50 + (class_id * 139) % 206,
                    50 + (class_id * 197) % 206);
}

int main(int argc, char **argv) {
  // Программа принимает ровно один аргумент — путь к RKNN-модели.
  if (argc != 2) {
    printf("%s <model_path>\n", argv[0]);
    return -1;
  }

  const char *model_path = argv[1];

  int ret;
  // Контекст приложения, который хранит дескриптор модели, атрибуты и буферы.
  rknn_app_context_t rknn_app_ctx;
  // Обнуляем контекст перед тем, как в него запишутся RKNN-ресурсы.
  memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

  // Загружаем имена классов, которые используются при печати и подписи рамок.
  init_post_process();

  // Создаём контекст NPU и выделяем входные/выходные буферы для модели.
  ret = init_yolov8_model(model_path, &rknn_app_ctx);
  if (ret != 0) {
    printf("init_yolov8_model fail! ret=%d model_path=%s\n", ret, model_path);
    return -1;
  }

  // OpenCV-матрица оборачивает память входного тензора RKNN без отдельного
  // буфера.
  cv::Mat bgr640(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, CV_8UC3,
                 rknn_app_ctx.input_mems[0]->virt_addr);

  cv::VideoCapture cap;
  // Запрашиваем у камеры размер кадра модели и открываем камеру с индексом
  // CAMERA_DEV.
  cap.set(cv::CAP_PROP_FRAME_WIDTH, MODEL_INPUT_SIZE);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, MODEL_INPUT_SIZE);
  cap.open(CAMERA_DEV);
  if (!cap.isOpened()) {
    fprintf(stderr, "Failed to open camera %d\n", CAMERA_DEV);
    return -1;
  }
  cv::Mat camFrame;

  // Запускаем MJPEG-сервер на всех сетевых интерфейсах, включая USB Ethernet.
  MjpegServer stream_server;
  if (!stream_server.start(8080)) {
    fprintf(stderr, "Failed to start MJPEG stream server\n");
    return -1;
  }

  commandedServoAngle = static_cast<float>(angleBase);
  lastServoUpdate = std::chrono::steady_clock::now();
  control.angle = 0;
  spi_push();

  // Непрерывная обработка кадров: захват → масштабирование → инференс →
  // детекции.
  while (1) {
    std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    cap >> camFrame;
    if (camFrame.empty()) {
      returnServoToBase();
      moveMotor(0.0f);
      spi_push();
      continue;
    }

    // Масштабируем кадр до размера входа модели; результат записывается в буфер
    // RKNN.
    cv::resize(camFrame, bgr640, cv::Size(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
               0, 0, cv::INTER_LINEAR);

    // Запускаем нейросеть на NPU; выходные тензоры сохраняются в контексте
    // модели.
    rknn_run(rknn_app_ctx.rknn_ctx, nullptr);

    // Преобразуем выходы YOLO в список рамок и классов.
    // 0.25 — минимальная уверенность, 0.45 — порог подавления перекрывающихся
    // рамок (NMS).
    object_detect_result_list od_results;
    post_process(&rknn_app_ctx, rknn_app_ctx.output_mems, 0.25, 0.45,
                 &od_results);

    // Для каждой детекции печатаем класс и уверенность, затем рисуем рамку и
    // подпись.
    bool detectionFound = false;
    float highestConfidence = -1.0f;
    for (int i = 0; i < od_results.count; i++) {
      object_detect_result *det_result = &(od_results.results[i]);

      // Координаты прямоугольника в системе координат входа модели.
      int x1 = det_result->box.left;
      int y1 = det_result->box.top;
      int x2 = det_result->box.right;
      int y2 = det_result->box.bottom;
      cv::Rect r = cv::Rect(x1, y1, x2 - x1, y2 - y1);

      // Выводим название класса и confidence в консоль.
      printf("%s: %.2f\n", coco_cls_to_name(det_result->cls_id),
             det_result->prop);

      // Рисуем рамку на исходном кадре.
      cv::rectangle(camFrame, r, classColor(det_result->cls_id), 1, 8, 0);

      // Формируем текст для изображения: имя класса и уверенность.
      std::string label = cv::format(
          "%s %.2f", coco_cls_to_name(det_result->cls_id), det_result->prop);

      // Рисуем подпись около верхнего края рамки; y не даём уйти за край кадра.
      cv::putText(
          camFrame, // Изображение, на которое наносится текст
          label,    // Текст подписи: класс и уверенность
          cv::Point(
              x1,
              std::max(y1 - 5, 15)), // Начало текста: x рамки и y над ней,
                                     // но не выше 15 пикселей от верхнего края
          cv::FONT_HERSHEY_SIMPLEX,  // Шрифт
          0.5,                       // Масштаб шрифта
          classColor(det_result->cls_id), // Цвет текста — такой же, как у рамки
          1,                              // Толщина линий текста в пикселях
          cv::LINE_AA                     // Сглаживание линий
      );

      if (det_result->prop > highestConfidence) {
        highestConfidence = det_result->prop;
        factY = (static_cast<float>(y1) + static_cast<float>(y2)) / 2.0f;
        factX = (static_cast<float>(x1) + static_cast<float>(x2)) / 2.0f;
        detectionFound = true;
      }
    }

    // Отправляем обработанный кадр подключённому браузеру по USB-сети.
    stream_server.publish(camFrame);

    std::chrono::steady_clock::time_point end =
        std::chrono::steady_clock::now();

    g_controlDt = std::chrono::duration<double>(end - begin).count();

    printf("Frame latency: %lf\n", g_controlDt);

    if (detectionFound) {

      errorY = target - factY;
      float errorOutsideDeadBand = std::abs(errorY) - deadBandPixels;
      if (errorOutsideDeadBand > 0.0f) {
        float speed = std::min(maxServoSpeedDegreesPerSecond,
                               errorOutsideDeadBand * servoSpeedGain);
        moveServo(errorY > 0.0f ? speed : -speed);
      } else {
        moveServo(0.0f);
      }

      errorX = target - factX;
      errorOutsideDeadBand = std::abs(errorX) - deadBandPixels;
      if (errorOutsideDeadBand > 0.0f) {
        float speed =
            std::min(static_cast<float>(maxPlatformSpeed),
                     errorOutsideDeadBand * platformSpeedGain);
        moveMotor(errorX > 0.0f ? speed : -speed);
      } else {
        moveMotor(0.0f);
      }
    } else {
      returnServoToBase();
      moveMotor(0.0f);
    }
    spi_push();
  }
  return 0;
}
