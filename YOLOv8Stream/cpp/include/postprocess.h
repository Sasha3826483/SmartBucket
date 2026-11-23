#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include "common.h"
#include "image_utils.h"
#include "rknn_api.h"
#include <stdint.h>
#include <vector>

// Максимальная длина имени класса объекта (например, "person", "bicycle").
#define OBJ_NAME_MAX_SIZE 64
// Максимальное число объектов, которое можно вернуть в одном результате.
#define OBJ_NUMB_MAX_SIZE 128
// Количество классов в модели COCO.
#define MODEL_CLASS_NUM 1
#define OBJ_CLASS_NUM 1
// Индексы наиболее распространённых классов COCO.
#define COCO_PERSON_CLASS_ID 0
#define COCO_BICYCLE_CLASS_ID 1
// Пороговые значения для фильтрации и подавления лишних рамок.
#define NMS_THRESH 0.45
#define BOX_THRESH 0.25

// class rknn_app_context_t;

// Описание одного обнаруженного объекта: ограничивающая рамка, уверенность,
// а также идентификатор класса, к которому относится объект.
typedef struct {
  image_rect_t box; // Координаты прямоугольника объекта в исходном масштабе.
  float prop;       // Достоверность/уверенность обнаружения.
  int cls_id;       // Идентификатор класса (0..79 для COCO).
} object_detect_result;

// Список обнаруженных объектов, возвращаемый постпроцессингом.
typedef struct {
  int id;    // Идентификатор набора результатов (например, номер кадра).
  int count; // Количество объектов в массиве results.
  object_detect_result results[OBJ_NUMB_MAX_SIZE]; // Массив найденных объектов.
} object_detect_result_list;

// Инициализация внутренних ресурсов постпроцессинга.
int init_post_process();
// Освобождение внутренних ресурсов постпроцессинга.
void deinit_post_process();
// Преобразование ID класса COCO в строковое имя класса.
char *coco_cls_to_name(int cls_id);

// Основная функция постпроцессинга: принимает выходы модели, фильтрует
// кандидаты по порогу уверенности, применяет NMS и формирует список объектов.
int post_process(rknn_app_context_t *app_ctx, void *outputs,
                 float conf_threshold, float nms_threshold,
                 object_detect_result_list *od_results);

// Альтернативное имя для deinit_post_process() в старом коде проекта.
void deinitPostProcess();
#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
