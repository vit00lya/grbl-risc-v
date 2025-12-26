#pragma once
#ifndef planner_h
#define planner_h

#include "grbl.h"

// The number of linear motions that can be in the plan at any give time
// Количество линейных перемещений, которые могут быть выполнены в плане в любой момент времени
#ifndef BLOCK_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define BLOCK_BUFFER_SIZE 16
  #else
    #define BLOCK_BUFFER_SIZE 18
  #endif
#endif

// This struct stores a linear movement of a g-code block motion with its critical "nominal" values
// are as specified in the source g-code. 
// Эта структура хранит линейное перемещение блока g-кода с его критическими "номинальными" значениями
// как указано в исходном g-коде.
struct plan_block_t{
  // Fields used by the bresenham algorithm for tracing the line
  // NOTE: Used by stepper algorithm to execute the block correctly. Do not alter these values.
  // Поля, используемые алгоритмом Брезенхэма для трассировки строки
  // ПРИМЕЧАНИЕ: Используются алгоритмом stepper для корректного выполнения блока. Не изменяйте эти значения. 
  uint8_t direction_bits;    // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h) // Бит направления, установленный для этого блока (относится к *_DIRECTION_BIT в файле config.h)
  uint32_t steps[N_AXIS];    // Step count along each axis // Количество шагов вдоль каждой оси
  uint32_t step_event_count; // The maximum step axis count and number of steps required to complete this block.  // Максимальное количество осей шага и количество шагов, необходимых для завершения этого блока.

  // Fields used by the motion planner to manage acceleration // Поля, используемые планировщиком движения для управления ускорением
  float entry_speed_sqr;         // The current planned entry speed at block junction in (mm/min)^2 // Текущая планируемая скорость въезда на перекрестке блоков в (мм/мин)^2
  float max_entry_speed_sqr;     // Maximum allowable entry speed based on the minimum of junction limit and 
                                 //   neighboring nominal speeds with overrides in (mm/min)^2
                                 // Максимально допустимая скорость на входе, основанная на минимальном пределе соединения и
                                 // соседних номинальных скоростях с превышениями в (мм/мин)^2
  float max_junction_speed_sqr;  // Junction entry speed limit based on direction vectors in (mm/min)^2 // Ограничение скорости при въезде на перекресток в зависимости от векторов направления в (мм/мин)^2
  float nominal_speed_sqr;       // Axis-limit adjusted nominal speed for this block in (mm/min)^2 // Предельная установленная номинальная скорость вращения по оси для данного блока в (мм/мин)^2
  float acceleration;            // Axis-limit adjusted line acceleration in (mm/min^2) // Заданное линейное ускорение по оси (мм/мин^2)
  float millimeters;             // The remaining distance for this block to be executed in (mm) // Оставшееся расстояние для выполнения этого блока в (мм)
  // uint8_t max_override;       // Maximum override value based on axis speed limits // Максимальное значение переопределения, основанное на ограничениях скорости по оси
 
  #ifdef USE_LINE_NUMBERS
    int32_t line_number;
  #endif
};

      
// Initialize and reset the motion plan subsystem
// Инициализировать и сбросить подсистему планирования движения
void plan_reset();

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position 
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
// Добавьте новое линейное перемещение в буфер. цель [N_AXIS] - это абсолютное целевое положение со знаком 
// в миллиметрах. Скорость подачи определяет скорость перемещения. Если скорость подачи инвертирована, подача
// rate принимается за "частоту" и завершает операцию за 1/feed_rate минуты.
#ifdef USE_LINE_NUMBERS
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
// Вызывается, когда текущий блок больше не нужен. Удаляет блок и освобождает память
// для новых блоков.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
// Возвращает текущий блок. Возвращает значение NULL, если буфер пуст
plan_block_t *plan_get_current_block();

// Called periodically by step segment buffer. Mostly used internally by planner.
// Периодически вызывается буфером пошаговых сегментов. В основном используется внутри planner.
uint8_t plan_next_block_index(uint8_t block_index);

// Called by step segment buffer when computing executing block velocity profile.
// Вызывается буфером пошагового сегмента при вычислении профиля скорости выполнения блока.
float plan_get_exec_block_exit_speed();

// Reset the planner position vector (in steps)
// Сбросить вектор положения планировщика (поэтапно)
void plan_sync_position();

// Reinitialize plan with a partially completed block
// Повторно инициализировать план с частично завершенным блоком
void plan_cycle_reinitialize();

// Returns the number of active blocks are in the planner buffer.
// Возвращает количество активных блоков, находящихся в буфере планировщика.
uint8_t plan_get_block_buffer_count();

// Returns the status of the block ring buffer. True, if buffer is full.
// Возвращает состояние кольцевого буфера блока. Значение True, если буфер заполнен.
uint8_t plan_check_full_buffer();

#endif
