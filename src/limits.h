#pragma once
#ifndef limits_h
#define limits_h 

#include "grbl.h"

// Initialize the limits module
// Инициализируем модуль ограничений
void limits_init();

// Disables hard limits.
// Отключает жесткие ограничения.
void limits_disable();

// Returns limit state as a bit-wise uint8 variable.
// Возвращает предельное состояние в виде побитовой переменной uint8.
u8 limits_get_state();

// Perform one portion of the homing cycle based on the input settings.
// Выполните одну часть цикла самонаведения в соответствии с входными настройками.
void limits_go_home(u8 cycle_mask);

// Check for soft limit violations
// Проверьте, нет ли нарушений мягкого лимита
void limits_soft_check(float *target);

#endif
