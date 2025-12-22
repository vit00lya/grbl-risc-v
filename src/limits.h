#pragma once
#ifndef limits_h
#define limits_h 

#include <array>
#include "grbl.h"

//  std::array<GPIO_TypeDef*, 3> limits_ports_ = {X_LIMIT_BIT_PORT, Y_LIMIT_BIT_PORT, Z_LIMIT_BIT_PORT };
//  std::array< HAL_PinsTypeDef, 3> limits_pins_ = { X_LIMIT_BIT_PIN, Y_LIMIT_BIT_PIN, Z_LIMIT_BIT_PIN };

void pin_init(const HAL_PinsTypeDef pin, GPIO_TypeDef* port, bool pull_up, HAL_GPIO_Line_Config irq_line);

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
