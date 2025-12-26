#pragma once
#ifndef limits_h
#define limits_h

#include <array>
#include "grbl.h"

class Limits {
private:
    GPIO_TypeDef* limit_bit_port_;
    HAL_PinsTypeDef limit_bit_pin_;
    HAL_GPIO_Line_Config irq_config_;

public:

  // Initialize the limits module
  // Инициализируем модуль ограничений
  void LimitsInit(const HAL_PinsTypeDef pin, GPIO_TypeDef* port, HAL_GPIO_Line_Config irq_line);

  // Disables hard limits.
  // Отключает жесткие ограничения.
  void LimitsDisable();

  // Returns limit state as a bit-wise uint8 variable.
  // Возвращает предельное состояние в виде побитовой переменной uint8.
  bool LimitGetState();

  // Perform one portion of the homing cycle based on the input settings.
  // Выполните одну часть цикла самонаведения в соответствии с входными настройками.
  void LimitsGoHome(uint8_t cycle_mask);

  // Check for soft limit violations
  // Проверьте, нет ли нарушений мягкого лимита
  void LimitsSoftCheck(float *target);
};

#endif
