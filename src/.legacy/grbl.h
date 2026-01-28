#pragma once

/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2015 Sungeun K. Jeon
  Copyright (c) 2025 Сандалов В.П. vit00lya@yandex.ru
  Copyright (c) 2025 Порошин Д.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Значения индекса массива Axis. Должны начинаться с 0 и быть непрерывными.
// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 3 // Number of axes // Количество осей
#define X_AXIS 0 // Axis indexing value.  // Значение индексации оси.
#define Y_AXIS 1
#define Z_AXIS 2

extern "C" {
    #include "mik32_hal_pcc.h"
    #include "mik32_hal_gpio.h"
    #include "mik32_hal_eeprom.h"
    #include "mik32_hal_timer32.h"
    #include "mik32_hal_irq.h"
    #include "mik32_hal.h"
    #include <mik32_memory_map.h> 
    #include <pad_config.h> 
    #include <gpio.h> 
    #include <power_manager.h> 
    #include <wakeup.h>
    #include "uart_lib.h"
    #include "xprintf.h"
    #include "mik32_hal_timer16.h"
}


#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD "20160726"

#include "config.h"
#include "cpu_map.h"
#include "utils.h"
#include "defaults.h"
// #include "settings.h"
 #include "system.h"
//  #include "limits.h"
//  #include "machine.h"
// #include "serial.h"
// #include "report.h"
// #include "print.h"
// #include "gcode.h"
// #include "planner.h"
// #include "limits.h"

// #include "eeprom_utils.h"
// #include "limits.h"


