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

typedef  signed char         i8;
typedef  signed short        i16;
typedef  signed long         i32;
typedef  signed long long    i64;

typedef  unsigned char       u8;
typedef  unsigned short      u16;
typedef  unsigned long       u32;
typedef  unsigned long long  u64;

typedef  float f;

extern "C" {
    #include "mik32_hal_pcc.h"
    #include "mik32_hal_gpio.h"
    #include <mik32_memory_map.h> 
    #include <pad_config.h> 
    #include <gpio.h> 
    #include <power_manager.h> 
    #include <wakeup.h>
}
#include <cmath>

#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD "20160726"

#include "config.h"
#include "cpu_map.h"
#include "utils.h"
#include "defaults.h"
#include "settings.h"
#include "system.h"



