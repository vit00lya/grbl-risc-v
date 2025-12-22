/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef stepper_h
#define stepper_h 

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

#include "planner.h"
#include <math.h>
#include "system.h"

// Initialize and setup the stepper motor subsystem
// Инициализация и настройка подсистемы шагового двигателя
void stepper_init();

// Enable steppers, but cycle does not start unless called by motion control or realtime command.
// Включить степперы, но цикл не запускается, если он не вызван системой управления движением или командой реального времени.
void st_wake_up();

// Immediately disables steppers
// Немедленно отключает степперы
void st_go_idle();

// Generate the step and direction port invert masks.
// Сгенерируйте маски инвертирования шага и направления порта.
void st_generate_step_dir_invert_masks();

// Reset the stepper subsystem variables 
// Сброс переменных шаговой подсистемы      
void st_reset();
             
// Reloads step segment buffer. Called continuously by realtime execution system.
// Перезагружает буфер сегмента шага. Непрерывно вызывается системой выполнения в реальном времени.
void st_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
// Вызывается функцией planner_recalculate(), когда исполняемый блок обновляется в соответствии с новым планом.
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
// Вызывается с помощью realtime status reporting, если в config.h включена функция realtime rate reporting.
#ifdef REPORT_REALTIME_RATE
float st_get_realtime_rate();
#endif

#endif
