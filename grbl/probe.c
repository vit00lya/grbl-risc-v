/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2015 Sungeun K. Jeon

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
  
#include "grbl.h"

// Инвертирует состояние штифта датчика в зависимости от пользовательских настроек и режима цикла измерения.
// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;

// Процедура инициализации пин-кода датчика.
// Probe pin initialization routine.
void probe_init() 
{
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins // Настроить в качестве входных контактов
  #ifdef DISABLE_PROBE_PIN_PULL_UP
    PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down. // Нормальная работа на низком уровне. Требуется внешнее выдвижение.
  #else
    PROBE_PORT |= PROBE_MASK;    // Enable internal pull-up resistors. Normal high operation. // Включите внутренние подтягивающие резисторы. Нормальная работа на высоком уровне.
  #endif
  // probe_configure_invert_mask(false); // Initialize invert mask. Not required. Updated when in-use. // Инициализируем инвертирующую маску. Не требовалось. Обновляется при использовании.
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to 
// appropriately set the pin logic according to setting for normal-high/normal-low operation 
// and the probing cycle modes for toward-workpiece/away-from-workpiece. 
// Вызывается подпрограммами probe_init() и mc_probe(). Устанавливает маску инвертирования выводов датчика на 
// соответствующим образом настраивает логику вывода в соответствии с настройками для работы в режиме normal-high/normal-low 
// и режимы цикла зондирования для приближения к заготовке/удаления от заготовки.
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero. // Инициализировать как ноль.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}


// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor. 
// Возвращает состояние пин-кода проверки. Срабатывает = true. Вызывается анализатором gcode и монитором состояния проверки.
uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }


// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
// Отслеживает состояние выводов датчика и записывает положение системы при обнаружении. Вызывается с помощью
// stepper ISR для каждого тика ISR.
// ПРИМЕЧАНИЕ: Эта функция должна быть чрезвычайно эффективной, чтобы не перегружать stepper ISR.
void probe_state_monitor()
{
  if (sys_probe_state == PROBE_ACTIVE) {
    if (probe_get_state()) {
      sys_probe_state = PROBE_OFF;
      memcpy(sys.probe_position, sys.position, sizeof(sys.position));
      bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
    }
  }
}
