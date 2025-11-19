/*
  probe.h - code pertaining to probing methods
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
  
#ifndef probe_h
#define probe_h 

// Values that define the probing state machine.   // Значения, которые определяют конечный автомат проверки.
#define PROBE_OFF     0 // Probing disabled or not in use. (Must be zero.) // Проверка отключена или не используется. (Должно быть равно нулю.)
#define PROBE_ACTIVE  1 // Actively watching the input pin. // Активно следит за вводимым выводом.

// Probe pin initialization routine. // Процедура инициализации пин-кода датчика.
void probe_init();

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to 
// appropriately set the pin logic according to setting for normal-high/normal-low operation 
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
// Вызывается подпрограммами probe_init() и mc_probe(). Устанавливает маску инвертирования выводов датчика на 
// соответствующим образом настраивает логику вывода в соответствии с настройками для работы в режиме normal-high/normal-low 
// и режимы цикла зондирования для приближения к заготовке/удаления от заготовки. 
void probe_configure_invert_mask(uint8_t is_probe_away);

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
// Возвращает состояние пин-кода проверки. Срабатывает = true. Вызывается анализатором gcode и монитором состояния проверки.
uint8_t probe_get_state();

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// Отслеживает состояние контакта датчика и записывает положение системы при обнаружении. Вызывается с помощью
// stepper ISR за такт ISR.
void probe_state_monitor();

#endif
