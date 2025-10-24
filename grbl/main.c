/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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

#include "grbl.h"


// Declare system global variable structure
// Глобальная структура 
system_t sys; 


int main(void)
{
  // Инициализация системы при включении питания
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts // Настройка серийного порта и прерываний
  settings_init(); // Load Grbl settings from EEPROM // Загрузка настроек grbl из EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers // Настройка выходов шаговых двигателей и таймеров прерываний
  system_init();   // Configure pinout pins and pin-change interrupt // Настройка выводов распиновки и прерывания смены выводов
  
  memset(&sys, 0, sizeof(system_t));  // Clear all system variables // Очистка всех системных переменных
  sys.abort = true;   // Set abort to complete initialization // Установка значения переменной abort при окончании инициализации
  sei(); // Enable interrupts //Включение перерываний

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  // Проверяем включение питания и установите системный сигнал тревоги, если включен режим самонаведения, для принудительного цикла самонаведения
  // установив состояние тревоги Grbl. Сигнал тревоги блокирует все команды g-кода, включая сценарии запуска
  //, но позволяет получить доступ к настройкам и внутренним командам. Только режим самонаведения
  // цикл "$H" или отключение блокировки сигнализации "$X" приведет к отключению сигнализации.
  // ПРИМЕЧАНИЕ: Сценарий запуска будет запущен после успешного завершения цикла самонаведения, но
  // не после отключения блокировок сигнализации. Предотвращает попадание блоков запуска motion в
  // все происходит бесконтрольно. Очень плохо.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Переводит Grbl в аварийное состояние после отключения питания или жесткого сброса.
  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif

  // Цикл инициализации Grbl при включении питания или прерывании работы системы. В последнем случае все процессы
  // возвращаются к этому циклу для повторной инициализации.
  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Отдельная задача настройки, требующая отключения прерываний, особенно при
    // прерывание работы системы и обеспечение того, чтобы все активные прерывания были полностью сброшены.
    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.
  
    // Reset Grbl primary systems. // Перезагрузите основные системы Grbl.
    serial_reset_read_buffer(); // Clear serial read buffer //Очищает Com порт
    gc_init(); // Set g-code parser to default state // Устанавливает g-code парсер в состояние по умолчанию
    spindle_init();
    coolant_init();
    limits_init(); 
    probe_init();
    plan_reset(); // Clear block buffer and planner variables //Очистить буфер блоков и переменную планировщика
    st_reset(); // Clear stepper subsystem variables. //Очистить переменные шаговой подсистемы

    // Синхронизируйте очищенные позиции gcode и планировать с текущей позицией в системе.
    // Sync cleared gcode and planner positions to current system position. 
    plan_sync_position();
    gc_sync_position();

    // Сброс системных переменных.
    // Reset system variables.
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
    sys.soft_limit = false;

    // Запустите основной цикл Grbl. Обрабатывает вводимые программой данные и выполняет их.
    // Start Grbl main loop. Processes program inputs and executes them. 
    protocol_main_loop();
    
  }
  return 0;   /* Never reached */
}
