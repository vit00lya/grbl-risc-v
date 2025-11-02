/*
  protocol.h - controls Grbl execution protocol and procedures
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

#ifndef protocol_h
#define protocol_h

// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have this 
// buffer.
// Размер буфера строк из потока последовательного ввода, который должен быть выполнен.
// ПРИМЕЧАНИЕ: Это не проблема, за исключением крайних случаев, но размер буфера строк может быть слишком мал
// и блоки g-кода могут быть обрезаны. Официально стандарты g-кода поддерживают до 256
// символов. В будущих версиях это значение будет увеличено, когда мы узнаем, сколько дополнительного
// пространства памяти мы можем здесь использовать, или мы перепишем синтаксический анализатор g-кода, чтобы в нем не было этого 
// буфера.
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// Starts Grbl main loop. It handles all incoming characters from the serial port and executes
// them as they complete. It is also responsible for finishing the initialization procedures.
// Запускает основной цикл Grbl. Он обрабатывает все входящие символы из последовательного порта и выполняет
// их по мере завершения. Он также отвечает за завершение процедур инициализации.
void protocol_main_loop();

// Checks and executes a realtime command at various stop points in main program
// Проверяет и выполняет команду в реальном времени в различных точках остановки основной программы
void protocol_execute_realtime();

// Уведомить шаговую подсистему о начале выполнения программы g-code в буфере.
// Notify the stepper subsystem to start executing the g-code program in buffer.
// void protocol_cycle_start();

// Повторно инициализирует буфер после задержки подачи для возобновления.
// Reinitializes the buffer after a feed hold for a resume.
// void protocol_cycle_reinitialize(); 

// Инициирует удержание загрузки запущенной программы
// Initiates a feed hold of the running program
// void protocol_feed_hold();

// Запускает функцию автоматического цикла, если она включена.
// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start();

// Блокировать до тех пор, пока не будут выполнены все буферизованные шаги
// Block until all buffered steps are executed
void protocol_buffer_synchronize();

#endif
