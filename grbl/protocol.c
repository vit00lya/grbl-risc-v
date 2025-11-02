/*
  protocol.c - controls Grbl execution protocol and procedures
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

// Определите различные типы комментариев для предварительного анализа.
// Define different comment types for pre-parsing.
#define COMMENT_NONE 0
#define COMMENT_TYPE_PARENTHESES 1
#define COMMENT_TYPE_SEMICOLON 2


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated. // Строка, которая должна быть выполнена. Завершается нулем.



// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
// Направляет и выполняет одну строку форматированного ввода из protocol_process. Хотя в основном
// входящий потоковый g-код блокируется, он также направляет и выполняет внутренние команды Grbl,
// такие как настройки, инициирование цикла самонаведения и переключение состояний переключателя.
static void protocol_execute_line(char *line) 
{      
  protocol_execute_realtime(); // Runtime command check point. // Контрольная точка выполнения команды.
  if (sys.abort) { return; } // Bail to calling function upon system abort   // Переход к вызывающей функции при прерывании работы системы

  #ifdef REPORT_ECHO_LINE_RECEIVED
    report_echo_line_received(line);
  #endif

  if (line[0] == 0) {
    // Empty or comment line. Send status message for syncing purposes. // Пустая строка или строка с комментарием. Отправьте сообщение о статусе для синхронизации.
    report_status_message(STATUS_OK);

  } else if (line[0] == '$') {
    // Grbl '$' system command // Системная команда Grbl '$'
    report_status_message(system_execute_line(line));
    
  } else if (sys.state == STATE_ALARM) {
    // Everything else is gcode. Block if in alarm mode. // Все остальное - это gcode. Заблокируйте, если он находится в режиме тревоги.
    report_status_message(STATUS_ALARM_LOCK);

  } else {
    // Parse and execute g-code block! // Разобрать и выполнить блок g-кода!
    report_status_message(gc_execute_line(line));
  }
}


/* 
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // ------------------------------------------------------------
  // Complete initialization procedures upon a power-up or reset.
  // ------------------------------------------------------------
  
  // Print welcome message   

  // ------------------------------------------------------------
  // Завершите процедуры инициализации при включении питания или сбросе настроек.
  // ------------------------------------------------------------
  
  // Распечатайте приветственное сообщение
  report_init_message();

  // Check for and report alarm state after a reset, error, or an initial power up.
  // Проверьте наличие аварийного состояния и сообщите о нем после сброса, ошибки или первоначального включения питания.
  if (sys.state == STATE_ALARM) {
    report_feedback_message(MESSAGE_ALARM_LOCK); 
  } else {
    // All systems go! But first check for safety door.
    // Все системы работают исправно! Но сначала проверьте наличие защитной двери.
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state. // Войдите в режим защитной двери. Должен вернуться в режим ожидания.
    } else {
      sys.state = STATE_IDLE; // Set system to ready. Clear all state flags. // Переведите систему в режим готовности. Снимите все флажки состояния.
    } 
    system_execute_startup(line); // Execute startup script. // Запустите сценарий запуска.
  }
    
  // ---------------------------------------------------------------------------------  
  // Primary loop! Upon a system abort, this exits back to main() to reset the system. 
  // ---------------------------------------------------------------------------------  
  // ---------------------------------------------------------------------------------  
  // Основной цикл! При прерывании работы системы программа возвращается в режим main() для перезагрузки системы. 
  // ---------------------------------------------------------------------------------

  uint8_t comment = COMMENT_NONE;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    
    // NOTE: While comment, spaces, and block delete(if supported) handling should technically 
    // be done in the g-code parser, doing it here helps compress the incoming data into Grbl's
    // line buffer, which is limited in size. The g-code standard actually states a line can't
    // exceed 256 characters, but the Arduino Uno does not have the memory space for this.
    // With a better processor, it would be very easy to pull this initial parsing out as a 
    // seperate task to be shared by the g-code parser and Grbl's system commands.

    // Обрабатывает одну строку входящих последовательных данных по мере их поступления. Выполняет
    // первоначальную фильтрацию, удаляя пробелы и комментарии и вводя все буквы с заглавной буквы.
    
    // ПРИМЕЧАНИЕ: Хотя обработка комментариев, пробелов и удаления блоков (если поддерживается) технически должна 
    // выполняется в синтаксическом анализаторе g-code, это помогает сжать входящие данные в Grbl
    // буфер строк, размер которого ограничен. В стандарте g-code указано, что строка не может
    // превышает 256 символов, но в Arduino Uno для этого недостаточно места в памяти.
    // С лучшим процессором было бы очень легко выполнить этот начальный синтаксический анализ как
    // отдельную задачу, которая будет совместно использоваться анализатором g-кода и системными командами Grbl.
    
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached // Достигнут конец строки
        line[char_counter] = 0; // Set string termination character. // Задает символ завершения строки.
        protocol_execute_line(line); // Line is complete. Execute it! // Строка завершена. Выполните ее!
        comment = COMMENT_NONE;
        char_counter = 0;
      } else {
        if (comment != COMMENT_NONE) {
          // Throw away all comment characters // Удалить все символы комментариев
          if (c == ')') {
            // End of comment. Resume line. But, not if semicolon type comment. // Удалить все символы комментариев
            if (comment == COMMENT_TYPE_PARENTHESES) { comment = COMMENT_NONE; }
          }
        } else {
          if (c <= ' ') { 
            // Throw away whitepace and control characters   // Убрать пробелы и управляющие символы
          } else if (c == '/') { 
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
            // Блокировка удаления НЕ ПОДДЕРЖИВАЕТСЯ. Игнорируйте символ.
            // ПРИМЕЧАНИЕ: Если поддерживается, просто нужно проверить систему, включена ли функция удаления блоков.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            // Включите флажок "комментарии" и игнорируйте все символы, кроме ')' или EOL.
            // ПРИМЕЧАНИЕ: Это не совсем соответствует определению NIST, но на данный момент этого достаточно.
            // В будущем мы могли бы просто удалить элементы в комментариях, но сохранить
            //управляющие символы 
            // comment, чтобы синтаксический анализатор g-кода мог проверить их на наличие ошибок.
            comment = COMMENT_TYPE_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            // ПРИМЕЧАНИЕ: ';' комментарий к EOL - это определение LinuxCNC. Не NIST.
            comment = COMMENT_TYPE_SEMICOLON;
            
          // TODO: Install '%' feature 
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute 
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.
            // ЗАДАЧА: Установить функцию "%" 
          // } иначе, если (c == '%') {
            // Знак процента начала и окончания программы НЕ ПОДДЕРЖИВАЕТСЯ.
            // ПРИМЕЧАНИЕ: Возможно, это установлено для того, чтобы сообщать Grbl, когда программа запущена, а не при ручном вводе,
// когда во время выполнения программы системный автоматический запуск цикла будет продолжать выполняться 
            // все до следующего знака "%". Это поможет устранить возобновляющиеся проблемы с определенными
            // функции, которые очищают буфер планировщика для своевременного выполнения его задачи.

          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow. Report error and reset line buffer.
            // Обнаруживаем переполнение строчного буфера. Сообщаем об ошибке и сбрасываем строчный буфер.
            report_status_message(STATUS_OVERFLOW);
            comment = COMMENT_NONE;
            char_counter = 0;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase // Верхний и нижний регистр
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }
      }
    }
    
    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has 
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    // Если в буфере последовательного чтения больше нет символов, которые нужно обработать и выполнить,
    // это означает, что потоковая передача g-кода либо заполнила буфер планировщика, либо 
    // завершена. В любом случае, если включен автоматический запуск цикла, все перемещения в очереди выполняются.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point. // Контрольная точка выполнения команды.
    if (sys.abort) { return; } // Bail to main() program loop to reset system. // Подключитесь к программному циклу main() для сброса системы.
              
  }
  
  return; /* Never reached */ /* Так и не достигнуто */
}


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute realtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to 
// set the system realtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to 
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
// Выполняет команды во время выполнения, когда это необходимо. Это вызывается из различных контрольных точек в главном
// программа, в первую очередь там, где может быть цикл while, ожидающий освобождения места в буфере, или в любой
// точке, где время выполнения с момента последней контрольной точки может составлять более доли секунды.
// Это способ асинхронного выполнения команд в реальном времени (он же многозадачность) с помощью g-кода grbl
// функции синтаксического анализа и планирования. Эта функция также служит интерфейсом для прерываний в 
// установите системные флаги реального времени, когда только основная программа обрабатывает их, устраняя необходимость в
// определяет более дорогостоящие с точки зрения вычислений переменные volatile. Это также обеспечивает управляемый способ 
// выполнения определенных задач без использования двух или более экземпляров одной и той же задачи, таких как planner
// пересчета буфера при задержке подачи или переопределении.
// ПРИМЕЧАНИЕ: Флаги переменных sys_rt_exec_state устанавливаются любым процессом, пошаговыми или последовательными прерываниями, распиновками,
// концевыми выключателями или основной программой.
void protocol_execute_realtime()
{
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times. / Переменная / Temp, позволяющая избежать многократного вызова volatile.

  do { // If system is suspended, suspend loop restarts here. // Если система приостановлена, цикл приостановки перезапускается здесь.
    
  // Check and execute alarms.  // Проверка и выполнение аварийных сигналов.
  rt_exec = sys_rt_exec_alarm; // Copy volatile sys_rt_exec_alarm. // Копируем изменчивый sys_rt_exec_alarm.
  if (rt_exec) { // Enter only if any bit flag is true // Ввод только в том случае, если какой-либо битовый флаг истинен
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    // Системный сигнал тревоги. Все было отключено из-за чего-то, что пошло не так. Отчет
    // источник ошибки для пользователя. В критическом случае Grbl отключается, вводя бесконечный
    // цикл до перезагрузки системы/прерывания.


    sys.state = STATE_ALARM; // Set system alarm state // Установить аварийное состояние системы
    if (rt_exec & EXEC_ALARM_HARD_LIMIT) {
      report_alarm_message(ALARM_HARD_LIMIT_ERROR); 
    } else if (rt_exec & EXEC_ALARM_SOFT_LIMIT) {
      report_alarm_message(ALARM_SOFT_LIMIT_ERROR);
    } else if (rt_exec & EXEC_ALARM_ABORT_CYCLE) {      
      report_alarm_message(ALARM_ABORT_CYCLE);
    } else if (rt_exec & EXEC_ALARM_PROBE_FAIL) {
      report_alarm_message(ALARM_PROBE_FAIL);
    } else if (rt_exec & EXEC_ALARM_HOMING_FAIL) {
      report_alarm_message(ALARM_HOMING_FAIL);
    }
    // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
    // Останавливать все действия при появлении флага критического события. В настоящее время жесткие и мягкие ограничения указывают на это.
    if (rt_exec & EXEC_CRITICAL_EVENT) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      bit_false_atomic(sys_rt_exec_state,EXEC_RESET); // Disable any existing reset // Отключить любой существующий сброс настроек
      do { 
        // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
        // typically occur while unattended or not paying attention. Gives the user time
        // to do what is needed before resetting, like killing the incoming stream. The 
        // same could be said about soft limits. While the position is not lost, the incoming
        // stream could be still engaged and cause a serious crash if it continues afterwards.
        
        // TODO: Allow status reports during a critical alarm. Still need to think about implications of this.
//         if (sys_rt_exec_state & EXEC_STATUS_REPORT) { 
//           report_realtime_status();
//           bit_false_atomic(sys_rt_exec_state,EXEC_STATUS_REPORT); 
//         }
// Ничего. Блокируйте ВСЕ до тех пор, пока пользователь не выполнит сброс или не включит питание. Жесткие ограничения
        // обычно возникают, когда пользователь не обслуживается или не обращает внимания. Дает пользователю время
        // выполнить необходимые действия перед сбросом, например, отключить входящий поток. То
//же самое можно сказать и о мягких ограничениях. Пока позиция не потеряна, входящий поток
        // может быть по-прежнему задействован и вызвать серьезный сбой, если он продолжится после этого.
        
        // TODO: Разрешить отчеты о состоянии во время критической тревоги. Все еще нужно подумать о последствиях этого.
// если (sys_rt_exec_state & EXEC_STATUS_REPORT) _BOS_ 
// отчет_реалтайм_статус();
// бит_фалсе_атомический(sys_rt_exec_state,EXEC_STATUS_REPORT); 
//         }
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    bit_false_atomic(sys_rt_exec_alarm,0xFF); // Clear all alarm flags // Снимите все тревожные флажки
  }
  
  // Check amd execute realtime commands // Проверьте, выполняет ли amd команды в реальном времени
  rt_exec = sys_rt_exec_state; // Copy volatile sys_rt_exec_state. // Скопировать изменчивое состояние sys_rt_exec_state.
  if (rt_exec) { // Enter only if any bit flag is true // Вводите только в том случае, если какой-либо битовый флаг имеет значение true
  
    // Execute system abort. // Выполнить системное прерывание.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true. // Только в том случае, если для этого параметра установлено значение true.
      return; // Nothing else to do but exit. // Ничего другого не остается, как выйти.
    }
    
    // Execute and serial print status // Состояние выполнения и последовательной печати
    if (rt_exec & EXEC_STATUS_REPORT) { 
      report_realtime_status();
      bit_false_atomic(sys_rt_exec_state,EXEC_STATUS_REPORT);
    }
  
    // Execute hold states.
    // NOTE: The math involved to calculate the hold should be low enough for most, if not all, 
    // operational scenarios. Once hold is initiated, the system enters a suspend state to block
    // all main program processes until either reset or resumed.
    // Состояния приостановки выполнения.
    // ПРИМЕЧАНИЕ: Математические расчеты, используемые для расчета задержки, должны быть достаточно низкими для большинства, если не для всех
    // операционных сценариев. Как только запускается задержка, система переходит в состояние приостановки, блокируя
    // все основные программные процессы до тех пор, пока они не будут сброшены или возобновлены.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR)) {
      
      // TODO: CHECK MODE? How to handle this? Likely nothing, since it only works when IDLE and then resets Grbl.
      // TODO: ПРОВЕРЬТЕ РЕЖИМ? Как с этим справиться? Скорее всего, ничего, поскольку он работает только в режиме ожидания, а затем сбрасывает Grbl.


      // State check for allowable states for hold methods.
      // Проверка состояния на наличие допустимых состояний для методов удержания.
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_MOTION_CANCEL | STATE_HOLD | STATE_SAFETY_DOOR))) {

        // If in CYCLE state, all hold states immediately initiate a motion HOLD.
        // Если устройство находится в циклическом состоянии, все состояния удержания немедленно инициируют удержание движения.
        if (sys.state == STATE_CYCLE) {
          st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration. // Уведомить шаговый модуль о необходимости повторного расчета для замедления удержания.
          sys.suspend = SUSPEND_ENABLE_HOLD; // Initiate holding cycle with flag. // Инициировать цикл удержания с помощью флажка.
        }
        // If IDLE, Grbl is not in motion. Simply indicate suspend ready state. // Если режим ожидания неактивен, Grbl не работает. Просто укажите состояние готовности к приостановке.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_ENABLE_READY; }
        
        // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
        // to halt and cancel the remainder of the motion.
        // Выполнить и отметить отмену движения с замедлением и возвратом в режим ожидания. Используется в основном в цикле проверки
        // для остановки и отмены оставшейся части движения.
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
          // to hold the CYCLE. If so, only flag that motion cancel is complete.
          // ОТМЕНА движения происходит только во время ЦИКЛА, но для удержания ЦИКЛА могут быть запущены функции HOLD и SAFETY_DOOR заранее
          //. Если это так, отметьте только, что отмена движения завершена.
          if (sys.state == STATE_CYCLE) { sys.state = STATE_MOTION_CANCEL; }
          sys.suspend |= SUSPEND_MOTION_CANCEL; // Indicate motion cancel when resuming. Special motion complete. // Указывает на отмену движения при возобновлении. Специальное движение завершено.
        }
    
        // Execute a feed hold with deceleration, only during cycle. // Выполняйте удержание подачи с замедлением только во время цикла.
        if (rt_exec & EXEC_FEED_HOLD) {
          // Block SAFETY_DOOR state from prematurely changing back to HOLD. // Блокирует преждевременное изменение состояния SAFETY_DOOR обратно на HOLD.
          if (bit_isfalse(sys.state,STATE_SAFETY_DOOR)) { sys.state = STATE_HOLD; }
        }
  
        // Execute a safety door stop with a feed hold, only during a cycle, and disable spindle/coolant.
        // NOTE: Safety door differs from feed holds by stopping everything no matter state, disables powered
        // devices (spindle/coolant), and blocks resuming until switch is re-engaged. The power-down is 
        // executed here, if IDLE, or when the CYCLE completes via the EXEC_CYCLE_STOP flag.
        // Выполните остановку защитной дверцы с удержанием подачи только во время цикла и отключите шпиндель/охлаждающую жидкость.
        // ПРИМЕЧАНИЕ: Защитная дверца отличается от удержания подачи тем, что останавливает все, независимо от состояния, отключает питание
        // устройств (шпиндель/охлаждающая жидкость) и блокирует возобновление работы до тех пор, пока переключатель не будет снова включен. 
        // Здесь выполняется отключение питания 
        // в режиме ожидания или по завершении ЦИКЛА с помощью флага EXEC_CYCLE_STOP.
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR); 
          // If already in active, ready-to-resume HOLD, set CYCLE_STOP flag to force de-energize.
          // NOTE: Only temporarily sets the 'rt_exec' variable, not the volatile 'rt_exec_state' variable.
          // Если он уже находится в активном режиме ожидания, готовом к возобновлению, установите флаг CYCLE_STOP для принудительного отключения питания.
          // ПРИМЕЧАНИЕ: Только временно устанавливается переменная 'rt_exec', а не переменная volatile 'rt_exec_state'.
          if (sys.suspend & SUSPEND_ENABLE_READY) { bit_true(rt_exec,EXEC_CYCLE_STOP); }
          sys.suspend |= SUSPEND_ENERGIZE;
          sys.state = STATE_SAFETY_DOOR;
        }
         
      }
      bit_false_atomic(sys_rt_exec_state,(EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR));      
    }
          
    // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
    // Выполните запуск цикла, запустив шаговое прерывание, чтобы начать выполнение блоков в очереди.
    if (rt_exec & EXEC_CYCLE_START) {
      // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
      // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
      // Блокировать, если вызывается одновременно с командами удержания: удержание подачи, отмена движения и защитная дверь.
      // Гарантирует, что автоматический запуск цикла не возобновит удержание без явного указания пользователя.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) { 
        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        // NOTE: SAFETY_DOOR is implicitly blocked. It reverts to HOLD when the door is closed.
        // Цикл запускается только в режиме ожидания или когда удержание завершено и готово к возобновлению.
        // ПРИМЕЧАНИЕ: Функция SAFETY_DOOR неявно заблокирована. Она возвращается в режим удержания, когда дверь закрыта.
        if ((sys.state == STATE_IDLE) || ((sys.state & (STATE_HOLD | STATE_MOTION_CANCEL)) && (sys.suspend & SUSPEND_ENABLE_READY))) {
          // Re-energize powered components, if disabled by SAFETY_DOOR.
          // Повторно включите компоненты, находящиеся под напряжением, если они отключены с помощью SAFETY_DOOR.
          if (sys.suspend & SUSPEND_ENERGIZE) { 
            // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
            // Отложенные задачи: Перезапустите шпиндель и подайте охлаждающую жидкость, задержите включение, затем возобновите цикл.
            if (gc_state.modal.spindle != SPINDLE_DISABLE) { 
              spindle_set_state(gc_state.modal.spindle, gc_state.spindle_speed); 
              delay_ms(SAFETY_DOOR_SPINDLE_DELAY); // TODO: Blocking function call. Need a non-blocking one eventually. // TODO: Блокирующий вызов функции. В конце концов, нужен неблокирующий вызов.
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) { 
              coolant_set_state(gc_state.modal.coolant); 
              delay_ms(SAFETY_DOOR_COOLANT_DELAY); // TODO: Blocking function call. Need a non-blocking one eventually.
            }
            // TODO: Install return to pre-park position. // ЗАДАЧА: Установите возврат в положение предварительной парковки.
          }
          // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
          // Запускайте цикл только в том случае, если в буфере планировщика существуют очереди перемещений и перемещение не отменено.
          if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
            sys.state = STATE_CYCLE;
            st_prep_buffer(); // Initialize step segment buffer before beginning cycle. // Инициализируйте буфер сегмента шага перед началом цикла.
            st_wake_up();
          } else { // Otherwise, do nothing. Set and resume IDLE state. // В противном случае ничего не предпринимайте. Установите и возобновите режим ожидания.
            sys.state = STATE_IDLE;
          }
          sys.suspend = SUSPEND_DISABLE; // Break suspend state. // Прервать приостановленное состояние.
        }
      }    
      bit_false_atomic(sys_rt_exec_state,EXEC_CYCLE_START);
    }
    
    // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
    // realtime command execution in the main program, ensuring that the planner re-plans safely.
    // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
    // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.   
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    // Повторно инициализирует циклический план и шаговую систему после приостановки подачи для возобновления. Вызывается
    // выполнением команды в реальном времени в основной программе, обеспечивая безопасное повторное планирование планировщиком.
    // ПРИМЕЧАНИЕ: Переменные алгоритма Брезенхэма по-прежнему сохраняются как в планировщике, так и в stepper
    // повторная инициализация цикла. Путь stepper должен продолжаться точно так же, как если бы ничего не произошло.   
    // ПРИМЕЧАНИЕ: EXEC_CYCLE_STOP устанавливается шаговой подсистемой при завершении цикла или удержании подачи.
    if (rt_exec & EXEC_CYCLE_STOP) {
      if (sys.state & (STATE_HOLD | STATE_SAFETY_DOOR) && !(sys.soft_limit)) {
        // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
        // has issued a resume command or reset.
        // Удержание завершено. Устанавливается для обозначения готовности к возобновлению.  Оставайтесь в состоянии удержания или ОТКРЫТИЯ, пока пользователь
        // не подаст команду возобновления или сброса.
        if (sys.suspend & SUSPEND_ENERGIZE) { // De-energize system if safety door has been opened. // Обесточьте систему, если была открыта защитная дверь.
          spindle_stop();
          coolant_stop();
        }
        bit_true(sys.suspend,SUSPEND_ENABLE_READY);
      } else { // Motion is complete. Includes CYCLE, HOMING, and MOTION_CANCEL states. // Движение завершено. Включает состояния ЦИКЛА, ВОЗВРАТА в исходное положение и остановки движения.
        sys.suspend = SUSPEND_DISABLE;
        sys.state = STATE_IDLE;
      }
      bit_false_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP);
    }
    
  }

  // Overrides flag byte (sys.override) and execution should be installed here, since they 
  // are realtime and require a direct and controlled interface to the main stepper program.

  // Reload step segment buffer
  // Здесь следует установить байт флага переопределения (sys.override) и execution, поскольку они 
  // работают в режиме реального времени и требуют прямого и управляемого интерфейса с основной программой stepper.

  // Перезагрузить буфер сегмента шага
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_MOTION_CANCEL | STATE_SAFETY_DOOR | STATE_HOMING)) { st_prep_buffer(); }  
  
  // If safety door was opened, actively check when safety door is closed and ready to resume.
  // NOTE: This unlocks the SAFETY_DOOR state to a HOLD state, such that CYCLE_START can activate a resume.
  // Если защитная дверь была открыта, активно проверяйте, закрыта ли защитная дверь и готова ли она к возобновлению работы.
  // ПРИМЕЧАНИЕ: Это переводит состояние SAFETY_DOOR в состояние ОЖИДАНИЯ, так что CYCLE_START может активировать возобновление.
  if (sys.state == STATE_SAFETY_DOOR) { 
    if (bit_istrue(sys.suspend,SUSPEND_ENABLE_READY)) { 
      if (!(system_check_safety_door_ajar())) {
        sys.state = STATE_HOLD; // Update to HOLD state to indicate door is closed and ready to resume. // Переведите в состояние ОЖИДАНИЯ, чтобы указать, что дверь закрыта и готова к возобновлению работы.
      }
    }
  }

  } while(sys.suspend); // Check for system suspend state before exiting. // Перед выходом проверьте состояние приостановки работы системы.
  
}  


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
// Блокировать до тех пор, пока не будут выполнены все буферизованные шаги или пока они не перейдут в состояние цикла. Работает с удержанием подачи
// во время вызова синхронизации, если это должно произойти. Также ожидает завершения цикла очистки.
void protocol_buffer_synchronize()
{
  // If system is queued, ensure cycle resumes if the auto start flag is present.
  // Если система поставлена в очередь, убедитесь, что цикл возобновляется при наличии флага автоматического запуска.
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // Check and execute run-time commands // Проверка и выполнение команд во время выполнения
    if (sys.abort) { return; } // Check for system abort // Проверьте, не прервалась ли система
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// Auto-cycle start has two purposes: 1. Resumes a plan_synchronize() call from a function that
// requires the planner buffer to empty (spindle enable, dwell, etc.) 2. As a user setting that 
// automatically begins the cycle when a user enters a valid motion command manually. This is 
// intended as a beginners feature to help new users to understand g-code. It can be disabled
// as a beginner tool, but (1.) still operates. If disabled, the operation of cycle start is
// manually issuing a cycle start command whenever the user is ready and there is a valid motion 
// command in the planner queue.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes 
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming 
// is finished, single commands), a command that needs to wait for the motions in the buffer to 
// execute calls a buffer sync, or the planner buffer is full and ready to go.
// Автоматический запуск цикла имеет две цели: 1. Возобновляет вызов plan_synchronize() из функции, которая
// требует, чтобы буфер планировщика был очищен (включение шпинделя, задержка и т.д.). 2. В качестве пользовательской настройки, которая 
// автоматически запускает цикл, когда пользователь вводит правильную команду перемещения вручную. Это 
// предназначена для начинающих пользователей, чтобы помочь им разобраться в g-коде. Ее можно отключить
// предназначена для начинающих, но (1.) все еще работает. Если функция запуска цикла отключена,она 
// запускает команду запуска цикла вручную всякий раз, когда пользователь готов и есть допустимое движение 
// команда в очереди планировщика.
// ПРИМЕЧАНИЕ: Эта функция вызывается только из основного цикла, буферной синхронизации и mc_line() и выполняется 
// при выполнении одного из этих условий соответственно: больше не отправляются блоки (т.е. потоковая передача 
// is finished, одиночные команды), команда, которая должна дождаться выполнения операций в буфере 
// execute вызывает синхронизацию буфера, или буфер планировщика заполнен и готов к работе.
void protocol_auto_cycle_start() { bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); } 
