/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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
#include "grbl.hpp"

#define LINE_FLAG_OVERFLOW bit(0)

static void protocol_exec_rt_suspend();


/*
  ГЛАВНЫЙ ЦИКЛ GRBL:
*/
void protocol_main_loop(char* line, uint8_t* line_flags)
{
  // Выполнить проверки оборудования, чтобы убедиться, что всё готово к работе.
  #ifdef CHECK_LIMITS_AT_INIT
    if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE)) {
      if (limits_get_state()) {
        sys.state = STATE_ALARM; // Убедиться, что состояние тревоги активно.
        report_feedback_message(MESSAGE_CHECK_LIMITS);
      }
    }
  #endif
  // Проверить и сообщить о состоянии тревоги после сброса, ошибки или начального включения.
  // ПРИМЕЧАНИЕ: Режим сна отключает драйверы шаговых двигателей, и положение не гарантируется.
  // Переинициализировать состояние сна как режим ТРЕВОГИ, чтобы пользователь выполнил
  // перемещение в ноль (homing) или подтвердил сброс.
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // Убедиться, что состояние тревоги установлено.
  } else {
    // Проверить, открыта ли защитная дверца.
    sys.state = STATE_IDLE;
    if (system_check_safety_door_ajar()) {
      bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // Войти в режим защитной дверцы. Должен вернуться в состояние IDLE.
    }
    // Всё готово к работе!
    system_execute_startup(line); // Выполнить стартовый скрипт.
  }

  //
  // ---------------------------------------------------------------------------------
  // Основная петля. При прерывании работы возвращается к функции main.
  // здесь станок находится в ожидании до получения каких-либо команд.
  // ---------------------------------------------------------------------------------


  uint8_t char_counter = 0;
  uint8_t client = CLIENT_SERIAL;
  for (;;) {
    // Обработать одну строку входящих последовательных данных, по мере их поступления.
    // Выполняет начальную фильтрацию: удаляет пробелы и комментарии, приводит буквы к верхнему регистру.
    if(*line_flags & LINE_FLAG_LINE_READ){
          protocol_execute_realtime(); // Точка проверки команд реального времени.
          if (sys.abort) { return; } // Выход из цикла при системном прерывании

          line[char_counter] = 0; // Установить символ завершения строки.
          #ifdef REPORT_ECHO_LINE_RECEIVED
            report_echo_line_received(line, client);
          #endif

          // Направить и выполнить одну строку форматированного ввода и сообщить статус выполнения.
          if (*line_flags & LINE_FLAG_OVERFLOW) {
            // Сообщить об ошибке переполнения строки.
            report_status_message(STATUS_OVERFLOW, client);
          } else if (line[0] == 0) {
            // Пустая строка или строка комментария. Для целей синхронизации.
            report_status_message(STATUS_OK, client);
          } else if (line[0] == '$') {
            // Системная команда Grbl '$'
            report_status_message(system_execute_line(line, client), client);
          } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
            // Всё остальное — gcode. Блокировать, если в режиме тревоги или JOG.
            report_status_message(STATUS_SYSTEM_GC_LOCK, client);
          } else {
            // Разобрать и выполнить g-code блок.
            report_status_message(gc_execute_line(line, client), client);
          }
          char_counter = 0;
        // else {
        // }
        delay(0);
    }

    // Если в буфере последовательного порта больше нет символов для обработки и выполнения,
    // это означает, что поток g-code либо заполнил буфер планировщика, либо завершён.
    // В любом случае, если автозапуск цикла включён, запустить поставленные в очередь перемещения.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Точка проверки команд реального времени.
    if (sys.abort) { return; } // Выход в main() для сброса системы.
    delay(0);
  }

  return; /* Никогда не достигается */
}


// Блокировать выполнение, пока все шаги в буфере не будут выполнены или не войдут в состояние цикла.
// Работает с удержанием подачи (feed hold) во время вызова синхронизации, если это происходит.
// Также ожидает чистого завершения цикла.
void protocol_buffer_synchronize()
{
  // Если система в очереди, убедиться, что цикл возобновится, если установлен флаг автозапуска.
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // Проверить и выполнить команды реального времени
    if (sys.abort) { return; } // Проверить системное прерывание
    delay(0);
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// Автозапуск цикла срабатывает, когда есть готовое к выполнению движение и главная программа
// не занята активным разбором команд.
// ПРИМЕЧАНИЕ: Эта функция вызывается из главного цикла, синхронизации буфера и mc_line().
// Выполняется, когда выполняется одно из условий: больше нет отправленных блоков (поток завершён,
// одиночные команды), команда, ожидающая выполнения движений в буфере, вызывает синхронизацию,
// или буфер планировщика заполнен и готов к работе.
void protocol_auto_cycle_start()
{
  if (plan_get_current_block() != NULL) { // Проверить, есть ли блоки в буфере.
    system_set_exec_state_flag(EXEC_CYCLE_START); // Если есть — выполнить их!
  }
}


// Эта функция является общим интерфейсом к системе выполнения команд реального времени Grbl.
// Она вызывается из различных контрольных точек в главной программе, в первую очередь там,
// где может быть цикл while, ожидающий освобождения места в буфере, или в любой точке,
// где время выполнения с последней контрольной точки может превышать долю секунды.
// Это способ асинхронного выполнения команд реального времени (многозадачность)
// вместе с функциями разбора g-code и планирования Grbl. Эта функция также служит
// интерфейсом для прерываний, устанавливающих флаги реального времени системы,
// которые обрабатываются только главной программой, устраняя необходимость в более
// вычислительно-затратных volatile-переменных. Это также обеспечивает контролируемый
// способ выполнения определённых задач без создания двух или более экземпляров
// одной и той же задачи, например, пересчёт буфера планировщиком при удержании подачи
// или переопределениях.
// ПРИМЕЧАНИЕ: Флаги переменной sys_rt_exec_state устанавливаются любым процессом,
// прерываниями шагового двигателя или последовательного порта, выводами,
// концевыми выключателями или главной программой.
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}

// Выполняет команды реального времени, когда это необходимо. Эта функция в основном работает
// как конечный автомат Grbl и управляет различными функциями реального времени.
// ПРИМЕЧАНИЕ: Не изменяйте это, если вы точно не знаете, что делаете!
void protocol_exec_rt_system()
{
  uint8_t rt_exec; // Временная переменная для избежания множественного обращения к volatile.
  rt_exec = sys_rt_exec_alarm; // Копировать volatile sys_rt_exec_alarm.
  if (rt_exec) { // Войти только если установлен хотя бы один битовый флаг
    // Системная тревога. Всё остановлено из-за серьёзной ошибки. Сообщить
    // пользователю источник ошибки. При критической ошибке Grbl отключается,
    // входя в бесконечный цикл до сброса/прерывания системы.
    sys.state = STATE_ALARM; // Установить состояние тревоги
    report_alarm_message(rt_exec);
    // Остановить всё при флаге критического события. В настоящее время это флаги
    // жёстких и программных концевых выключателей.
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // Отключить любой существующий сброс
      do {
        delay(0);
        // Блокировать всё, кроме сброса и отчётов о состоянии, пока пользователь
        // не выполнит сброс или перезагрузку питания. Жёсткие концевые обычно
        // срабатывают, когда за станком не следят. Даёт пользователю и GUI время
        // сделать необходимое перед сбросом, например, остановить входящий поток.
        // То же самое можно сказать о программных концевых. Хотя позиция не теряется,
        // продолжение потока может привести к серьёзной аварии.
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // Очистить тревогу
  }

  rt_exec = sys_rt_exec_state; // Копировать volatile sys_rt_exec_state.
  if (rt_exec) {

    // Выполнить системное прерывание.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Единственное место, где это устанавливается в true.
      return; // Больше нечего делать, кроме выхода.
    }

    // Выполнить и вывести через последовательный порт отчёт о состоянии
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status(CLIENT_ALL);
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // ПРИМЕЧАНИЕ: Как только удержание инициировано, система немедленно входит в состояние
    // приостановки, блокируя все процессы главной программы до сброса или возобновления.
    // Это обеспечивает безопасное завершение удержания.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

      // Проверка состояния на допустимость для методов удержания.
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {

        // Если в состоянии CYCLE или JOG, немедленно инициировать удержание движения.
        if (sys.state & (STATE_CYCLE | STATE_JOG)) {
          if (!(sys.suspend & (SUSPEND_MOTION_CANCEL | SUSPEND_JOG_CANCEL))) { // Блокировать, если уже удержание.
            st_update_plan_block_parameters(); // Уведомить шаговый модуль о пересчёте для замедления удержания.
            sys.step_control = STEP_CONTROL_EXECUTE_HOLD; // Инициировать состояние приостановки с активным флагом.
            if (sys.state == STATE_JOG) { // JOG отменяется при любом событии удержания, кроме сна.
              if (!(rt_exec & EXEC_SLEEP)) { sys.suspend |= SUSPEND_JOG_CANCEL; }
            }
          }
        }
        // Если IDLE, Grbl не движется. Просто указать состояние приостановки, удержание завершено.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // Выполнить и отметить отмену движения с замедлением и возвратом в IDLE.
        // Используется в основном циклом зондирования для остановки и отмены остатка движения.
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // MOTION_CANCEL происходит только во время CYCLE, но HOLD и SAFETY_DOOR могут быть
          // инициированы заранее для удержания CYCLE. Отмена движения действительна только
          // для одного блока планировщика, в то время как отмена JOG обрабатывает несколько блоков.
          if (!(sys.state & STATE_JOG)) { sys.suspend |= SUSPEND_MOTION_CANCEL; } // ПРИМЕЧАНИЕ: Состояние STATE_CYCLE.
        }

        // Выполнить удержание подачи с замедлением, если требуется. Затем приостановить систему.
        if (rt_exec & EXEC_FEED_HOLD) {
          // Блокировать переход состояний SAFETY_DOOR, JOG и SLEEP в состояние HOLD.
          if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP))) { sys.state = STATE_HOLD; }
        }

        // Выполнить останов защитной дверцы с удержанием подачи и отключением шпинделя/СОЖ.
        // ПРИМЕЧАНИЕ: Защитная дверца отличается от удержания подачи тем, что останавливает всё
        // независимо от состояния, отключает питаемые устройства (шпиндель/СОЖ) и блокирует
        // возобновление до повторного замыкания выключателя.
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR);
          // Если выполняется JOG, блокировать методы защитной дверцы до завершения отмены JOG.
          if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
            // Проверить, не открылась ли дверца снова во время восстановления парковки.
            // Игнорировать, если уже втягивание, парковка или сон.
            if (sys.state == STATE_SAFETY_DOOR) {
              if (sys.suspend & SUSPEND_INITIATE_RESTORE) { // Активное восстановление
                #ifdef PARKING_ENABLE
                  // Установить удержание и сбросить соответствующие флаги для перезапуска парковки.
                  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    st_update_plan_block_parameters(); // Уведомить шаговый модуль о пересчёте для замедления.
                    sys.step_control = (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION);
                    sys.suspend &= ~(SUSPEND_HOLD_COMPLETE);
                  } // иначе активно NO_MOTION.
                #endif
                sys.suspend &= ~(SUSPEND_RETRACT_COMPLETE | SUSPEND_INITIATE_RESTORE | SUSPEND_RESTORE_COMPLETE);
                sys.suspend |= SUSPEND_RESTART_RETRACT;
              }
            }
            if (sys.state != STATE_SLEEP) { sys.state = STATE_SAFETY_DOOR; }
          }
          // ПРИМЕЧАНИЕ: Этот флаг не меняется при закрытии дверцы, в отличие от sys.state.
          // Гарантирует выполнение парковочных движений, если выключатель дверцы замкнётся
          // и состояние вернётся в HOLD.
          sys.suspend |= SUSPEND_SAFETY_DOOR_AJAR;
        }

      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP;
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    // Выполнить запуск цикла, запуская прерывание шагового двигателя для выполнения блоков в очереди.
    if (rt_exec & EXEC_CYCLE_START) {
      // Блокировать, если вызвано одновременно с командами удержания: feed hold, motion cancel, safety door.
      // Гарантирует, что автозапуск цикла не возобновит удержание без явного ввода пользователя.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
        // Возобновить состояние дверцы, когда парковочное движение втянуто и дверца закрыта.
        if ((sys.state == STATE_SAFETY_DOOR) && !(sys.suspend & SUSPEND_SAFETY_DOOR_AJAR)) {
          if (sys.suspend & SUSPEND_RESTORE_COMPLETE) {
            sys.state = STATE_IDLE; // Установить IDLE для немедленного возобновления цикла.
          } else if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
            // Флаг для повторного включения питаемых компонентов и восстановления исходной позиции.
            // ПРИМЕЧАНИЕ: Для возобновления после защитной дверцы выключатель должен быть замкнут
            // (состояние HOLD), и втягивание должно быть завершено. Для восстановления нормальной
            // работы процедуры восстановления инициируются следующим флагом. После их завершения
            // будет автоматически вызван CYCLE_START для возобновления и выхода из приостановки.
            sys.suspend |= SUSPEND_INITIATE_RESTORE;
          }
        }
        // Запуск цикла только когда IDLE или когда удержание завершено и готово к возобновлению.
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; // Установить для восстановления в процедуре приостановки и последующего запуска цикла.
          } else {
            // Запустить цикл, только если есть движения в очереди планировщика и движение не отменено.
            sys.step_control = STEP_CONTROL_NORMAL_OP; // Восстановить нормальное управление шагами
            if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
              sys.suspend = SUSPEND_DISABLE; // Снять состояние приостановки.
              sys.state = STATE_CYCLE;
              st_prep_buffer(); // Инициализировать буфер сегментов шагов перед началом цикла.
              st_wake_up();
            } else { // Иначе ничего не делать. Установить и возобновить состояние IDLE.
              sys.suspend = SUSPEND_DISABLE; // Снять состояние приостановки.
              sys.state = STATE_IDLE;
            }
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // Переинициализирует план цикла и шаговую систему после удержания подачи для возобновления.
      // Вызывается выполнением команд реального времени в главной программе, обеспечивая
      // безопасное перепланирование.
      // ПРИМЕЧАНИЕ: Переменные алгоритма Брезенхема сохраняются при переинициализации
      // как планировщика, так и шагового цикла. Путь шагового двигателя должен продолжиться
      // точно так, как если бы ничего не произошло.
      // ПРИМЕЧАНИЕ: EXEC_CYCLE_STOP устанавливается подсистемой шагового двигателя,
      // когда цикл или удержание подачи завершены.
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
        // Удержание завершено. Установить флаг готовности к возобновлению.
        // Оставаться в состояниях HOLD или DOOR, пока пользователь не даст команду возобновления или сброс.
        plan_cycle_reinitialize();
        if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { sys.suspend |= SUSPEND_HOLD_COMPLETE; }
        bit_false(sys.step_control,(STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION));
      } else {
        // Движение завершено. Включает состояния CYCLE/JOG/HOMING и события отмены jog/motion/soft limit.
        // ПРИМЕЧАНИЕ: Отмена движения и JOG немедленно возвращаются в IDLE после завершения удержания.
        if (sys.suspend & SUSPEND_JOG_CANCEL) {   // Для отмены JOG очистить буферы и синхронизировать позиции.
          sys.step_control = STEP_CONTROL_NORMAL_OP;
          plan_reset();
          st_reset();
          gc_sync_position();
          plan_sync_position();
        }
        if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) { // Происходит только когда дверца открывается во время JOG.
          sys.suspend &= ~(SUSPEND_JOG_CANCEL);
          sys.suspend |= SUSPEND_HOLD_COMPLETE;
          sys.state = STATE_SAFETY_DOOR;
        } else {
          sys.suspend = SUSPEND_DISABLE;
          sys.state = STATE_IDLE;
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // Выполнить переопределения (override).
  rt_exec = sys_rt_exec_motion_override; // Копировать volatile sys_rt_exec_motion_override
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // Очистить все флаги переопределения движения.

    uint8_t new_f_override =  sys.f_override;
    if (rt_exec & EXEC_FEED_OVR_RESET) { new_f_override = DEFAULT_FEED_OVERRIDE; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_PLUS) { new_f_override += FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_COARSE_MINUS) { new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_PLUS) { new_f_override += FEED_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_FEED_OVR_FINE_MINUS) { new_f_override -= FEED_OVERRIDE_FINE_INCREMENT; }
    new_f_override = min((int)new_f_override,MAX_FEED_RATE_OVERRIDE);
    new_f_override = max((int)new_f_override,MIN_FEED_RATE_OVERRIDE);

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // Установить для немедленного отчёта об изменении
      plan_update_velocity_profile_parameters();
      plan_cycle_reinitialize();
    }
  }

  rt_exec = sys_rt_exec_accessory_override;
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); // Очистить все флаги переопределения принадлежностей.

    // ПРИМЕЧАНИЕ: В отличие от переопределений движения, переопределения шпинделя
    // не требуют переинициализации планировщика.
    uint8_t last_s_override =  sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min((int)last_s_override,MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max((int)last_s_override,MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
      sys.spindle_speed_ovr = last_s_override;
      // ПРИМЕЧАНИЕ: Переопределения скорости шпинделя во время HOLD обрабатываются функцией приостановки.
      if (sys.state == STATE_IDLE) { spindle_set_state(gc_state.modal.spindle, gc_state.spindle_speed); }
  	else { bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); }
      sys.report_ovr_counter = 0; // Установить для немедленного отчёта об изменении
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
      // Переопределение останова шпинделя разрешено только в состоянии HOLD.
      // ПРИМЕЧАНИЕ: Счётчики отчётов устанавливаются в spindle_set_state() при выполнении останова шпинделя.
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }

    // ПРИМЕЧАНИЕ: Поскольку состояние СОЖ всегда выполняет синхронизацию планировщика при изменении,
    // текущее состояние выполнения можно определить по состоянию парсера.
    // ПРИМЕЧАНИЕ: Переопределения СОЖ работают только в состояниях IDLE, CYCLE, HOLD и JOG.
    if (rt_exec & (EXEC_COOLANT_FLOOD_OVR_TOGGLE | EXEC_COOLANT_MIST_OVR_TOGGLE)) {
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_JOG))) {
        uint8_t coolant_state = gc_state.modal.coolant;
        #ifdef ENABLE_M7
          if (rt_exec & EXEC_COOLANT_MIST_OVR_TOGGLE) {
            if (coolant_state & COOLANT_MIST_ENABLE) { bit_false(coolant_state,COOLANT_MIST_ENABLE); }
            else { coolant_state |= COOLANT_MIST_ENABLE; }
          }
          if (rt_exec & EXEC_COOLANT_FLOOD_OVR_TOGGLE) {
            if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
            else { coolant_state |= COOLANT_FLOOD_ENABLE; }
          }
        #else
          if (coolant_state & COOLANT_FLOOD_ENABLE) { bit_false(coolant_state,COOLANT_FLOOD_ENABLE); }
          else { coolant_state |= COOLANT_FLOOD_ENABLE; }
        #endif
        coolant_set_state(coolant_state); // Счётчик отчёта устанавливается в coolant_set_state().
        gc_state.modal.coolant = coolant_state;
      }
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif

  // Перезагрузить буфер сегментов шагов
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG)) {
    st_prep_buffer();
  }

}


// Обрабатывает процедуры приостановки системы Grbl, такие как удержание подачи,
// защитная дверца и парковочное движение. Система входит в этот цикл, создаёт
// локальные переменные для задач приостановки и возвращается к функции, вызвавшей
// приостановку, чтобы Grbl возобновил нормальную работу.
// Эта функция написана так, чтобы способствовать пользовательским парковочным движениям.
static void protocol_exec_rt_suspend()
{
  #ifdef PARKING_ENABLE
    // Объявить и инициализировать локальные переменные парковки
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
    #ifdef USE_LINE_NUMBERS
      pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
    #endif
  #endif

  plan_block_t *block = plan_get_current_block();
  uint8_t restore_condition;
  #ifdef VARIABLE_SPINDLE
    float restore_spindle_speed;
    if (block == NULL) {
      restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
      restore_spindle_speed = gc_state.spindle_speed;
    } else {
      restore_condition = (block->condition & PL_COND_SPINDLE_MASK) | coolant_get_state();
      restore_spindle_speed = block->spindle_speed;
    }
    #ifdef DISABLE_LASER_DURING_HOLD
      if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
        system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP);
      }
    #endif
  #else
    if (block == NULL) { restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant); }
    else { restore_condition = (block->condition & PL_COND_SPINDLE_MASK) | coolant_get_state(); }
  #endif

  while (sys.suspend) {
    if (sys.abort) { return; }

    // Блокировать, пока начальное удержание не завершено и станок не остановил движение.
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      // Менеджер парковки. Обрабатывает включение/отключение питания, проверку состояния
      // выключателей и парковочные движения для состояний защитной дверцы и сна.
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {

        // Обрабатывает втягивание и отключение питания.
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {

          // Убедиться, что предыдущее переопределение останова шпинделя отключено
          // при запуске процедуры защитной дверцы.
          sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED;

          #ifndef PARKING_ENABLE

            spindle_set_state(SPINDLE_DISABLE,0.0); // Отключить питание
            coolant_set_state(COOLANT_DISABLE);     // Отключить питание

          #else

            // Получить текущую позицию и сохранить место восстановления и точку втягивания шпинделя.
            system_convert_array_steps_to_mpos(parking_target,sys_position);
            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              memcpy(restore_target,parking_target,sizeof(parking_target));
              retract_waypoint += restore_target[PARKING_AXIS];
              retract_waypoint = min(retract_waypoint,PARKING_TARGET);
            }

            // Выполнить медленное вытягивание парковочного втягивания. Парковка требует
            // включённого homing, текущая позиция не должна превышать целевую позицию парковки,
            // и лазерный режим должен быть отключён.
            // ПРИМЕЧАНИЕ: Состояние останется DOOR до завершения отключения питания и втягивания.
            #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
            if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags,BITFLAG_LASER_MODE) &&
                            (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
            #else
            if ((bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) &&
                            (parking_target[PARKING_AXIS] < PARKING_TARGET) &&
                            bit_isfalse(settings.flags,BITFLAG_LASER_MODE)) {
            #endif
              // Втянуть шпиндель на расстояние вытягивания. Убедиться, что движение втягивания
              // уходит от заготовки и не превышает целевую позицию парковки.
              if (parking_target[PARKING_AXIS] < retract_waypoint) {
                parking_target[PARKING_AXIS] = retract_waypoint;
                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Сохранить состояние принадлежностей
                pl_data->spindle_speed = restore_spindle_speed;
                mc_parking_motion(parking_target, pl_data);
              }

              // ПРИМЕЧАНИЕ: Очистить состояние принадлежностей после втягивания
              // и после прерванного восстановления.
              pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
              pl_data->spindle_speed = 0.0;
              spindle_set_state(SPINDLE_DISABLE,0.0); // Отключить питание
              coolant_set_state(COOLANT_DISABLE); // Отключить питание

              // Выполнить быстрое парковочное втягивание к целевой позиции парковки.
              if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                parking_target[PARKING_AXIS] = PARKING_TARGET;
                pl_data->feed_rate = PARKING_RATE;
                mc_parking_motion(parking_target, pl_data);
              }

            } else {

              // Парковочное движение невозможно. Просто отключить шпиндель и СОЖ.
              // ПРИМЕЧАНИЕ: Лазерный режим не запускает парковку, чтобы лазер остановился немедленно.
              spindle_set_state(SPINDLE_DISABLE,0.0); // Отключить питание
              coolant_set_state(COOLANT_DISABLE);     // Отключить питание

            }

          #endif

          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {


          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            // Шпиндель и СОЖ уже должны быть остановлены, но повторить для надёжности.
            spindle_set_state(SPINDLE_DISABLE,0.0); // Отключить питание
            coolant_set_state(COOLANT_DISABLE); // Отключить питание
            st_go_idle(); // Отключить шаговые двигатели
            while (!(sys.abort)) {
              protocol_exec_rt_system();
              delay(0);
              } // Ничего не делать до сброса.
            return; // Получено прерывание. Вернуться для переинициализации.
          }

          // Разрешить возобновление после парковки/защитной дверцы.
          // Активно проверять, закрыта ли дверца и готова ли к возобновлению.
          if (sys.state == STATE_SAFETY_DOOR) {
            if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); // Сбросить флаг открытой дверцы, обозначая готовность к возобновлению.
            }
          }

          // Обрабатывает восстановление парковки и возобновление после защитной дверцы.
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              // Выполнить быстрое восстановление до позиции вытягивания. Парковка требует homing.
              // ПРИМЕЧАНИЕ: Состояние останется DOOR до завершения отключения и втягивания.
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                // Проверить, что движение не уходит ниже позиции вытягивания.
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

            // Отложенные задачи: перезапустить шпиндель и СОЖ, задержка включения, затем возобновить цикл.
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              // Блокировать, если дверца снова открылась во время предыдущих действий восстановления.
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                  // В лазерном режиме игнорировать задержку раскрутки шпинделя. Включить лазер при старте цикла.
                  bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
                } else {
                  spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
                  delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DELAY_MODE_SYS_SUSPEND);
                }
              }
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) {
              // Блокировать, если дверца снова открылась во время предыдущих действий восстановления.
              if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                // ПРИМЕЧАНИЕ: Лазерный режим учитывает эту задержку. Вытяжная система часто управляется этим пином.
                coolant_set_state((restore_condition & (PL_COND_FLAG_COOLANT_FLOOD | PL_COND_FLAG_COOLANT_MIST)));
                delay_sec(SAFETY_DOOR_COOLANT_DELAY, DELAY_MODE_SYS_SUSPEND);
              }
            }

            #ifdef PARKING_ENABLE
              // Выполнить медленное погружение от позиции вытягивания до позиции возобновления.
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                // Блокировать, если дверца снова открылась во время предыдущих действий восстановления.
                if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                  // Независимо от того, было ли парковочное втягивание допустимым/безопасным,
                  // восстановление должно быть логически допустимым — либо возвратом в исходную
                  // позицию через допустимое пространство станка, либо отсутствием движения.
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
        	pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Восстановить состояние принадлежностей
        	pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); // Установить для возобновления программы.
            }
          }

        }


      } else {

        // Менеджер удержания подачи. Управляет состояниями переопределения останова шпинделя.
        // ПРИМЕЧАНИЕ: Удержание считается завершённым по проверке условия в начале процедуры приостановки.
        if (sys.spindle_stop_ovr) {
          // Обрабатывает начало останова шпинделя
          if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_INITIATE) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              spindle_set_state(SPINDLE_DISABLE,0.0); // Отключить питание
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_ENABLED; // Установить состояние переопределения останова как включённое.
            } else {
              sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // Очистить состояние переопределения останова
            }
          // Обрабатывает восстановление состояния шпинделя
          } else if (sys.spindle_stop_ovr & (SPINDLE_STOP_OVR_RESTORE | SPINDLE_STOP_OVR_RESTORE_CYCLE)) {
            if (gc_state.modal.spindle != SPINDLE_DISABLE) {
              report_feedback_message(MESSAGE_SPINDLE_RESTORE);
              if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
                // В лазерном режиме игнорировать задержку раскрутки. Включить лазер при старте цикла.
                bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
              } else {
                spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
              }
            }
            if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_RESTORE_CYCLE) {
              system_set_exec_state_flag(EXEC_CYCLE_START);  // Установить для возобновления программы.
            }
            sys.spindle_stop_ovr = SPINDLE_STOP_OVR_DISABLED; // Очистить состояние переопределения останова
          }
        } else {
          // Обрабатывает состояние шпинделя во время удержания.
          // ПРИМЕЧАНИЕ: Переопределения скорости шпинделя могут изменяться во время удержания.
          // ПРИМЕЧАНИЕ: STEP_CONTROL_UPDATE_SPINDLE_PWM автоматически сбрасывается при возобновлении в генераторе шагов.
          if (bit_istrue(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
            spindle_set_state((restore_condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)), restore_spindle_speed);
            bit_false(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
          }
        }

      }
    }

    protocol_exec_rt_system();
    delay(0);
  }
}
