/*
  system.c - Обрабатывает системные команды и процессы реального времени
  Часть Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl - свободное программное обеспечение: вы можете распространять и/или изменять
  его на условиях Стандартной общественной лицензии GNU в том виде, в каком
  она была опубликована Фондом свободного программного обеспечения; либо
  версии 3 лицензии, либо (по вашему выбору) любой более поздней версии.

  Grbl распространяется в надежде, что он будет полезен,
  но БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ; даже без подразумеваемой гарантии
  ТОВАРНОГО СОСТОЯНИЯ ПРИ ПРОДАЖЕ или ПРИГОДНОСТИ ДЛЯ ОПРЕДЕЛЕННОЙ ЦЕЛИ.
  Подробнее см. в Стандартной общественной лицензии GNU.

  Вы должны были получить копию Стандартной общественной лицензии GNU
  вместе с Grbl. Если это не так, см. <http://www.gnu.org/licenses/>.
*/

#include "grbl.hpp"
// #include <SPI.h>

void system_init()
{
  // SPI.begin();
  // SPI.setHwCs(true);
  // SPI.setFrequency(F_STEPPER_TIMER);

  // Отключить все управляющие входы
  // CONTROL_PORT &= ~CONTROL_MASK;
  // SPI.write(regs.data);

  // Подключить прерывание к входному пину управления
  // pinMode(CONTROL_INPUT_GPIO_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(CONTROL_INPUT_GPIO_PIN), pin_control_vect, CHANGE);
}

uint8_t system_control_get_state(){}

// Возвращает состояние управляющих пинов в виде битового поля uint8. Каждый бит указывает состояние входного пина,
// где 1 означает срабатывание, а 0 - отсутствие срабатывания. Применяется инвертирующая маска.
// Организация битового поля определяется CONTROL_PIN_INDEX в заголовочном файле.
// IRAM_ATTR - указывает, что функцию необходимо поместить в RAM память для более быстрого выполнения
// Переделать функцию
// IRAM_ATTR uint8_t system_control_get_state()
// {
//   uint8_t control_state = 0;
//   uint8_t pin = (CONTROL_PORT_INPUTS & CONTROL_MASK);
//   #ifdef INVERT_CONTROL_PIN_MASK
//     pin ^= INVERT_CONTROL_PIN_MASK;
//   #endif
//   if (pin) {
//     #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
//       if (bit_istrue(pin,(1<<CONTROL_SAFETY_DOOR_BIT))) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
//     #endif
//     if (bit_istrue(pin,(1<<CONTROL_RESET_BIT))) { control_state |= CONTROL_PIN_INDEX_RESET; }
//     if (bit_istrue(pin,(1<<CONTROL_FEED_HOLD_BIT))) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
//     if (bit_istrue(pin,(1<<CONTROL_CYCLE_START_BIT))) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
//   }
//   return(control_state);
// }


// Прерывание по спаду для команд, управляемых пинами, таких как запуск цикла, удержание подачи и сброс.
// Устанавливает только переменную выполнения команд реального времени, чтобы основная программа
// выполнила их, когда будет готова. Работает точно так же, как символьные команды реального времени,
// когда они извлекаются непосредственно из входящего последовательного потока данных.
static uint8_t control_input_pivot;

void pin_control_vect() {
}

// IRAM_ATTR void pin_control_vect() {
//   uint8_t control_state = 0;

//   if (!control_input_pivot && !CONTROL_PORT_INPUTS) {
//     cli();
//     // Пройти по всем управляющим входным пинам, устанавливая только один бит в 0 за раз
//     // и проверить, выключен ли физический пин для этой комбинации
//     for (control_input_pivot = 1; control_input_pivot; control_input_pivot <<= 1) {
//       if (CONTROL_MASK & control_input_pivot) {
//         CONTROL_PORT |= CONTROL_MASK;
//         CONTROL_PORT &= ~control_input_pivot;
//         SPI.write32(regs.data);
//         if (!GPIP(CONTROL_INPUT_GPIO_PIN)) {
//           control_state |= control_input_pivot;
//         }
//       }
//     }
//     sei();
//     CONTROL_PORT_INPUTS = control_state;

//     // Вернуть все входы сдвигового регистра в нулевое состояние
//     CONTROL_PORT &= ~CONTROL_MASK;
//     SPI.write(regs.data);
//   } else {
//     CONTROL_PORT_INPUTS = 0;
//   }

//   uint8_t pin = system_control_get_state();
//   if (pin) {
//     if (bit_istrue(pin,CONTROL_PIN_INDEX_RESET)) {
//       mc_reset();
//     } else if (bit_istrue(pin,CONTROL_PIN_INDEX_CYCLE_START)) {
//       bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
//     #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
//       } else if (bit_istrue(pin,CONTROL_PIN_INDEX_FEED_HOLD)) {
//         bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
//     #else
//       } else if (bit_istrue(pin,CONTROL_PIN_INDEX_SAFETY_DOOR)) {
//         bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
//     #endif
//     }
//   }
// }

// Возвращает, открыта (T) или закрыта (F) защитная дверь, на основе состояния пина.
uint8_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); // Входной пин не активирован, поэтому просто возвращаем, что дверь закрыта.
  #endif
}


// Выполняет пользовательский стартовый скрипт, если он сохранен.
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    ///delay(0);
    if (!(settings_read_startup_line(n, line))) {
      line[0] = 0;
      report_execute_startup_message(line,STATUS_SETTING_READ_FAIL, CLIENT_SERIAL);
    } else {
      if (line[0] != 0) {
        uint8_t status_code = gc_execute_line(line, CLIENT_SERIAL);
        report_execute_startup_message(line,status_code, CLIENT_SERIAL);
      }
    }
  }
}


// Направляет и выполняет одну строку форматированного ввода из protocol_process. Хотя в основном
// это входящие блоки g-кода, также выполняются внутренние команды Grbl, такие как
// настройки, инициирование цикла возврата в нулевую точку и переключение состояний.
// Отличается от модуля команд реального времени тем, что зависит от готовности Grbl
// выполнить следующую строку во время цикла, поэтому для переключателей, таких как удаление блоков,
// переключатель влияет только на строки, обрабатываемые после этого, а не обязательно в реальном времени
// во время цикла, поскольку в буфере уже сохранены движения. Однако эта "задержка" не должна
// быть проблемой, поскольку эти команды обычно не используются во время цикла.
uint8_t system_execute_line(char *line, uint8_t client)
{
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // Вспомогательная переменная
  float parameter, value;

  switch( line[char_counter] ) {
    case 0 : report_grbl_help(client); break;
    case 'J' : // Ручное перемещение (Jogging)
      // Выполнять только в состояниях IDLE или JOG.
      if (sys.state != STATE_IDLE && sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
      if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line, client)); // ПРИМЕЧАНИЕ: $J= игнорируется внутри парсера g-кода и используется для обнаружения движений ручного перемещения.
      break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : // Выводит настройки Grbl
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Блокировать во время цикла. Занимает слишком много времени для вывода.
          else { report_grbl_settings(client); }
          break;
        case 'G' : // Выводит состояние парсера g-кода
          // TODO: Переместить это в команды реального времени для GUI, чтобы запрашивать эти данные во время приостановки.
          report_gcode_modes(client);
          break;
        case 'C' : // Установить режим проверки g-кода [IDLE/CHECK]
          // Выполнить сброс при выключении. Режим проверки g-кода должен работать только если Grbl
          // находится в состоянии покоя и готов, независимо от блокировок тревоги. Это в основном для
          // сохранения простоты и согласованности.
          if ( sys.state == STATE_CHECK_MODE ) {
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // Требуется отсутствие режима тревоги.
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // Отключить блокировку тревоги [ALARM]
          if (sys.state == STATE_ALARM) {
            // Блокировать, если защитная дверь открыта.
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Не запускать стартовый скрипт. Предотвращает выполнение сохраненных движений в стартовом скрипте, которые могут вызвать аварии.
          } // Otherwise, no effect.
          break;
      }
      break;
    default :
      // Блокировать любые системные команды, требующие состояния IDLE/ALARM. (например, EEPROM, возврат в нулевую точку)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        case '#' : // Вывести параметры Grbl NGC
          if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(client); }
          break;
        case 'H' : // Выполнить цикл возврата в нулевую точку [IDLE/ALARM]
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // Блокировать, если защитная дверь открыта.
          sys.state = STATE_HOMING; // Установить переменную состояния системы
          if (line[2] == 0) {
            mc_homing_cycle(HOMING_CYCLE_ALL);
          #ifdef HOMING_SINGLE_AXIS_COMMANDS
            } else if (line[3] == 0) {
              switch (line[2]) {
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                default: return(STATUS_INVALID_STATEMENT);
              }
          #endif
          } else { return(STATUS_INVALID_STATEMENT); }
          if (!sys.abort) {  // Выполнить стартовые скрипты после успешного возврата в нулевую точку.
            sys.state = STATE_IDLE;  
// Установить в IDLE по завершении.
            st_go_idle(); // Установить шаговые двигатели в состояние покоя согласно настройкам перед возвратом.
            if (line[2] == 0) { system_execute_startup(line); }
          }
          break;
        case 'S' : // Переводит Grbl в спящий режим [IDLE/ALARM]
          if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); // Установить для немедленного выполнения спящего режима
          break;
        case 'I' : // Вывести или сохранить информацию о сборке. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) {
            settings_read_build_info(line);
            report_build_info(line, client);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // Сохранить стартовую строку [IDLE/ALARM]
              if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; // Установить вспомогательную переменную как счетчик начала строки информации пользователя.
              do {
                ///delay(0);
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // Восстановить настройки по умолчанию [IDLE/ALARM]
          if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
          switch (line[5]) {
            #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            #endif
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Принудительный сброс для гарантии правильной инициализации настроек.
          break;
        case 'N' : // Стартовые строки. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // Вывести стартовые строки
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              ///delay(0);
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL, CLIENT_ALL);
              } else {
                report_startup_line(helper_var,line, CLIENT_ALL);
              }
            }
            break;
          } else { // Сохранить стартовую строку [Только IDLE] Предотвращает движение во время ALARM.
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Сохранять только в состоянии покоя.
            helper_var = true;  // Set helper_var to flag storing method.
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              ///delay(0);
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line, client); // Set helper_var to returned status code.
            if (helper_var) { return(helper_var); }
            else {
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); // Если команда '$' дошла до сюда, значит всё в порядке.
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


// Возвращает машинную позицию оси 'idx'. Должен быть передан массив 'step'.
// ПРИМЕЧАНИЕ: Если шаги двигателя и машинная позиция находятся в разных системах координат,
//   эта функция служит центральным местом для вычисления преобразования.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if (idx==X_AXIS) {
      pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if (idx==Y_AXIS) {
      pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    pos = steps[idx]/settings.steps_per_mm[idx];
  #endif
  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}


// Только для расчета CoreXY. Возвращает "шаги" по оси X или Y на основе шагов двигателей CoreXY.
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif


// Проверяет и сообщает, превышает ли целевой массив пределы перемещения станка.
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      // Когда включено принудительное установление начала координат при возврате в нулевую точку,
      // проверки мягких пределов должны учитывать направленность.
      // ПРИМЕЧАНИЕ: max_travel хранится как отрицательное значение
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // ПРИМЕЧАНИЕ: max_travel хранится как отрицательное значение
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}


// Специальные обработчики для установки и очистки флагов выполнения Grbl в реальном времени.
void system_set_exec_state_flag(uint8_t mask) {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_state |= (mask);
  //restore_SREG(sreg);
}

void system_clear_exec_state_flag(uint8_t mask) {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_state &= ~(mask);
  //restore_SREG(sreg);
  //sei();
}

void system_set_exec_alarm(uint8_t code) {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_alarm = code;
  //restore_SREG(sreg);
}

void system_clear_exec_alarm() {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_alarm = 0;
  //restore_SREG(sreg);
  //  sei();
}

void system_set_exec_motion_override_flag(uint8_t mask) {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_motion_override |= (mask);
  //restore_SREG(sreg);
}

void system_set_exec_accessory_override_flag(uint8_t mask) {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_accessory_override |= (mask);
  //restore_SREG(sreg);
}

void system_clear_exec_motion_overrides() {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_motion_override = 0;
  //restore_SREG(sreg);
  //  sei();
}

void system_clear_exec_accessory_overrides() {
  //uint8_t sreg = save_SREG();
  HAL_IRQ_DisableInterrupts();
  //cli();
  sys_rt_exec_accessory_override = 0;
  //restore_SREG(sreg);
  //  sei();
}
