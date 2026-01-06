/* 
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such 
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a 
  different style feedback is desired (i.e. JSON), then a user can change these following 
  methods to accomodate their needs.
*/

#include "grbl.h"
#include "report.h"
#include "machine.h"

void Report::SetMachine(void* machine){
  machine_glb_ = machine;
}

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an 
// 'error:'  to indicate some error event with the line or some critical system error during 
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
// NOTE: In silent mode, all error codes are greater than zero.
// TODO: Install silent mode to return only numeric values, primarily for GUIs.
// Обрабатывает основной ответ протокола подтверждения для потоковых интерфейсов и обратной связи с пользователем.
// Для каждой входящей строки этот метод выдает "ok" в случае успешной команды или 
// "error:", чтобы указать на какое-либо событие, связанное с ошибкой в строке, или на какую-либо критическую системную ошибку во время работы 
//. События Errors могут возникать из-за синтаксического анализатора g-кода, модуля настроек или асинхронно
// из-за критической ошибки, такой как срабатывание жесткого ограничения. Интерфейс должен всегда отслеживать эти события
// ответы.
// ПРИМЕЧАНИЕ: В автоматическом режиме все коды ошибок больше нуля.
// TODO: Установите автоматический режим, чтобы возвращать только числовые значения, в первую очередь для графических интерфейсов.
void Report::StatusMessage(uint8_t status_code)
{
  if (status_code == 0) { // STATUS_OK
    printer_.PgmString("ok\r\n");
  } else {
    printer_.PgmString("error: ");
    #ifdef REPORT_GUI_MODE
      printer_.Uint8Base10(status_code);
    #else
      switch(status_code) {          
        case STATUS_EXPECTED_COMMAND_LETTER:
        printer_.PgmString("Expected command letter"); break;
        case STATUS_BAD_NUMBER_FORMAT:
        printer_.PgmString("Bad number format"); break;
        case STATUS_INVALID_STATEMENT:
        printer_.PgmString("Invalid statement"); break;
        case STATUS_NEGATIVE_VALUE:
        printer_.PgmString("Value < 0"); break;
        case STATUS_SETTING_DISABLED:
        printer_.PgmString("Setting disabled"); break;
        case STATUS_SETTING_STEP_PULSE_MIN:
        printer_.PgmString("Value < 3 usec"); break;
        case STATUS_SETTING_READ_FAIL:
        printer_.PgmString("EEPROM read fail. Using defaults"); break;
        case STATUS_IDLE_ERROR:
        printer_.PgmString("Not idle"); break;
        case STATUS_ALARM_LOCK:
        printer_.PgmString("Alarm lock"); break;
        case STATUS_SOFT_LIMIT_ERROR:
        printer_.PgmString("Homing not enabled"); break;
        case STATUS_OVERFLOW:
        printer_.PgmString("Line overflow"); break;
        #ifdef MAX_STEP_RATE_HZ
          case STATUS_MAX_STEP_RATE_EXCEEDED: 
          printer_.PgmString("Step rate > 30kHz"); break;
        #endif      
        // Common g-code parser errors.
        // Распространенные ошибки синтаксического анализатора g-кода.
        case STATUS_GCODE_MODAL_GROUP_VIOLATION:
        printer_.PgmString("Modal group violation"); break;
        case STATUS_GCODE_UNSUPPORTED_COMMAND:
        printer_.PgmString("Unsupported command"); break;
        case STATUS_GCODE_UNDEFINED_FEED_RATE:
        printer_.PgmString("Undefined feed rate"); break;
        default:
          // Remaining g-code parser errors with error codes // Оставшиеся ошибки синтаксического анализатора g-кода с кодами ошибок
          printer_.PgmString("Invalid gcode ID:");
          printer_.Uint8Base10(status_code); // Print error code for user reference // Напечатать код ошибки для справки пользователю
      }
    #endif  
    printer_.PgmString("\r\n");
  }
}

// Prints alarm messages. // Выводит тревожные сообщения.
void Report::AlarmMessage(int8_t alarm_code)
{
  printer_.PgmString("ALARM: ");
  #ifdef REPORT_GUI_MODE
    printer_.Uint8Base10(alarm_code);
  #else
    switch (alarm_code) {
      case ALARM_HARD_LIMIT_ERROR: 
      printer_.PgmString("Hard limit"); break;
      case ALARM_SOFT_LIMIT_ERROR:
      printer_.PgmString("Soft limit"); break;
      case ALARM_ABORT_CYCLE: 
      printer_.PgmString("Abort during cycle"); break;
      case ALARM_PROBE_FAIL:
      printer_.PgmString("Probe fail"); break;
      case ALARM_HOMING_FAIL:
      printer_.PgmString("Homing fail"); break;
    }
  #endif
  printer_.PgmString("\r\n");
  delay_ms(500); // Force delay to ensure message clears serial write buffer. // Принудительная задержка, чтобы гарантировать, что сообщение очистит буфер последовательной записи.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
// TODO: Install silence feedback messages option in settings

// Печатает сообщения обратной связи. Это служит централизованным способом предоставления дополнительных
// отзывов пользователей о событиях, которые не относятся к протоколу сообщений о состоянии/тревоге. Это
// сообщения, такие как предупреждения о настройке, переключении переключателей и способах выхода из аварийных сигналов.
// ПРИМЕЧАНИЕ: Для интерфейсов сообщения всегда заключаются в квадратные скобки. И если установлен беззвучный режим
//, коды номеров сообщений будут меньше нуля.
// ЗАДАЧА: Установите опцию отключения сообщений обратной связи в настройках

void Report::FeedbackMessage(uint8_t message_code)
{
  printer_.PgmString("[");
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
    printer_.PgmString("Reset to continue"); break;
    case MESSAGE_ALARM_LOCK:
    printer_.PgmString("'$H'|'$X' to unlock"); break;
    case MESSAGE_ALARM_UNLOCK:
    printer_.PgmString("Caution: Unlocked"); break;
    case MESSAGE_ENABLED:
    printer_.PgmString("Enabled"); break;
    case MESSAGE_DISABLED:
    printer_.PgmString("Disabled"); break; 
    case MESSAGE_SAFETY_DOOR_AJAR:
    printer_.PgmString("Check Door"); break;
    case MESSAGE_PROGRAM_END:
    printer_.PgmString("Pgm End"); break;
    case MESSAGE_RESTORE_DEFAULTS:
    printer_.PgmString("Restoring defaults"); break;
  }
  printer_.PgmString("]\r\n");
}


// Welcome message // Приветственное сообщение
void Report::InitMessage()
{
  printer_.PgmString("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n");
}

// Grbl help message // Справочное сообщение Grbl
void Report::GrblHelp() {
  #ifndef REPORT_GUI_MODE
    printer_.PgmString("$$ (view Grbl settings)\r\n"
                        "$# (view # parameters)\r\n"
                        "$G (view parser state)\r\n"
                        "$I (view build info)\r\n"
                        "$N (view startup blocks)\r\n"
                        "$x=value (save Grbl setting)\r\n"
                        "$Nx=line (save startup block)\r\n"
                        "$C (check gcode mode)\r\n"
                        "$X (kill alarm lock)\r\n"
                        "$H (run homing cycle)\r\n"
                        "~ (cycle start)\r\n"
                        "! (feed hold)\r\n"
                        "? (current status)\r\n"
                        "ctrl-x (reset Grbl)\r\n");
  #endif
}


// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
// Распечатать глобальные настройки Grbl.
// ПРИМЕЧАНИЕ: Схема нумерации здесь должна соответствовать схеме, сохраненной в settings.c
void Report::GrblSettings(const settings_t& settings) {
  // Print Grbl settings. // Распечатать настройки Grbl.
  #ifdef REPORT_GUI_MODE
    printer_.PgmString("$0="); printer_.Uint8Base10(settings.pulse_microseconds);
    printer_.PgmString("\r\n$1="); printer_.Uint8Base10(settings.stepper_idle_lock_time);
    printer_.PgmString("\r\n$2="); printer_.Uint8Base10(settings.step_invert_mask); 
    printer_.PgmString("\r\n$3="); printer_.Uint8Base10(settings.dir_invert_mask); 
    printer_.PgmString("\r\n$4="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
    printer_.PgmString("\r\n$5="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
    printer_.PgmString("\r\n$6="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
    printer_.PgmString("\r\n$10="); printer_.Uint8Base10(settings.status_report_mask);
    printer_.PgmString("\r\n$11="); printer_.FloatSettingValue(settings.junction_deviation);
    printer_.PgmString("\r\n$12="); printer_.FloatSettingValue(settings.arc_tolerance);
    printer_.PgmString("\r\n$13="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
    printer_.PgmString("\r\n$20="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
    printer_.PgmString("\r\n$21="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
    printer_.PgmString("\r\n$22="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
    printer_.PgmString("\r\n$23="); printer_.Uint8Base10(settings.homing_dir_mask);
    printer_.PgmString("\r\n$24="); printer_.FloatSettingValue(settings.homing_feed_rate);
    printer_.PgmString("\r\n$25="); printer_.FloatSettingValue(settings.homing_seek_rate);
    printer_.PgmString("\r\n$26="); printer_.Uint8Base10(settings.homing_debounce_delay);
    printer_.PgmString("\r\n$27="); printer_.FloatSettingValue(settings.homing_pulloff);
    printer_.PgmString("\r\n");
  #else      
    printer_.PgmString("$0="); printer_.Uint8Base10(settings.pulse_microseconds);
    printer_.PgmString(" (step pulse, usec)\r\n$1="); printer_.Uint8Base10(settings.stepper_idle_lock_time);
    printer_.PgmString(" (step idle delay, msec)\r\n$2="); printer_.Uint8Base10(settings.step_invert_mask); 
    printer_.PgmString(" (step port invert mask:"); printer_.Uint8Base2(settings.step_invert_mask);  
    printer_.PgmString(")\r\n$3="); printer_.Uint8Base10(settings.dir_invert_mask); 
    printer_.PgmString(" (dir port invert mask:"); printer_.Uint8Base2(settings.dir_invert_mask);  
    printer_.PgmString(")\r\n$4="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
    printer_.PgmString(" (step enable invert, bool)\r\n$5="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
    printer_.PgmString(" (limit pins invert, bool)\r\n$6="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
    printer_.PgmString(" (probe pin invert, bool)\r\n$10="); printer_.Uint8Base10(settings.status_report_mask);
    printer_.PgmString(" (status report mask:"); printer_.Uint8Base2(settings.status_report_mask);
    printer_.PgmString(")\r\n$11="); printer_.FloatSettingValue(settings.junction_deviation);
    printer_.PgmString(" (junction deviation, mm)\r\n$12="); printer_.FloatSettingValue(settings.arc_tolerance);
    printer_.PgmString(" (arc tolerance, mm)\r\n$13="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
    printer_.PgmString(" (report inches, bool)\r\n$20="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
    printer_.PgmString(" (soft limits, bool)\r\n$21="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
    printer_.PgmString(" (hard limits, bool)\r\n$22="); printer_.Uint8Base10(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
    printer_.PgmString(" (homing cycle, bool)\r\n$23="); printer_.Uint8Base10(settings.homing_dir_mask);
    printer_.PgmString(" (homing dir invert mask:"); printer_.Uint8Base2(settings.homing_dir_mask);  
    printer_.PgmString(")\r\n$24="); printer_.FloatSettingValue(settings.homing_feed_rate);
    printer_.PgmString(" (homing feed, mm/min)\r\n$25="); printer_.FloatSettingValue(settings.homing_seek_rate);
    printer_.PgmString(" (homing seek, mm/min)\r\n$26="); printer_.Uint8Base10(settings.homing_debounce_delay);
    printer_.PgmString(" (homing debounce, msec)\r\n$27="); printer_.FloatSettingValue(settings.homing_pulloff);
    printer_.PgmString(" (homing pull-off, mm)\r\n");
  #endif
  
  // Print axis settings
  // Настройки оси печати
  uint8_t idx, set_idx;
  uint8_t val = AXIS_SETTINGS_START_VAL;
  for (set_idx=0; set_idx<AXIS_N_SETTINGS; set_idx++) {
    for (idx=0; idx<N_AXIS; idx++) {
      printer_.PgmString("$");
      printer_.Uint8Base10(val+idx);
      printer_.PgmString("=");
      switch (set_idx) {
        case 0: printer_.FloatSettingValue(settings.steps_per_mm[idx]); break;
        case 1: printer_.FloatSettingValue(settings.max_rate[idx]); break;
        case 2: printer_.FloatSettingValue(settings.acceleration[idx]/(60*60)); break;
        case 3: printer_.FloatSettingValue(-settings.max_travel[idx]); break;
      }
      #ifdef REPORT_GUI_MODE
        printer_.PgmString("\r\n");
      #else
        printer_.PgmString(" (");
        switch (idx) {
          case X_AXIS: printer_.PgmString("x"); break;
          case Y_AXIS: printer_.PgmString("y"); break;
          case Z_AXIS: printer_.PgmString("z"); break;
        }
        switch (set_idx) {
          case 0: printer_.PgmString(", step/mm"); break;
          case 1: printer_.PgmString(" max rate, mm/min"); break;
          case 2: printer_.PgmString(" accel, mm/sec^2"); break;
          case 3: printer_.PgmString(" max travel, mm"); break;
        }      
        printer_.PgmString(")\r\n");
      #endif
    }
    val += AXIS_SETTINGS_INCREMENT;
  }  
}


// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported). 
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
// Выводит текущие параметры проверки. По команде проверки эти параметры обновляются после
// успешной проверки или неудачной проверки с помощью команды G38.3 без ошибок (если поддерживается). 
// Эти значения сохраняются до тех пор, пока Grbl не переключится на питание, после чего они будут повторно обнулены.
void Report::ProbeParameters()
{
  uint8_t i;
  float print_position[N_AXIS];
 
  // Report in terms of machine position.
  // Отчет с точки зрения положения машины.
  printer_.PgmString("[PRB:");
  for (i=0; i< N_AXIS; i++) {
    print_position[i] = static_cast<Machine*>(machine_glb_)->GetPosition(i);
    printer_.FloatCoordValue(print_position[i]);
    if (i < (N_AXIS-1)) { printer_.PgmString(","); }
  }
  printer_.PgmString(":");
  printer_.Uint8Base10(static_cast<Machine*>(machine_glb_)->GetProbeSucceeded());
  printer_.PgmString("]\r\n");
}


// Prints Grbl NGC parameters (coordinate offsets, probing)
// Выводит параметры Grbl NGC (смещения координат, зондирование)
void Report::NgcParameters()
{
  // float coord_data[N_AXIS];
  // uint8_t coord_select, i;
  // for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) { 
  //   if (!(settings_read_coord_data(coord_select,coord_data))) { 
  //     Report::StatusMessage(STATUS_SETTING_READ_FAIL);
  //     return;
  //   } 
  //   printer_.PgmString("[G");
  //   switch (coord_select) {
  //     case 6: printer_.PgmString("28"); break;
  //     case 7: printer_.PgmString("30"); break;
  //     default: printer_.Uint8Base10(coord_select+54); break; // G54-G59
  //   }  
  //   printer_.PgmString(":");         
  //   for (i=0; i<N_AXIS; i++) {
  //     printer_.FloatCoordValue(coord_data[i]);
  //     if (i < (N_AXIS-1)) { printer_.PgmString(","); }
  //     else { printer_.PgmString("]\r\n"); }
  //   } 
  // }
  // printer_.PgmString("[G92:"); // Print G92,G92.1 which are not persistent in memory // Выведите G92,G92.1, которые не являются постоянными в памяти
  // for (i=0; i<N_AXIS; i++) {
  //   printer_.FloatCoordValue(gc_state.coord_offset[i]);
  //   if (i < (N_AXIS-1)) { printer_.PgmString(","); }
  //   else { printer_.PgmString("]\r\n"); }
  // } 
  // printer_.PgmString("[TLO:"); // Print tool length offset value // Значение смещения длины печатающего инструмента
  // printer_.FloatCoordValue(gc_state.tool_length_offset);
  // printer_.PgmString("]\r\n");
  Report::ProbeParameters(); // Print probe parameters. Not persistent in memory. // Вывод параметров датчика. Не сохраняется в памяти.
}


// Print current gcode parser mode state // Вывести текущее состояние режима синтаксического анализа gcode
void Report::GcodeModes()
{
  printer_.PgmString("[");
  
  switch (gc_state.modal.motion) {
    case MOTION_MODE_SEEK : printer_.PgmString("G0"); break;
    case MOTION_MODE_LINEAR : printer_.PgmString("G1"); break;
    case MOTION_MODE_CW_ARC : printer_.PgmString("G2"); break;
    case MOTION_MODE_CCW_ARC : printer_.PgmString("G3"); break;
    case MOTION_MODE_NONE : printer_.PgmString("G80"); break;
    default: 
      printer_.PgmString("G38.");
      printer_.Uint8Base10(gc_state.modal.motion - (MOTION_MODE_PROBE_TOWARD-2));
  }

  printer_.PgmString(" G");
  printer_.Uint8Base10(gc_state.modal.coord_select+54);
  
  switch (gc_state.modal.plane_select) {
    case PLANE_SELECT_XY : printer_.PgmString(" G17"); break;
    case PLANE_SELECT_ZX : printer_.PgmString(" G18"); break;
    case PLANE_SELECT_YZ : printer_.PgmString(" G19"); break;
  }
  
  if (gc_state.modal.units == UNITS_MODE_MM) { printer_.PgmString(" G21"); }
  else { printer_.PgmString(" G20"); }
  
  if (gc_state.modal.distance == DISTANCE_MODE_ABSOLUTE) { printer_.PgmString(" G90"); }
  else { printer_.PgmString(" G91"); }
  
  if (gc_state.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { printer_.PgmString(" G93"); }
  else { printer_.PgmString(" G94"); }
    
  switch (gc_state.modal.program_flow) {
    case PROGRAM_FLOW_RUNNING : printer_.PgmString(" M0"); break;
    case PROGRAM_FLOW_PAUSED : printer_.PgmString(" M1"); break;
    case PROGRAM_FLOW_COMPLETED : printer_.PgmString(" M2"); break;
  }

  switch (gc_state.modal.spindle) {
    case SPINDLE_ENABLE_CW : printer_.PgmString(" M3"); break;
    case SPINDLE_ENABLE_CCW : printer_.PgmString(" M4"); break;
    case SPINDLE_DISABLE : printer_.PgmString(" M5"); break;
  }
  
  switch (gc_state.modal.coolant) {
    case COOLANT_DISABLE : printer_.PgmString(" M9"); break;
    case COOLANT_FLOOD_ENABLE : printer_.PgmString(" M8"); break;
    #ifdef ENABLE_M7
      case COOLANT_MIST_ENABLE : printer_.PgmString(" M7"); break;
    #endif
  }
  
  printer_.PgmString(" T");
  printer_.Uint8Base10(gc_state.tool);
  
  printer_.PgmString(" F");
  printer_.FloatRateValue(gc_state.feed_rate);
  
  #ifdef VARIABLE_SPINDLE
    printer_.PgmString(" S");
    printer_.FloatRateValue(gc_state.spindle_speed);
  #endif

  printer_.PgmString("]\r\n");
}

// Prints specified startup line // Выводит указанную строку запуска
void Report::StartupLine(uint8_t n, char *line)
{
  printer_.PgmString("$N"); printer_.Uint8Base10(n);
  printer_.PgmString("="); printer_.String(line);
  printer_.PgmString("\r\n");
}


// Prints build info line
void Report::BuildInfo(char *line)
{
  printer_.PgmString("[" GRBL_VERSION "." GRBL_VERSION_BUILD ":");
  printer_.String(line);
  printer_.PgmString("]\r\n");
}


// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
// Выводит строку символьной строки, полученную Grbl от пользователя, которая была предварительно обработана,
// и отправлена в процедуру protocol_execute_line() для выполнения Grbl.
void Report::EchoLineReceived(char *line)
{
  printer_.PgmString("[echo: "); printer_.String(line);
  printer_.PgmString("]\r\n");
}

// Печатает данные в режиме реального времени. Эта функция позволяет в режиме реального времени получить снимок шаговой подпрограммы 
 // и фактическое местоположение станка с ЧПУ. Пользователи могут изменить следующую функцию по своему усмотрению.
 // конкретные потребности, но желаемый отчет о данных в режиме реального времени должен быть как можно более коротким. Это
// необходимо, поскольку сводит к минимуму вычислительную нагрузку и позволяет grbl продолжать бесперебойную работу,
// особенно при работе с программами g-code с быстрыми, короткими отрезками строк и высокочастотными отчетами (5-20 Гц).
 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram 
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly, 
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void Report::RealtimeStatus(settings_t& settings)
{
  // **Under construction** Bare-bones status report. Provides real-time machine position relative to 
  // the system power on location (0,0,0) and work coordinate position (G54 and G92 applied). Eventually
  // to be added are distance to go on block, processed block id, and feed rate. Also a settings bitmask
  // for a user to select the desired real-time data.
  // **В стадии разработки** Краткий отчет о состоянии станка. Отображает положение станка в реальном времени относительно 
  // местоположения включения системы (0,0,0) и положения рабочих координат (применяются G54 и G92). В конце концов
  // необходимо добавить расстояние до блока, идентификатор обрабатываемого блока и скорость подачи. Также битовую маску настроек
  // чтобы пользователь мог выбрать нужные данные в режиме реального времени.

  // Временно закомментировал нужно переделать на  static_cast<Machine*>(machine_glb_)->GetPosition
  // uint8_t idx;
  // int32_t current_position[N_AXIS]; // Copy current state of the system position variable // Скопировать текущее состояние системной переменной положения
  // memcpy(current_position,sys.position,sizeof(sys.position));
  // float print_position[N_AXIS];
 
  // Report current machine state // Сообщить о текущем состоянии машины
  switch (static_cast<Machine*>(machine_glb_)->GetMachineState()) {
    case STATE_IDLE: printer_.PgmString("<Idle"); break;
    case STATE_MOTION_CANCEL: // Report run state. // Отчет о состоянии выполнения.
    case STATE_CYCLE: printer_.PgmString("<Run"); break;
    case STATE_HOLD: printer_.PgmString("<Hold"); break;
    case STATE_HOMING: printer_.PgmString("<Home"); break;
    case STATE_ALARM: printer_.PgmString("<Alarm"); break;
    case STATE_CHECK_MODE: printer_.PgmString("<Check"); break;
    case STATE_SAFETY_DOOR: printer_.PgmString("<Door"); break;
  }
 
  // If reporting a position, convert the current step count (current_position) to millimeters. 
  // Если вы сообщаете о местоположении, преобразуйте текущее количество шагов (current_position) в миллиметры.
  // if (bit_istrue(settings.status_report_mask,(BITFLAG_RT_STATUS_MACHINE_POSITION | BITFLAG_RT_STATUS_WORK_POSITION))) {
    // system_convert_array_steps_to_mpos(print_position,current_position);
  // }
  
  // Report machine position // Сообщить о положении машины
  // if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_MACHINE_POSITION)) {
  //   printer_.PgmString(",MPos:"); 
  //   for (idx=0; idx< N_AXIS; idx++) {
  //     printer_.FloatCoordValue(print_position[idx]);
  //     if (idx < (N_AXIS-1)) { printer_.PgmString(","); }
  //   }
  // }
  
  // Report work position // Сообщить о рабочем месте
  // if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_WORK_POSITION)) {
  //   printer_.PgmString(",WPos:"); 
  //   for (idx=0; idx< N_AXIS; idx++) {
  //     // Apply work coordinate offsets and tool length offset to current position. // Примените смещения рабочих координат и длины инструмента к текущему положению.
  //     print_position[idx] -= gc_state.coord_system[idx]+gc_state.coord_offset[idx];
  //     if (idx == TOOL_LENGTH_OFFSET_AXIS) { print_position[idx] -= gc_state.tool_length_offset; }    
  //     printer_.FloatCoordValue(print_position[idx]);
  //     if (idx < (N_AXIS-1)) { printer_.PgmString(","); }
  //   }
  // }
        
  // Returns the number of active blocks are in the planner buffer. // Возвращает количество активных блоков, находящихся в буфере планировщика.
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_PLANNER_BUFFER)) {
    printer_.PgmString(",Buf:");
    printer_.Uint8Base10(plan_get_block_buffer_count());
  }

  // Report serial read buffer status // Сообщить о состоянии буфера последовательного чтения
  if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_SERIAL_RX)) {
    printer_.PgmString(",RX:");
    printer_.SerialGetRxBufferCount();
  }
    
  #ifdef USE_LINE_NUMBERS
    // Report current line number // Сообщить номер текущей строки
    printer_.PgmString(",Ln:"); 
    int32_t ln=0;
    plan_block_t * pb = plan_get_current_block();
    if(pb != NULL) {
      ln = pb->line_number;
    } 
    printInteger(ln);
  #endif
    
  #ifdef REPORT_REALTIME_RATE
    // Report realtime rate  // Сообщать о скорости в реальном времени2
    printer_.PgmString(",F:"); 
    printer_.FloatRateValue(st_get_realtime_rate());
  #endif    
  
   if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_LIMIT_PINS)) {
     printer_.PgmString(",Lim:");
     printer_.PgmString(static_cast<Machine*>(machine_glb_)->LimitsGetState());
   }
  
  #ifdef REPORT_CONTROL_PIN_STATE 
    printer_.PgmString(",Ctl:");
    printer_.Uint8Base2(CONTROL_PIN & CONTROL_MASK);
  #endif
  
  printer_.PgmString(">\r\n");
}
