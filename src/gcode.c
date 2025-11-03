/*
  gcode.c - rs274/ngc parser.
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

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.

// ПРИМЕЧАНИЕ: Максимальный номер строки определен стандартом g-code равным 99999. Этопроизвольное значение, и для некоторых графических интерфейсов может потребоваться больше. Поэтому мы увеличили его, исходя из максимального безопасного значения
// значение при преобразовании числа с плавающей запятой (точность 7,2 знака)s в целое число.

#define MAX_LINE_NUMBER 9999999 

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1 
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required

// Объявить внешнюю структуру gc
// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


void gc_init() 
{
  memset(&gc_state, 0, sizeof(parser_state_t));

  // Загрузите систему координат G54 по умолчанию.
  // Load default G54 coordinate system.
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { 
    report_status_message(STATUS_SETTING_READ_FAIL); 
  } 
}


// Задает положение анализатора g-кода в миллиметрах. Ввод выполняется поэтапно. Вызывается системой для прерывания и жесткого управления
// ограничивает процедуры извлечения.
// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position() 
{
  system_convert_array_steps_to_mpos(gc_state.position,sys.position);
}


static uint8_t gc_check_same_position(float *pos_a, float *pos_b) 
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    if (pos_a[idx] != pos_b[idx]) { return(false); }
  }
  return(true);
}

// Выполняет одну строку G-кода, заканчивающуюся 0. Предполагается, что строка содержит только символы верхнего регистра
// и значения со знаком плавающей запятой (без пробелов). Комментарии и блокировка удалены
// символы удалены. В этой функции все единицы измерения и положения преобразуются и 
// экспортируются во внутренние функции grbl в виде (мм, мм/мин) и абсолютных машинных 
// координат соответственно.

// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and 
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine 
// coordinates, respectively.
uint8_t gc_execute_line(char *line) 
{
  /* -------------------------------------------------------------------------------------
     ШАГ 1: Инициализируйте структуру блока синтаксического анализа и скопируйте текущие режимы состояния g-кода. Синтаксический анализатор
     обновляет эти режимы и команды, поскольку строка блока является синтаксическим анализатором и будет использоваться и
выполняться только после успешной проверки ошибок. Структура блока синтаксического анализа также содержит
структуру значений блоков, переменные отслеживания слов и средство отслеживания немодальных команд для нового 
     блок. Эта структура содержит всю необходимую информацию для выполнения блока. */
  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parser and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new 
     block. This struct contains all of the necessary information to execute the block. */

  memset(&gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct. // Инициализируем структуру блока синтаксического анализа.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); // Copy current modes // Копировать текущие режимы
  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // Tracks G10 P coordinate selection for execution // Отслеживает выбор координат G10 P для выполнения
  float coordinate_data[N_AXIS]; // Multi-use variable to store coordinate data for execution // Многоцелевая переменная для хранения координатных данных для выполнения
  float parameter_data[N_AXIS]; // Multi-use variable to store parameter data for execution
  
  // Initialize bitflag tracking variables for axis indices compatible operations. // Инициализируйте переменные отслеживания bitflag для операций, совместимых с индексами axis.
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking 

  // Initialize command and value words variables. Tracks words contained in this block. // Инициализирует переменные command и value words. Отслеживает слова, содержащиеся в этом блоке.
  uint16_t command_words = 0; // G and M command words. Also used for modal group violations. / Командные слова / G и M. Также используются для обозначения групповых нарушений модальности.
  uint16_t value_words = 0; // Value words. 

  /* -------------------------------------------------------------------------------------
     ШАГ 2: Импортируйте все g-кодовые слова в строку блока. G-кодовое слово - это буква, за которой следует
цифра, которая может быть либо командой "G"/"M", либо задает/присваивает значение команды. Кроме того,
выполните первоначальную проверку ошибок на предмет нарушений модальной группы командных слов при любых повторных
     слов, а также для отрицательных значений, установленных для значений слов F, N, P, T и S. */
  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also, 
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */
     
  uint8_t word_bit; // Bit-value for assigning tracking variables // Бит-значение для назначения переменных отслеживания
  uint8_t char_counter = 0;  
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;

  while (line[char_counter] != 0) { // Loop until no more g-code words in line. // Повторяйте цикл до тех пор, пока в строке не закончатся слова g-кода.

    // Импортируйте следующее g-кодовое слово, ожидая, что за буквой будет значение. В противном случае выдается ошибка.
    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter] // [Ожидаемая буква в слове]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value] // [Ожидаемое значение слова]

    // Convert values to smaller uint8 significand and mantissa values for parsing this word.
    // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more 
    // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
    // accurate enough for value words that require integers to within 0.0001. This should be
    // a good enough comprimise and catch most all non-integer errors. To make it compliant, 
    // we would simply need to change the mantissa to int16, but this add compiled flash space.
    // Maybe update this later.

    // Преобразуйте значения в меньшие значения uint8 и значения мантиссы для синтаксического анализа этого слова.
    // ПРИМЕЧАНИЕ: значение мантиссы умножается на 100 для получения нецелочисленных значений команды. Это более 
    // более точный, чем требование NIST к gcode, равное x10, при использовании для команд, но не совсем
    // достаточно точный для слов-значений, для которых требуются целые числа с точностью до 0,0001. Это должно быть
    // достаточно хорошей компиляцией и улавливать большинство нецелочисленных ошибок. Чтобы сделать ее совместимой,
    // нам просто нужно было бы изменить мантиссу на int16, но это добавит скомпилированное флэш-пространство.
    // Возможно, обновим это позже.
    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.
        // NOTE: Rounding must be used to catch small floating point errors. 

    // Check if the g-code word is supported or errors due to modal group violations or has
    // been repeated in the g-code block. If ok, update the command or record its value.

    // Вычислите мантиссу для команд Gxx.x.
    // ПРИМЕЧАНИЕ: Округление должно использоваться для обнаружения небольших ошибок с плавающей запятой. 

    // Проверьте, поддерживается ли слово g-кода или ошибки вызваны нарушениями модальной группы или
    // были повторены в блоке g-кода. Если все в порядке, обновите команду или запишите ее значение.
    
    switch(letter) {

      /* Командные слова "G" и "M": анализируют команды и проверяют наличие нарушений в модальных группах.
         ПРИМЕЧАНИЕ: Номера модальных групп определены в таблице 4 NIST RS274-NGC v3, стр.20. */
      /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
         NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */
         
      case 'G':
	// Определите команду "G" и ее модальную группу
        // Determine 'G' command and its modal group
        switch(int_value) {
          case 10: case 28: case 30: case 92:
	    // Проверьте, не вызывается ли G10/28/30/92 с помощью G0/1/2/3/38 в том же блоке.
            // * G43.1 также является командой axis, но явно не определена таким образом.
            // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Игнорируйте пункты G28.1, G30.1 и G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict] // [Конфликт слов оси/команд]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line. // Без перерыва. Переходит к следующей строке.
          case 4: case 53: 
            word_bit = MODAL_GROUP_G0; 
            switch(int_value) {
              case 4: gc_block.non_modal_command = NON_MODAL_DWELL; break; // G4
              case 10: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_DATA; break; // G10
              case 28:
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_0; break;  // G28
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_0; break; // G28.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G28.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command. // Установите равным нулю, чтобы указать допустимую нецелочисленную команду G.
                break;
              case 30: 
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_1; break;  // G30
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_1; break; // G30.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G30.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command. // Установите равным нулю, чтобы указать допустимую нецелочисленную команду G.
                break;
              case 53: gc_block.non_modal_command = NON_MODAL_ABSOLUTE_OVERRIDE; break; // G53
              case 92: 
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_OFFSET; break; // G92
                  case 10: gc_block.non_modal_command = NON_MODAL_RESET_COORDINATE_OFFSET; break; // G92.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G92.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command. // Установите равным нулю, чтобы указать допустимую нецелочисленную команду G.
                break;      
            }
            break;
          case 0: case 1: case 2: case 3: case 38: 
            // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.

	    // Проверьте, не вызывается ли G0/1/2/3/38 с помощью G10/28/30/92 в том же блоке.
            // * G43.1 также является командой axis, но явно не определена таким образом.
	    
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE; 
            // No break. Continues to next line.
          case 80: 
            word_bit = MODAL_GROUP_G1; 
            switch(int_value) {
              case 0: gc_block.modal.motion = MOTION_MODE_SEEK; break; // G0
              case 1: gc_block.modal.motion = MOTION_MODE_LINEAR; break; // G1
              case 2: gc_block.modal.motion = MOTION_MODE_CW_ARC; break; // G2
              case 3: gc_block.modal.motion = MOTION_MODE_CCW_ARC; break; // G3
              case 38: 
                switch(mantissa) {
                  case 20: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD; break; // G38.2
                  case 30: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD_NO_ERROR; break; // G38.3
                  case 40: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY; break; // G38.4
                  case 50: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY_NO_ERROR; break; // G38.5
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G38.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command. // Установите равным нулю, чтобы указать допустимую нецелочисленную команду G.
                break;
              case 80: gc_block.modal.motion = MOTION_MODE_NONE; break; // G80
            }            
            break;
          case 17: case 18: case 19: 
            word_bit = MODAL_GROUP_G2; 
            switch(int_value) {
              case 17: gc_block.modal.plane_select = PLANE_SELECT_XY; break;
              case 18: gc_block.modal.plane_select = PLANE_SELECT_ZX; break;
              case 19: gc_block.modal.plane_select = PLANE_SELECT_YZ; break;
            }
            break;
          case 90: case 91: 
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3; 
              if (int_value == 90) { gc_block.modal.distance = DISTANCE_MODE_ABSOLUTE; } // G90
              else { gc_block.modal.distance = DISTANCE_MODE_INCREMENTAL; } // G91
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
	      // Установите значение равно нулю, чтобы указать допустимую нецелочисленную команду G.
              // В противном случае по умолчанию используется инкрементальный режим arc IJK. G91.1 ничего не делает.
            }
            break;
          case 93: case 94: 
            word_bit = MODAL_GROUP_G5; 
            if (int_value == 93) { gc_block.modal.feed_rate = FEED_RATE_MODE_INVERSE_TIME; } // G93
            else { gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN; } // G94
            break;
          case 20: case 21: 
            word_bit = MODAL_GROUP_G6; 
            if (int_value == 20) { gc_block.modal.units = UNITS_MODE_INCHES; }  // G20
            else { gc_block.modal.units = UNITS_MODE_MM; } // G21
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            // NOTE: Not required since cutter radius compensation is always disabled. Only here
            // to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
	    // ПРИМЕЧАНИЕ: Не требуется, так как компенсация радиуса фрезы всегда отключена. Только здесь
            // для поддержки команд G40, которые часто появляются в заголовках программы g-code, для настройки значений по умолчанию.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
            // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
            // all are explicit axis commands, regardless if they require axis words or not.
	    // ПРИМЕЧАНИЕ: В стандарте NIST g-code четко указано, что при изменении смещения длины инструмента
	    //не может быть изменено перемещение оси или смещение координат. Имеется в виду G43, G43.1 и G49
            // все они являются явными командами axis, независимо от того, требуются ли для них слова axis или нет.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL; 
            } else if (mantissa == 10) { // G43.1
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported G43.x command]
            mantissa = 0; // Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59: 
            // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value-54; // Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }      
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
	// Проверьте наличие более чем одной команды для каждой модальной группы нарушений в текущем блоке
	// ПРИМЕЧАНИЕ: Переменная 'word_bit' всегда присваивается, если команда допустима.
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;
        
      case 'M':
      
        // Determine 'M' command and its modal group // Определите команду "M" и ее модальную группу
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30: 
            word_bit = MODAL_GROUP_M4; 
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore. // Дополнительная остановка не поддерживается. Игнорировать.
              case 2: case 30: gc_block.modal.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset 
            }
            break;
          #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
            case 4: 
          #endif
          case 3: case 5:
            word_bit = MODAL_GROUP_M7; 
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;
              #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
                case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;
              #endif
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;
            }
            break;            
         #ifdef ENABLE_M7  
          case 7:
         #endif
          case 8: case 9:
            word_bit = MODAL_GROUP_M8; 
            switch(int_value) {      
             #ifdef ENABLE_M7
              case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
             #endif
              case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
            }
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }
      
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
	// Проверьте наличие более чем одной команды для каждой модальной группы нарушений в текущем блоке
	// ПРИМЕЧАНИЕ: Переменная 'word_bit' всегда присваивается, если команда допустима.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;
      
      // NOTE: All remaining letters assign values.
      // ПРИМЕЧАНИЕ: Все остальные буквы присваивают значения.
      default: 
  
        /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
           legal g-code words and stores their value. Error-checking is performed later since some
           words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */
	/* Некомандные слова: На этом начальном этапе синтаксического анализа проверяются только повторы оставшихся
           допустимых слов в g-коде и сохраняется их значение. Проверка на ошибки выполняется позже, поскольку некоторые
           слова (I, J, K, L, P, R) имеют множество значений и/или зависят от выполняемых команд. */
        switch(letter){
          // case 'A': // Not supported
          // case 'B': // Not supported
          // case 'C': // Not supported
          // case 'D': // Not supported
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // Not supported
          case 'I': word_bit = WORD_I; gc_block.values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // Not supported
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
          case 'T': word_bit = WORD_T; break; // gc.values.t = int_value;
          case 'X': word_bit = WORD_X; gc_block.values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        } 
        
        // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [Word repeated]
        // Check for invalid negative values for words F, N, P, T, and S.
        // NOTE: Negative value check is done here simply for code-efficiency.
	// Проверьте, нет ли недопустимых отрицательных значений для слов F, N, P, T и S.
        // ПРИМЕЧАНИЕ: Проверка отрицательных значений здесь выполняется просто для повышения эффективности кода.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [Word value cannot be negative]
        }
        value_words |= bit(word_bit); // Flag to indicate parameter assigned. // Флаг, указывающий на присвоенный параметр.
      
    }   
  } 
  // Parsing complete!
  

  /* -------------------------------------------------------------------------------------
     STEP 3: Error-check all commands and values passed in this block. This step ensures all of
     the commands are valid for execution and follows the NIST standard as closely as possible.
     If an error is found, all commands and values in this block are dumped and will not update
     the active system g-code modes. If the block is ok, the active system g-code modes will be
     updated based on the commands of this block, and signal for it to be executed. 
     
     Also, we have to pre-convert all of the values passed based on the modes set by the parsed
     block. There are a number of error-checks that require target information that can only be
     accurately calculated if we convert these values in conjunction with the error-checking.
     This relegates the next execution step as only updating the system g-code modes and 
     performing the programmed actions in order. The execution step should not require any 
     conversion calculations and would only require minimal checks necessary to execute.
  */

  /* NOTE: At this point, the g-code block has been parsed and the block line can be freed.
     NOTE: It's also possible, at some future point, to break up STEP 2, to allow piece-wise 
     parsing of the block on a per-word basis, rather than the entire block. This could remove 
     the need for maintaining a large string variable for the entire block and free up some memory. 
     To do this, this would simply need to retain all of the data in STEP 1, such as the new block
     data struct, the modal group and value bitflag tracking variables, and axis array indices 
     compatible variables. This data contains all of the information necessary to error-check the 
     new g-code block when the EOL character is received. However, this would break Grbl's startup
     lines in how it currently works and would require some refactoring to make it compatible.
  */  
  
  // [0. Non-specific/common error-checks and miscellaneous setup]: 
  
  // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  // command has been sent. If so, set axis command to current motion mode.

  /* -------------------------------------------------------------------------------------
     ШАГ 3: Ошибка - проверьте все команды и значения, переданные в этом блоке. Этот шаг гарантирует, что все
команды допустимы для выполнения и соответствуют стандарту NIST настолько точно, насколько это возможно.
     Если обнаружена ошибка, все команды и значения в этом блоке будут удалены и не будут обновляться
     режимы активного системного g-кода. Если с блоком все в порядке, режимы активного системного g-кода будут
обновлены на основе команд этого блока и подадут сигнал на его выполнение. 
     
     Кроме того, мы должны предварительно преобразовать все переданные значения на основе режимов, установленных анализируемым
     блок. Существует ряд проверок на наличие ошибок, для которых требуется целевая информация, которая может быть
точно рассчитана только в том случае, если мы преобразуем эти значения в сочетании с проверкой на наличие ошибок.
     Это приводит к тому, что следующий этап выполнения сводится только к обновлению режимов системного g-кода и
выполнению запрограммированных действий по порядку. Этап выполнения не должен требовать каких-либо
преобразований и потребует лишь минимальных проверок, необходимых для выполнения.
  */

  /* ПРИМЕЧАНИЕ: На этом этапе блок g-кода разобран, и строка блока может быть освобождена.
     ПРИМЕЧАНИЕ: Также возможно, в какой-то момент в будущем, разбить ШАГ 2, чтобы разрешить
разбор блока по частям на основе каждого слова, а не всего блока. Это могло бы избавить
от необходимости поддерживать большую строковую переменную для всего блока и освободить немного памяти. 
     Для этого просто необходимо сохранить все данные, приведенные на ШАГЕ 1, такие как
структура данных нового блока, переменные отслеживания модальной группы и значения bitflag, а также индексы массива axis 
     совместимые переменные. Эти данные содержат всю информацию, необходимую для проверки на ошибки
нового блока g-кода при получении символа EOL. Однако это нарушило бы запуск Grbl.
     строки в том, как он работает в настоящее время, и потребовало бы некоторого рефакторинга, чтобы сделать его совместимым.
  */  
  
  // [0. Неспецифические/распространенные ошибки -проверка и другие настройки]: 
  
  // Определите условия команды axis для неявного выполнения. Слова для Axis были переданы, но явная команда axis не была отправлена
  //. Если это так, переведите команду axis в текущий режим движения.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // Assign implicit motion-mode // Назначить неявный режим движения
  }
  
  // Check for valid line number N value. // Проверьте правильность значения номера строки N.
  if (bit_istrue(value_words,bit(WORD_N))) {
    // Line number value cannot be less than zero (done) or greater than max line number. // Значение номера строки не может быть меньше нуля (готово) или больше максимального номера строки.
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number] // [Превышает максимальный номер строки]
  }
  // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking. // ПРИМЕЧАНИЕ: Многозначное слово. Устанавливается в конце проверки на ошибки.
  
  // Track for unused words at the end of error-checking.
  // NOTE: Single-meaning value words are removed all at once at the end of error-checking, because
  // they are always used when present. This was done to save a few bytes of flash. For clarity, the
  // single-meaning value words may be removed as they are used. Also, axis words are treated in the
  // same way. If there is an explicit/implicit axis command, XYZ words are always used and are 
  // are removed at the end of error-checking.  
  
  // [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol.
  
  // [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
  //   is not defined after switching to G94 from G93.

  // Отследите неиспользуемые слова в конце проверки на наличие ошибок.
  // ПРИМЕЧАНИЕ: В конце проверки на наличие ошибок все слова, имеющие однозначное значение, удаляются сразу, потому что
  // они используются всегда, когда они присутствуют. Это было сделано для экономии нескольких байт флэш-памяти. Для наглядности в таблице
  // многозначные слова могут быть удалены по мере их использования. Кроме того, слова оси обрабатываются в
  // таким же образом. Если существует явная/неявная команда axis, всегда используются слова XYZ, которые являются 
  // удаляются в конце проверки на ошибки.  
  
  // [1. Комментарии ]: Сообщение НЕ ПОДДЕРЖИВАЕТСЯ. Обработка комментариев осуществляется по протоколу.
  
  // [2. Установите режим скорости подачи ]: При активном G1,G2/3 неявно или явно пропущено слово G93 F. Скорость подачи
  
  if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
    // ПРИМЕЧАНИЕ: G38 также может работать в обратном времени, но не определен как ошибка. Здесь добавлена проверка пропущенного слова F.
    // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
    if (axis_command == AXIS_COMMAND_MOTION_MODE) { 
      if ((gc_block.modal.motion != MOTION_MODE_NONE) || (gc_block.modal.motion != MOTION_MODE_SEEK)) {
        if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [F word missing]
      }
    }

    // ПРИМЕЧАНИЕ: Кажется излишним проверять, передается ли слово F после переключения с G94 на G93. Мы бы
    // добились того же, если бы значение скорости подачи всегда сбрасывалось на ноль и не определялось после каждого
    // обратный временной блок, поскольку команды, использующие это значение, уже выполняют неопределенные проверки. Это позволило бы
    // также выполнять другие команды, следующие за этим переключателем, и не выдавать ненужные ошибки. Этот код
    // комбинируется с указанным выше режимом подачи и приведенной ниже настройкой скорости подачи для проверки ошибок.

    // [3. Установите скорость подачи ]: Значение F отрицательное (готово).
    // - В режиме обратного времени: всегда неявно обнуляйте значение скорости подачи до и после завершения блока.
    // ПРИМЕЧАНИЕ: Если вы находитесь в режиме G93 или переключились в него с G94, просто сохраняйте значение F в качестве инициализированного нуля или передаваемого слова F 
    // значение в блоке. Если в команде движения, требующей скорости подачи, не будет передано слово F, это приведет к ошибке 
    // out при проверке ошибок в режимах движения. Однако, если в команде движения, требующей скорости подачи, не будет передано слово F, это приведет к ошибке
    // скорость подачи, мы просто переходим к следующему этапу, и значение скорости подачи в состоянии обновляется до нуля и остается неопределенным.
    
    // NOTE: It seems redundant to check for an F word to be passed after switching from G94 to G93. We would
    // accomplish the exact same thing if the feed rate value is always reset to zero and undefined after each
    // inverse time block, since the commands that use this value already perform undefined checks. This would
    // also allow other commands, following this switch, to execute and not error out needlessly. This code is 
    // combined with the above feed rate mode and the below set feed rate error-checking.

    // [3. Set feed rate ]: F is negative (done.)
    // - In inverse time mode: Always implicitly zero the feed rate value before and after block completion.
    // NOTE: If in G93 mode or switched into it from G94, just keep F value as initialized zero or passed F word 
    // value in the block. If no F word is passed with a motion command that requires a feed rate, this will error 
    // out in the motion modes error-checking. However, if no F word is passed with NO motion command that requires
    // a feed rate, we simply move on and the state feed rate value gets updated to zero and remains undefined.
  } else { // = G94
    // - In units per mm mode: If F word passed, ensure value is in mm/min, otherwise push last state value.
    // - В режиме единиц измерения на мм: Если передано слово F, убедитесь, что значение указано в мм/мин, в противном случае введите последнее значение состояния.
    if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { // Last state is also G94 // Последним состоянием также является G94
      if (bit_istrue(value_words,bit(WORD_F))) {
        if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
      } else {
        gc_block.values.f = gc_state.feed_rate; // Push last state feed rate // Скорость подачи в последнем состоянии
      }
    } // Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
    // В противном случае, переключаемся с G93 на G94, поэтому не нажимайте скорость подачи последнего состояния. Оно не определено или передано значение F-слова.
  } 
  // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking. // ПРИМЕЧАНИЕ: Многозначное слово. Устанавливается в конце проверки на ошибки.
  
  // [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(value_words,bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
  // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking.  // ПРИМЕЧАНИЕ: Многозначное слово. Устанавливается в конце проверки на ошибки.
    
  // [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
  // bit_false(value_words,bit(WORD_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

  // [6. Change tool ]: N/A
  // [7. Spindle control ]: N/A
  // [8. Coolant control ]: N/A
  // [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED.
  
  // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.

  // [6. Инструмент для изменения ]: Нет
  // [7. Управление шпинделем ]: Нет/ A
  // [8. Управление охлаждающей жидкостью ]: Нет/A
  // [9. Включение/выключение скорости подачи или переопределение скорости вращения шпинделя ]: НЕ ПОДДЕРЖИВАЕТСЯ.
  
  // [10. Задержка ]: Значение P отсутствует. Значение P отрицательное (готово.) ПРИМЕЧАНИЕ: Смотрите ниже.
  
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P word missing]
    bit_false(value_words,bit(WORD_P));
  }
  
  // [11. Set active plane ]: N/A
  switch (gc_block.modal.plane_select) {
    case PLANE_SELECT_XY:
      axis_0 = X_AXIS;
      axis_1 = Y_AXIS;
      axis_linear = Z_AXIS;
      break;
    case PLANE_SELECT_ZX:
      axis_0 = Z_AXIS;
      axis_1 = X_AXIS;
      axis_linear = Y_AXIS;
      break;
    default: // case PLANE_SELECT_YZ:
      axis_0 = Y_AXIS;
      axis_1 = Z_AXIS;
      axis_linear = X_AXIS;
  }   
            
  // [12. Установите единицы измерения длины ]: N/A
  // Предварительно преобразуйте значения координат XYZ в миллиметры, если это применимо.
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used. // Индексы осей согласованы, поэтому можно использовать цикл.
      if (bit_istrue(axis_words,bit(idx)) ) {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }
  
  // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
  // [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
  //   NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. Grbl supports G40 
  //   only for the purpose to not error when G40 is sent with a g-code program header to setup the default modes.
  
  // [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are. 
  // [G43.1 Errors]: Motion command in same line. 
  //   NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid 
  //   axis that is configured (in config.h). There should be an error if the configured axis
  //   is absent or if any of the other axis words are present.

  // [13. Компенсация радиуса резца ]: G41/42 НЕ ПОДДЕРЖИВАЕТСЯ. Ошибка, если она включена при активном G53.
  // [Ошибки G40]: Дуга G2/3 программируется после G40. Линейный ход после отключения меньше диаметра инструмента.
  // ПРИМЕЧАНИЕ: Поскольку компенсация радиуса резца никогда не включается, эти ошибки G40 не применяются. Grbl поддерживает G40 
  // только для того, чтобы избежать ошибки при отправке G40 с заголовком программы g-code для настройки режимов по умолчанию.
  
  // [14. Компенсация длины резца ]: G43 НЕ ПОДДЕРЖИВАЕТСЯ, но поддерживаются G43.1 и G49. 
  // [Ошибка G43.1]: Команда перемещения в той же строке. 
  // ПРИМЕЧАНИЕ: Хотя это явно не указано, G43.1 следует применять только к одной допустимой оси 
  //, которая настроена (в файле config.h). Должна быть ошибка, если настроенная ось
  // отсутствует или если присутствуют какие-либо другие слова оси.
  
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates called in block. // Указывает на вызов в блоке.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
      if (axis_words ^ (1<<TOOL_LENGTH_OFFSET_AXIS)) { FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR); }
    }
  }
  
  // [15. Coordinate system selection ]: *N/A. Error, if cutter radius comp is active.
  // TODO: An EEPROM read of the coordinate data may require a buffer sync when the cycle
  // is active. The read pauses the processor temporarily and may cause a rare crash. For 
  // future versions on processors with enough memory, all coordinate data should be stored
  // in memory and written to EEPROM only when there is not a cycle active.

  // [15. Выбор системы координат ]: *N/A. Ошибка, если активирован параметр настройки радиуса резания.
  // ЗАДАЧА: При активном цикле считывания данных координат из EEPROM может потребоваться синхронизация буфера
  //. Чтение временно приостанавливает работу процессора и может привести к редкому сбою. Для 
  // в будущих версиях на процессорах с достаточным объемом памяти все данные о координатах должны храниться
  // в памяти и записываться в EEPROM только тогда, когда цикл не активен.
  
  memcpy(coordinate_data,gc_state.coord_system,sizeof(gc_state.coord_system));
  if ( bit_istrue(command_words,bit(MODAL_GROUP_G12)) ) { // Check if called in block
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select,coordinate_data))) { FAIL(STATUS_SETTING_READ_FAIL); } 
    }
  }
  
  // [16. Set path control mode ]: N/A. Only G61. G61.1 and G64 NOT SUPPORTED.
  // [17. Set distance mode ]: N/A. Only G91.1. G90.1 NOT SUPPORTED.
  // [18. Set retract mode ]: NOT SUPPORTED.
  
  // [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets.
  // NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
  // commands all treat axis words differently. G10 as absolute offsets or computes current position as
  // the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
  // all the current coordinate system and G92 offsets.

  // [16. Установить режим управления траекторией ]: N/A. Только G61. G61.1 и G64 НЕ ПОДДЕРЖИВАЮТСЯ.
  // [17. Режим установки расстояния ]: N/A. Только G91.1. G90.1 НЕ ПОДДЕРЖИВАЕТСЯ.
  // [18. Установить режим втягивания ]: НЕ ПОДДЕРЖИВАЕТСЯ.
  
  // [19. Остальные немодальные действия ]: Установите флажок перейти в заданное положение, установите G10 или установите смещения осей.
  // ПРИМЕЧАНИЕ: Нам нужно разделить немодальные команды, которые используют слова оси (G10/G28/G30/G92), так как все эти
  // команды по-разному обрабатывают слова оси. G10 используется как абсолютное смещение или вычисляет текущее положение как
  // значение оси, G92 аналогично G10 L20, и G28/30 в качестве промежуточного целевого положения, при котором соблюдаются
  // все текущие смещения системы координат и G92.
  
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:  
      // [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
      // [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to nCoordSys(max 9). Axis words missing.
      // [G10 L20 Errors]: P must be 0 to nCoordSys(max 9). Axis words missing.

      // [Ошибки G10]: L отсутствует и не равно 2 или 20. Слово P отсутствует. (Выполнено отрицательное значение P)
      // [Ошибки G10 L2]: Слово R НЕ ПОДДЕРЖИВАЕТСЯ. Значение P не равно 0 для nCoordSys(максимум 9). Пропущены слова Axis.
      // [Ошибки G10 L20]: Значение P должно быть равно 0 для nCoordSys(максимум 9). Пропущены слова Axis.
      
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS) }; // [No axis words] // [Нет слов об оси]
      if (bit_isfalse(value_words,((1<<WORD_P)|(1<<WORD_L)))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P/L word missing]
      coord_select = trunc(gc_block.values.p); // Convert p value to int. // Преобразовать значение p в int.
      if (coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys] // [Больше, чем N систем]
      if (gc_block.values.l != 20) {
        if (gc_block.values.l == 2) {
          if (bit_istrue(value_words,bit(WORD_R))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G10 L2 R not supported]// [G10 L2 R не поддерживается] 
        } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported L]
      }
      bit_false(value_words,(bit(WORD_L)|bit(WORD_P)));
      
      // Determine coordinate system to change and try to load from EEPROM. // Определите систему координат, которую нужно изменить, и попробуйте загрузить из EEPROM.
      if (coord_select > 0) { coord_select--; } // Adjust P1-P6 index to EEPROM coordinate data indexing. // Отрегулируйте индекс P1-P6 в соответствии с индексацией координатных данных EEPROM.
      else { coord_select = gc_block.modal.coord_select; } // Index P0 as the active coordinate system  // Индекс P0 в качестве активной системы координат
      if (!settings_read_coord_data(coord_select,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); } // [EEPROM read fail] // [Ошибка чтения EEPROM]
    
      // Pre-calculate the coordinate data changes. NOTE: Uses parameter_data since coordinate_data may be in use by G54-59. // Предварительно рассчитайте изменения данных о координатах. ПРИМЕЧАНИЕ: Используется параметр_data, поскольку данные о координатах могут использоваться G54-59.
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used. // Индексы осей согласованы, поэтому можно использовать цикл.
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system. // Обновить оси, определенные только в блоке. Всегда в машинных координатах. Можно изменить неактивную систему
        if (bit_istrue(axis_words,bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20: Update coordinate system axis at current position (with modifiers) with programmed value // L20: Обновить ось системы координат в текущем положении (с модификаторами) с запрограммированным значением
            parameter_data[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
            if (idx == TOOL_LENGTH_OFFSET_AXIS) { parameter_data[idx] -= gc_state.tool_length_offset; }
          } else {
            // L2: Update coordinate system axis to programmed value. // L2: Обновить ось системы координат до запрограммированного значения.
            parameter_data[idx] = gc_block.values.xyz[idx]; 
          }
        }
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92 Errors]: No axis words. // [Ошибки G92]: Нет слов об оси.
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
    
      // Update axes defined only in block. Offsets current system to defined value. Does not update when
      // active coordinate system is selected, but is still active unless G92.1 disables it.

      // Обновляет оси, определенные только в блоке. Смещает текущую систему координат на заданное значение. Не обновляется, когда выбрана
      // активная система координат, но остается активной, если G92.1 не отключит ее.
      
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used. // Индексы осей согласованы, поэтому можно использовать цикл.
        if (bit_istrue(axis_words,bit(idx)) ) {
          gc_block.values.xyz[idx] = gc_state.position[idx]-coordinate_data[idx]-gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] -= gc_state.tool_length_offset; }
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;
      
    default:

      // At this point, the rest of the explicit axis commands treat the axis values as the traditional
      // target position with the coordinate system offsets, G92 offsets, absolute override, and distance
      // modes applied. This includes the motion mode commands. We can now pre-compute the target position.
      // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.

      // На этом этапе остальные явные команды axis обрабатывают значения оси как традиционные
      // целевое положение со смещениями в системе координат, смещениями G92, абсолютным переопределением и расстоянием
      // применяются режимы. Сюда входят команды режима движения. Теперь мы можем предварительно рассчитать целевую позицию.
      // ПРИМЕЧАНИЕ: Смещения инструмента могут быть добавлены к этим преобразованиям, когда/ если будет добавлена эта функция.
      
      if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO block any axis command. // TLO блокирует любую команду axis.
        if (axis_words) {
          for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space. // Индексы осей согласованы, поэтому для экономии места на флэш-памяти можно использовать loop.
            if ( bit_isfalse(axis_words,bit(idx)) ) {
              gc_block.values.xyz[idx] = gc_state.position[idx]; // No axis word in block. Keep same axis position. // В блоке нет слова "ось". Сохраняйте прежнее положение оси.
            } else {
              // Update specified value according to distance mode or ignore if absolute override is active.
              // NOTE: G53 is never active with G28/30 since they are in the same modal group.
	      // Обновите указанное значение в соответствии с режимом расстояния или проигнорируйте, если активировано абсолютное переопределение.
              // ПРИМЕЧАНИЕ: G53 никогда не активируется с G28/30, поскольку они находятся в одной модальной группе.
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // Apply coordinate offsets based on distance mode.
		// Примените смещения координат в зависимости от режима расстояния.
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {
                  gc_block.values.xyz[idx] += coordinate_data[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // Incremental mode // Инкрементальный режим
                  gc_block.values.xyz[idx] += gc_state.position[idx];
                }
              }
            }
          }
        }
      }
          
      // Check remaining non-modal commands for errors.
      // Проверьте остальные немодальные команды на наличие ошибок.
      switch (gc_block.non_modal_command) {        
        case NON_MODAL_GO_HOME_0:
	  // [Ошибки G28]: Включена компенсация положения фрезы. 
          // Извлеките данные о начальном положении фрезы G28 (в координатах станка) из EEPROM
          // [G28 Errors]: Cutter compensation is enabled. 
          // Retreive G28 go-home position data (in machine coordinates) from EEPROM
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; } // Set to none if no intermediate motion. // Установите значение "нет", если промежуточное движение отсутствует.
          if (!settings_read_coord_data(SETTING_INDEX_G28,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_GO_HOME_1:
          // [G30 Errors]: Cutter compensation is enabled. 
          // Retreive G30 go-home position data (in machine coordinates) from EEPROM
	  // [Ошибки G30]: Включена компенсация положения фрезы. 
          // Извлеките данные о начальном положении фрезы G30 (в координатах станка) из EEPROM
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; } // Set to none if no intermediate motion. // Установите значение "нет", если промежуточное движение отсутствует.
          if (!settings_read_coord_data(SETTING_INDEX_G30,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_SET_HOME_0: case NON_MODAL_SET_HOME_1:
          // [G28.1/30.1 Errors]: Cutter compensation is enabled. 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
	  // [Ошибки G28.1/30.1]: Включена компенсация резца. 
          // ПРИМЕЧАНИЕ: Если здесь указаны слова axis, они интерпретируются как неявный режим движения.
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET: 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
	  // ПРИМЕЧАНИЕ: Если здесь передаются слова axis, они интерпретируются как неявный режим движения.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
          // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
	 // [Ошибки G53]: G0 и G1 неактивны. Включена компенсация резца.
          // ПРИМЕЧАНИЕ: Все явные команды axis word относятся к этой модальной группе. Поэтому неявная проверка не требуется. 
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 not active]
          }
          break;
      }
  }
      
  // [20. Motion modes ]:
  // [20. Режимы движения ]:
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
    // [G80 Errors]: Axis word exist and are not used by a non-modal command.
    // [Ошибки G80]: Слово Axis существует и не используется немодальной командой.
    if ((axis_words) && (axis_command != AXIS_COMMAND_NON_MODAL)) { 
      FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); // [No axis words allowed]
    }

  // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or 
  // was explicitly commanded in the g-code block.
    // Проверьте остальные режимы движения, если слово оси неявно (существует и не используется G10/28/30/92) или 
  // было явно задано в блоке g-кода.
  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {
  
    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
      // [G0 Errors]: Axis letter not configured or without real value (done.)
      // Axis words are optional. If missing, set axis command flag to ignore execution.
      // [Ошибки G0]: Буква Axis не настроена или не имеет реального значения (готово.)
      // Слова Axis необязательны. Если они отсутствуют, установите флаг команды axis для игнорирования выполнения.
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

      // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
      // the value must be positive. In inverse time mode, a positive value must be passed with each block.
      // [Ошибка G0]: Буква Axis не настроена или не имеет реального значения (готово.)
      // Слова Axis необязательны. Если они отсутствуют, установите флажок команды axis для игнорирования выполнения.
      //если (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
    } else {
      // Проверьте, задана ли скорость подачи для требуемых режимов движения.
      // Check if feed rate is defined for the motion modes that require it.
      if (gc_block.values.f == 0.0) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [Feed rate undefined]
     
      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR: 
          // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
          // Axis words are optional. If missing, set axis command flag to ignore execution.
	  // [Ошибки G1]: Скорость подачи не определена. Буква Axis не настроена или не имеет реального значения.
          // Слова Axis необязательны. Если они отсутствуют, установите флажок команды axis для игнорирования выполнения.
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

          break;
        case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
          // [G2/3 Errors All-Modes]: Feed rate undefined.
          // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
          // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current 
          //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).   
          // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.        
          // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.

	  // [Ошибки G2/3 для всех режимов]: Скорость подачи не определена.
          // [Ошибки G2/3 для режима радиуса]: В выбранной плоскости нет обозначений осей. Целевая точка совпадает с текущей.
          // [Ошибки режима смещения G2/3]: Отсутствуют обозначения осей и/или смещения в выбранной плоскости. Радиус до текущей точки
	  // и радиус до целевой точки отличаются более чем на 0,002 мм (предел электромагнитной совместимости 0,5 мм ИЛИ 0,005 мм и радиус 0,1%).   
          // [Ошибки режима полного цикла G2/3]: НЕ ПОДДЕРЖИВАЕТСЯ. Существуют обозначения осей. Смещения не запрограммированы. P должно быть целым числом.        
          // ПРИМЕЧАНИЕ: Для трассировки дуги требуются как радиус, так и смещения, которые предварительно вычисляются с помощью проверки ошибок.
        
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words] // [Нет слов об оси]
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } // [No axis words in plane] // [В плоскости нет слов об оси]
        
          // Calculate the change in position along each selected axis // Вычислить изменение положения вдоль каждой выбранной оси
          float x,y;
          x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // Delta x between current position and target // Разница x между текущим положением и целью
          y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // Delta y between current position and target // Delta y between current position and target

          if (value_words & bit(WORD_R)) { // Arc Radius Mode  // Режим радиуса дуги
            bit_false(value_words,bit(WORD_R));
            if (gc_check_same_position(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target] // [Недопустимая цель]
          
            // Convert radius value to proper units. // Преобразовать значение радиуса в соответствующие единицы измерения.
            if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
            /*  We need to calculate the center of the circle that has the designated radius and passes
                through both the current position and the target position. This method calculates the following
                set of equations where [x,y] is the vector from current to target position, d == magnitude of 
                that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the 
                length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point 
                [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.
    
                d^2 == x^2 + y^2
                h^2 == r^2 - (d/2)^2
                i == x/2 - y/d*h
                j == y/2 + x/d*h
    
                                                                     O <- [i,j]
                                                                  -  |
                                                        r      -     |
                                                            -        |
                                                         -           | h
                                                      -              |
                                        [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                  | <------ d/2 ---->|
              
                C - Current position
                T - Target position
                O - center of circle that pass through both C and T
                d - distance from C to T
                r - designated radius
                h - distance from center of CT to O
    
                Expanding the equations:
 
                d -> sqrt(x^2 + y^2)
                h -> sqrt(4 * r^2 - x^2 - y^2)/2
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
   
                Which can be written:
    
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
    
                Which we for size and speed reasons optimize to:
 
                h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                i = (x - (y * h_x2_div_d))/2
                j = (y + (x * h_x2_div_d))/2       
            */      

            // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
            // than d. If so, the sqrt of a negative number is complex and error out.

	    /* Нам нужно вычислить центр окружности, которая имеет заданный радиус и проходит
                как через текущее, так и через целевое положение. Этот метод вычисляет следующее
                набор уравнений, где [x,y] - вектор от текущего до целевого положения, d == величина
		этого вектора, h == гипотенуза треугольника, образованного радиусом окружности, расстояние до
		центра вектора перемещения. Вектор, перпендикулярный вектору перемещения [-y,x], масштабируется до 
                длина h [-y/d*h, x/d*h] и добавляется к центру вектора перемещения [x/2,y/2], чтобы сформировать новую точку 
                [i,j] в точке [x/2-y/d*h, y/2+x/d*h], которая будет центром нашей дуги.
    
                d^ 2 == x^ 2 + y ^ 2
                h^2 == r^2 - (d/2)^2
                i == x/2 - y/d*h
                j == y/2 + x/d*h
    
                                                 O <- [i,j]
                                                                  -  |
                                                        r      -     |
                                                            -        |
                                                         -           | h
                                                      -              |
                                        [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                  | <------ d/2 ---->|
              
                C - Текущее положение
                T - Целевое положение
                O - центр окружности, проходящей через C и T
                d - расстояние от C до T
                r - обозначенный радиус
                h - расстояние от центра CT до O
    
                Расширяем уравнения:
 
                d -> sqrt(x ^ 2 + y ^ 2)
                h -> sqrt(4 * r ^ 2 - x ^ 2 - y ^ 2)/2
                i -> (x - (y * sqrt(4 * r^ 2 - x^ 2 - y^2)) / sqrt(x ^ 2 + y^ 2)) / 2 
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
   
                Который может быть записан:
    
                i -> (x - (y * sqrt(4 * r^ 2 - x^ 2 - y^2))/sqrt(x ^ 2 + y^2))/2
                j -> (y + (x * sqrt(4 * r^ 2 - x^ 2 - y^2))/sqrt(x ^ 2 + y^2))/2
    
                Который мы по соображениям размера и скорости оптимизируем до:
 
                h_x2_div_d = sqrt(4 * r^ 2 - x ^ 2 - y^ 2)/sqrt(x ^ 2 + y ^ 2)
                i = (x - (y * h_x2_div_d))/2
                j = (y + (x * h_x2_div_d))/2       
            */      

            // Сначала используйте h_x2_div_d для вычисления 4* h ^ 2, чтобы проверить, является ли оно отрицательным или r меньше
            //, чем d. Если это так, то значение sqrt отрицательного числа является сложным и содержит ошибку.
	    
            float h_x2_div_d = 4.0 * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [Arc radius error]
    
            // Finish computing h_x2_div_d.
	   // Завершите вычисление h_x2_div_d.
            h_x2_div_d = -sqrt(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
	    // Переверните знак h_x2_div_d, если окружность расположена против часовой стрелки (см. рисунок ниже)
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }  

            /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
               the left hand circle will be generated - when it is negative the right hand circle is generated.
          
                                                                   T  <-- Target position
                                                   
                                                                   ^ 
                        Clockwise circles with this center         |          Clockwise circles with this center will have
                        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                         \         |          /   
            center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                   |
                                                                   |
                                                   
                                                                   C  <-- Current position                                
            */  
            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
            // even though it is advised against ever generating such circles in a single line of g-code. By 
            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
            // travel and thus we get the unadvisably long arcs as prescribed.

	    /* Окружность против часовой стрелки расположена слева от целевого направления. При положительном смещении
	       будет создан левый круг, при отрицательном - правый.
          
                                                                   T <- Целевое положение
                                                   
                                                                   ^ 
                        Круги по часовой стрелке с этим центром    |    Круги по часовой стрелке с этим центром будут иметь
                        будет иметь > 180° углового перемещения    | < 180° углового перемещения, что хорошо!
                                                         \         |          /   
      центр дуги при положительном значении h_x2_div_d -> x <----- | ----->  x <- центр дуги при отрицательном значении h_x2_div_d
                                                                   |
                                                                   |
                                                   
                                                                   C <-- Текущее положение                                
            */  
            // Отрицательное значение R в g-коде означает "Я хочу окружность с углом поворота более 180 градусов" (поди разберись!), 
            // хотя не рекомендуется создавать такие окружности в одной строке g-кода. Путем
	    // инвертирования знака h_x2_div_d центр окружностей помещается на противоположной стороне линии
	    // перемещения, и, таким образом, мы получаем нецелесообразно длинные дуги, как предписано.
	    
            if (gc_block.values.r < 0) { 
                h_x2_div_d = -h_x2_div_d; 
                gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc // Завершено с помощью r. Присвоено положительное значение для mc_arc
            }        
            // Complete the operation by calculating the actual center of the arc // Завершите операцию вычислением фактического центра дуги
            gc_block.values.ijk[axis_0] = 0.5*(x-(y*h_x2_div_d));
            gc_block.values.ijk[axis_1] = 0.5*(y+(x*h_x2_div_d));
          
          } else { // Arc Center Format Offset Mode   // Режим смещения формата центра дуги
            if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [No offsets in plane] // [Нет смещений в плоскости]
            bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));  
          
            // Convert IJK values to proper units. // Преобразуйте значения IJK в соответствующие единицы измерения.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space. // Индексы осей согласованы, поэтому для экономии места на флэш-памяти можно использовать loop.
                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
              }
            }         

            // Arc radius from center to target // Радиус дуги от центра до цели 
            x -= gc_block.values.ijk[axis_0]; // Delta x between circle center and target // Дельта x между центром окружности и целью
            y -= gc_block.values.ijk[axis_1]; // Delta y between circle center and target // Дельта y между центром окружности и целью
            float target_r = hypot_f(x,y); 

            // Compute arc radius for mc_arc. Defined from current location to center. // Вычислить радиус дуги для mc_arc. Определяется от текущего местоположения до центра. 
            gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]); 
            
            // Compute difference between current location and target radii for final error-checks. // Вычислите разницу между текущим местоположением и целевым радиусом для окончательной проверки на ошибки.
            float delta_r = fabs(target_r-gc_block.values.r);
            if (delta_r > 0.005) { 
              if (delta_r > 0.5) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.5mm // [Погрешность определения дуги] > 0,5 мм
              if (delta_r > (0.001*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.005mm AND 0.1% radius // [Погрешность определения дуги] > 0,005 мм И радиус 0,1%
            }
          }
          break;
        case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_TOWARD_NO_ERROR:
        case MOTION_MODE_PROBE_AWAY: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
          // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
          //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
          //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to 
          //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.

	  // [Ошибки G38]: Цель - тот же ток. Нет слов об оси. Включена компенсация резца. Скорость подачи
          // не определена. Пробник запущен. ПРИМЕЧАНИЕ: Проверка пробника перенесена в цикл проверки. Вместо возврата
          // при обнаружении ошибки он выдает сигнал тревоги, предотвращающий дальнейшее перемещение зонда. Это также сделано для того, чтобы
	  // позволить буферу планировщика опустеть и отключить триггер зонда перед следующим циклом зондирования.
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (gc_check_same_position(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          break;
      } 
    }
  }
  
  // [21. Program flow ]: No error checks required.

  // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
  // radius mode, or axis words that aren't used in the block.

  // [21. Выполнение программы ]: Проверка на ошибки не требуется.

  // [0. Проверка на неконкретные ошибки]: Полная проверка неиспользуемых значений слов, т.е. IJK, используемых в режиме arc
  // radius, или слов оси, которые не используются в блоке.
  
  bit_false(value_words,(bit(WORD_N)|bit(WORD_F)|bit(WORD_S)|bit(WORD_T))); // Remove single-meaning value words. 
  if (axis_command) { bit_false(value_words,(bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words. 
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

   
  /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
  */ 
  
  // [0. Non-specific/common error-checks and miscellaneous setup]:

  /* -------------------------------------------------------------------------------------
     ШАГ 4: ВЫПОЛНИТЬ!!
     Предполагается, что все проверки на ошибки завершены и никаких режимов сбоя не существует. Нам просто
     нужно обновить состояние и выполнить блок в соответствии с порядком выполнения.
  */ 
  
  // [0. Неспецифические/распространенные проверки на ошибки и прочие настройки]:
  gc_state.line_number = gc_block.values.n;
  
  // [1. Comments feedback ]:  NOT SUPPORTED
  
  // [2. Set feed rate mode ]:
  // [1. Комментарии и обратная связь ]: НЕ ПОДДЕРЖИВАЕТСЯ
  
  // [2. Установите режим скорости подачи ]:
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  
  // [3. Set feed rate ]:
  // [3. Установите скорость подачи ]:
  gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.

  // [4. Set spindle speed ]:
  // [4. Установите частоту вращения шпинделя ]:
  if (gc_state.spindle_speed != gc_block.values.s) { 
    // Update running spindle only if not in check mode and not already enabled.
    // Обновляйте запущенный шпиндель, только если он не находится в режиме проверки и еще не включен.
    if (gc_state.modal.spindle != SPINDLE_DISABLE) { spindle_run(gc_state.modal.spindle, gc_block.values.s); }
    gc_state.spindle_speed = gc_block.values.s; 
  }
    
  // [5. Select tool ]: NOT SUPPORTED. Only tracks tool value.
  // [5. Выберите инструмент ]: НЕ ПОДДЕРЖИВАЕТСЯ. Отслеживается только значение инструмента.
  gc_state.tool = gc_block.values.t;

// [6. Смена инструмента ]: НЕ ПОДДЕРЖИВАЕТСЯ

  // [7. Управление шпинделем ]:
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
    // Update spindle control and apply spindle speed when enabling it in this block.
    // Обновите управление шпинделем и измените частоту вращения шпинделя, включив его в этом блоке.
    spindle_run(gc_block.modal.spindle, gc_state.spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;    
  }

  // [8. Coolant control ]:
  // [8. Управление охлаждающей жидкостью ]:
  if (gc_state.modal.coolant != gc_block.modal.coolant) {
    coolant_run(gc_block.modal.coolant);
    gc_state.modal.coolant = gc_block.modal.coolant;
  }
  
  // [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED

  // [10. Dwell ]:
  // [9. Включение/выключение скорости подачи или переопределение шпинделя ]: НЕ ПОДДЕРЖИВАЕТСЯ

  // [10. Пребывать ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }
  
  // [11. Set active plane ]:
  // [11. Установить активную плоскость ]:
  gc_state.modal.plane_select = gc_block.modal.plane_select;  

  // [12. Set length units ]:
  // [12. Установленные единицы измерения длины ]:
  gc_state.modal.units = gc_block.modal.units;

  // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
  // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.

  // [14. Cutter length compensation ]: G43.1 and G49 supported. G43 NOT SUPPORTED.
  // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
  // of execution. The error-checking step would simply load the offset value into the correct
  // axis of the block XYZ value array.
  // [13. Компенсация радиуса резания ]: G41/42 НЕ ПОДДЕРЖИВАЕТСЯ
  // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // ПРИМЕЧАНИЕ: Не требуется, так как всегда отключен.

  // [14. Компенсация длины резца ]: Поддерживаются G43.1 и G49. G43 НЕ ПОДДЕРЖИВАЕТСЯ.
  // ПРИМЕЧАНИЕ: Если бы G43 поддерживался, его работа ничем бы не отличалась от работы G43.1 с точки зрения исполнения.
  // На этапе проверки ошибок значение смещения будет просто загружено в правильную ось
  // массива значений блока XYZ.
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates a change.
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) { // G43.1
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
    } else { // G49
      gc_state.tool_length_offset = 0.0;
    }
  }
  
  // [15. Coordinate system selection ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,coordinate_data,sizeof(coordinate_data));
  }
  
  // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
  // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.
  
  // [17. Set distance mode ]:
  // [16. Установить режим управления путями ]: G61.1/G64 НЕ ПОДДЕРЖИВАЕТСЯ
  // gc_state.modal.control = gc_block.modal.control; // ПРИМЕЧАНИЕ: Всегда по умолчанию.
  
  // [17. Установите режим дистанционирования ]:
  gc_state.modal.distance = gc_block.modal.distance;
  
  // [18. Set retract mode ]: NOT SUPPORTED
    
  // [19. Go to predefined position, Set G10, or Set axis offsets ]:
  // [18. Установить режим втягивания ]: НЕ ПОДДЕРЖИВАЕТСЯ
    
  // [19. Перейдите в заданное положение, установите G10 или задайте смещения осей ]:
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:    
      settings_write_coord_data(coord_select,parameter_data);
      // Обновите системную систему координат, если она активна в данный момент.
      // Update system coordinate system if currently active.
      if (gc_state.modal.coord_select == coord_select) { memcpy(gc_state.coord_system,parameter_data,sizeof(parameter_data)); }
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1: 
      // Move to intermediate position before going home. Obeys current coordinate system and offsets 
      // and absolute and incremental modes.
      // Переместитесь в промежуточное положение, прежде чем отправиться домой. Соответствует текущей системе координат и смещениям 
      // а также абсолютному и инкрементальному режимам.
      if (axis_command) {
        #ifdef USE_LINE_NUMBERS
          mc_line(gc_block.values.xyz, -1.0, false, gc_state.line_number);
        #else
          mc_line(gc_block.values.xyz, -1.0, false);
        #endif
      }
      #ifdef USE_LINE_NUMBERS
        mc_line(parameter_data, -1.0, false, gc_state.line_number); 
      #else
        mc_line(parameter_data, -1.0, false); 
      #endif
      memcpy(gc_state.position, parameter_data, sizeof(parameter_data));
      break;
    case NON_MODAL_SET_HOME_0: 
      settings_write_coord_data(SETTING_INDEX_G28,gc_state.position);
      break;
    case NON_MODAL_SET_HOME_1:
      settings_write_coord_data(SETTING_INDEX_G30,gc_state.position);
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      memcpy(gc_state.coord_offset,gc_block.values.xyz,sizeof(gc_block.values.xyz));
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET: 
      clear_vector(gc_state.coord_offset); // Disable G92 offsets by zeroing offset vector. // Отключите смещения G92, обнулив вектор смещения.
      break;
  }


  // [20. Режимы движения ]:
  // ПРИМЕЧАНИЕ: Команды G10,G28,G30,G92 блокируют и не позволяют использовать слова axis в режимах движения. 
  // Вводите режимы движения, только если в блоке есть слова axis или командное слово режима движения.
  
  // [20. Motion modes ]:
  // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes. 
  // Enter motion modes only if there are axis words or a motion mode command word in the block.
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      switch (gc_state.modal.motion) {
        case MOTION_MODE_SEEK:
          #ifdef USE_LINE_NUMBERS
            mc_line(gc_block.values.xyz, -1.0, false, gc_state.line_number);
          #else
            mc_line(gc_block.values.xyz, -1.0, false);
          #endif
          break;
        case MOTION_MODE_LINEAR:
          #ifdef USE_LINE_NUMBERS
            mc_line(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, gc_state.line_number);
          #else
            mc_line(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate);
          #endif
          break;
        case MOTION_MODE_CW_ARC: 
          #ifdef USE_LINE_NUMBERS
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, true, gc_state.line_number);  
          #else
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, true); 
          #endif
          break;        
        case MOTION_MODE_CCW_ARC:
          #ifdef USE_LINE_NUMBERS
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, false, gc_state.line_number);  
          #else
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, false); 
          #endif
          break;
        case MOTION_MODE_PROBE_TOWARD: 
          // NOTE: gc_block.values.xyz is returned from mc_probe_cycle with the updated position value. So
          // upon a successful probing cycle, the machine position and the returned value should be the same.
	  // ПРИМЕЧАНИЕ: gc_block.values.xyz возвращается из mc_probe_cycle с обновленным значением положения. Таким образом,
	  // после успешного цикла проверки положение машины и возвращаемое значение должны совпадать.
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, false, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, false);
          #endif
          break;
        case MOTION_MODE_PROBE_TOWARD_NO_ERROR:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, true, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, true);
          #endif
          break;
        case MOTION_MODE_PROBE_AWAY:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, false, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, false);
          #endif
          break;
        case MOTION_MODE_PROBE_AWAY_NO_ERROR:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, true, gc_state.line_number);
          #else        
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, true);
          #endif
      }

      // С точки зрения анализатора, позиция теперь == целевая. На самом деле
      // система управления движением может все еще обрабатывать действие и реальное положение инструмента
      // в любом промежуточном местоположении.
      // As far as the parser is concerned, the position is now == target. In reality the
      // motion control system might still be processing the action and the real tool position
      // in any intermediate location.
      memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
    }
  }

  // [21. Выполнение программы ]:
  // M0,M1,M2,M30: Выполнение действий, не связанных с выполнением программы. Во время паузы программы буфер может 
  // пополняться и может быть возобновлен только командой запуска цикла во время выполнения.
  
  // [21. Program flow ]:
  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may 
  // refill and can only be resumed by the cycle start run-time command.
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) { 
	protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
	if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
	  if (sys.state != STATE_CHECK_MODE) {
		bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); // Use feed hold for program pause. // Используйте удержание подачи для приостановки программы.
		protocol_execute_realtime(); // Execute suspend. // Выполнить приостановку.
	  }
	} else { // == PROGRAM_FLOW_COMPLETED
	  // Upon program complete, only a subset of g-codes reset to certain defaults, according to 
	  // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
	  // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
	  // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.
	  // == PROGRAM_FLOW_COMPLETED
	  // После завершения программы только часть g-кодов сбрасывается на определенные значения по умолчанию, согласно 
	  // Описаниям завершения программы и тестированию в LinuxCNC. Только модальные группы [G-код 1,2,3,5,7,12]
	  // и [M-код 7,8,9] изменить на [G1,G17,G90,G94,G40,G54,M5,M9,M48]. Остальные модальные группы
	  // [G-код 4,6,8,10,13,14,15] и [M-код 4,5,6], а также модальные слова [F,S,T,H] не сбрасываются.
	  gc_state.modal.motion = MOTION_MODE_LINEAR;
	  gc_state.modal.plane_select = PLANE_SELECT_XY;
	  gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
	  gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
	  // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
	  gc_state.modal.coord_select = 0; // G54
	  gc_state.modal.spindle = SPINDLE_DISABLE;
	  gc_state.modal.coolant = COOLANT_DISABLE;
	  // gc_state.modal.override = OVERRIDE_DISABLE; // Not supported.

	  // Выполните изменение координат и остановите шпиндель/охлаждающую жидкость.
	  // Execute coordinate change and spindle/coolant stop.
	  if (sys.state != STATE_CHECK_MODE) {
		if (!(settings_read_coord_data(gc_state.modal.coord_select,coordinate_data))) { FAIL(STATUS_SETTING_READ_FAIL); } 
		memcpy(gc_state.coord_system,coordinate_data,sizeof(coordinate_data));
		spindle_stop();
		coolant_stop();		
	  }
	  
	  report_feedback_message(MESSAGE_PROGRAM_END);
	}
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // Reset program flow. // Сброс хода выполнения программы.
  }

  // TODO: % для обозначения запуска программы.
  // TODO: % to denote start of program.
  return(STATUS_OK);
}
        

/* 
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches
   
   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {*M7} enable mist coolant (* Compile-option)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/

/*
Не поддерживается:

  - Сохраненные циклы
  - Коррекция радиуса инструмента
  - Оси A,B,C
  - Вычисление выражений
  - Переменные
  - Переопределяющее управление (TBD)
  - Смена инструмента
  - Переключатели
   
   (*) Указывает необязательный параметр, включаемый через config.h и повторно компилируемый
   группа 0 = {G92.2, G92.3} (Немодальный: отмена и повторное включение смещений G92)
   группа 1 = {G81 - G89} (Режимы движения: постоянные циклы)
   группа 4 = {M1} (Необязательная остановка, игнорируется)
   группа 6 = {M6} (смена инструмента)
   группа 7 = {G41, G42} компенсация радиуса резца (поддерживается G40)
   группа 8 = {G43} смещение длины инструмента (поддерживаются G43.1/G49)
   группа 8 = {*M7} включение распыления охлаждающей жидкости (* Опция компиляции)
   группа 9 = {M48, M49} включение/выключение переключателей подачи и скорости
группа 10 = {G98, G99} режим возврата в режим сохраненных циклов
   группа 13 = {G61.1, G64} режим управления траекторией (поддерживается G61)
*/
