/*
  config.h - compile time configuration
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
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
  
// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.

// Этот файл содержит конфигурации во время компиляции для внутренней системы Grbl. По большей частипользователям
// не нужно будет напрямую изменять их, но они здесь для конкретных нужд, т.е.
// настройка производительности или приспособление к нестандартным машинам.

// ВАЖНО: Для распространения любых изменений здесь требуется полная перекомпиляция исходного кода.

#ifndef config_h
#define config_h



// Default settings. Used when resetting EEPROM. Change to desired name in defaults.h
// Настройки по умолчанию. Используется при сбросе EEPROM. Измените название на желаемое в defaults.h
#define DEFAULTS_GENERIC

// Serial baud rate
// Скорость серийного порта
#define BAUD_RATE 115200

// Default cpu mappings. Grbl officially supports the Arduino Uno only. Other processor types
// may exist from user-supplied templates or directly user-defined in cpu_map.h
// Сопоставления процессоров по умолчанию. Grbl официально поддерживает только Arduino Uno. Другие типы процессоров
// могут быть получены из предоставленных пользователем шаблонов или непосредственно определены пользователем в cpu_map.h
#define CPU_MAP_ELRON_UNO_AMUR // Arduino Uno CPU

// Определите специальные символы команд реального времени. Эти символы "выбираются" непосредственно из потока данных
// для последовательного чтения и не передаются в анализатор выполнения строки grbl. Выберите символы
//, которые не используются и не должны использоваться в потоковой программе g-code. Могут использоваться управляющие символы ASCII 
//, если они доступны для настройки пользователем. Кроме того, для интерфейсных программ могут быть выбраны расширенные коды ASCII (>127), которые никогда не используются в программах с g-кодом 
//.
// ПРИМЕЧАНИЕ: В случае изменения обновите справочное сообщение в report.c вручную.

// Define realtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed g-code program. ASCII control characters may be 
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in 
// g-code programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.

#define CMD_STATUS_REPORT '?'
#define CMD_FEED_HOLD '!'
#define CMD_CYCLE_START '~'
#define CMD_RESET 0x18 // ctrl-x.
#define CMD_SAFETY_DOOR '@'

// Если функция самонаведения включена, функция блокировки инициализации самонаведения переводит Grbl в аварийное состояние при включении питания. Это вынуждает
// пользователя выполнить цикл самонаведения (или переопределить блокировки), прежде чем делать что-либо еще. Это
// в основном функция безопасности, напоминающая пользователю о необходимости вернуться домой, поскольку Grbl не знает, где он находится.

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up. This forces
// the user to perform the homing cycle (or override the locks) before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
#define HOMING_INIT_LOCK // Comment to disable

// Определите шаблоны циклов наведения с помощью битовых масок. Цикл наведения сначала выполняет поисковый режим
// для быстрого включения концевых выключателей, затем следует более медленный режим определения местоположения и завершается коротким движением
// отводящим движением для отключения концевых выключателей. Выполняются следующие определения HOMING_CYCLE_x 
// в порядке, начинающемся с суффикса 0, и завершается процедура наведения только для указанных осей. Если
// ось опущена в определении, она не будет возвращена в исходное положение, и система не обновит ее положение.
// Это означает, что это позволяет пользователям с нестандартными декартовыми станками, такими как токарный станок (x, затем z,
// без y), настраивать поведение цикла самонаведения в соответствии со своими потребностями. 
// ПРИМЕЧАНИЕ: Цикл наведения разработан таким образом, чтобы разрешить совместное использование предельных выводов, если оси не совпадают
// цикл, но для этого необходимо изменить некоторые настройки выводов в файле cpu_map.h. Например, по умолчанию используется режим наведения
// cycle может совместно использовать вывод ограничения Z с выводами ограничения X или Y, поскольку они находятся в разных циклах.
// При совместном использовании вывода освобождается ценный вывод ввода-вывода для других целей. Теоретически, все оси ограничивают контакты
// может быть уменьшен до одного штифта, если все оси фиксируются отдельными циклами, или наоборот, все три оси
// на отдельном штифте, но фиксируются за один цикл. Также следует отметить, что функция жестких ограничений 
// на это не повлияет совместное использование контактов.
// ПРИМЕЧАНИЕ: Значения по умолчанию установлены для традиционного 3-осевого станка с ЧПУ. Сначала очистите ось Z, затем X и Y.

// Define the homing cycle patterns with bitmasks. The homing cycle first performs a search mode
// to quickly engage the limit switches, followed by a slower locate mode, and finished by a short
// pull-off motion to disengage the limit switches. The following HOMING_CYCLE_x defines are executed 
// in order starting with suffix 0 and completes the homing routine for the specified-axes only. If
// an axis is omitted from the defines, it will not home, nor will the system update its position.
// Meaning that this allows for users with non-standard cartesian machines, such as a lathe (x then z,
// with no y), to configure the homing cycle behavior to their needs. 
// NOTE: The homing cycle is designed to allow sharing of limit pins, if the axes are not in the same
// cycle, but this requires some pin settings changes in cpu_map.h file. For example, the default homing
// cycle can share the Z limit pin with either X or Y limit pins, since they are on different cycles.
// By sharing a pin, this frees up a precious IO pin for other purposes. In theory, all axes limit pins
// may be reduced to one pin, if all axes are homed with seperate cycles, or vice versa, all three axes
// on separate pin, but homed in one cycle. Also, it should be noted that the function of hard limits 
// will not be affected by pin sharing.
// NOTE: Defaults are set for a traditional 3-axis CNC machine. Z-axis first to clear, followed by X & Y.
#define HOMING_CYCLE_0 (1<<Z_AXIS)                // REQUIRED: First move Z to clear workspace. // ТРЕБУЕТСЯ: Сначала переместите Z, чтобы очистить рабочее пространство.
#define HOMING_CYCLE_1 ((1<<X_AXIS)|(1<<Y_AXIS))  // OPTIONAL: Then move X,Y at the same time. // НЕОБЯЗАТЕЛЬНО: Затем переместите X и Y одновременно.
// #define HOMING_CYCLE_2                         // OPTIONAL: Uncomment and add axes mask to enable // НЕОБЯЗАТЕЛЬНО: Раскомментируйте и добавьте маску осей, чтобы включить

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or 
// greater.

// Количество циклов самонаведения, выполненных после первоначального нажатия машиной на концевые выключатели.
// Это помогает предотвратить превышение скорости и должно улучшить повторяемость. Это значение должно быть равно единице или
// больше.
#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)

// After homing, Grbl will set by default the entire machine space into negative space, as is typical
// for professional CNC machines, regardless of where the limit switches are located. Uncomment this 
// define to force Grbl to always set the machine origin at the homed location despite switch orientation.
// #define HOMING_FORCE_SET_ORIGIN // Uncomment to enable.

// Number of blocks Grbl executes upon startup. These blocks are stored in EEPROM, where the size
// and addresses are defined in settings.h. With the current settings, up to 2 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the g-code
// parser state depending on user preferences.

// После настройки Grbl по умолчанию установит для всего пространства станка отрицательное значение, как это обычно бывает
// для профессиональных станков с ЧПУ, независимо от того, где расположены концевые выключатели. Раскомментируйте это 
// определите, чтобы Grbl всегда устанавливал исходную точку компьютера в исходном местоположении, независимо от ориентации переключателя.
// #определите исходную ТОЧКУ_FORCE_SET_ORIGIN
// Раскомментируйте, чтобы включить.

// Количество блоков, которые Grbl выполняет при запуске. Эти блоки хранятся в EEPROM, где размер
// и адреса определяются в настройках.h. При текущих настройках может быть установлено до 2 блоков запуска.
// сохраняться и выполняться по порядку. Эти блоки запуска обычно используются для задания g-кода
// состояние синтаксического анализатора зависит от предпочтений пользователя.

#define N_STARTUP_LINE 2 // Integer (1-2)

// Number of floating decimal points printed by Grbl for certain value types. These settings are 
// determined by realistic and commonly observed values in CNC machines. For example, position
// values cannot be less than 0.001mm or 0.0001in, because machines can not be physically more
// precise this. So, there is likely no need to change these, but you can if you need to here.
// NOTE: Must be an integer value from 0 to ~4. More than 4 may exhibit round-off errors.

// Количество знаков после запятой с плавающей запятой, выводимых Grbl для определенных типов значений. Эти настройки 
// определяются реалистичными и часто наблюдаемыми значениями на станках с ЧПУ. Например, положение
// значения не могут быть меньше 0,001 мм или 0,0001 дюйма, поскольку машины физически не могут быть больше
// уточни это. Таким образом, скорее всего, нет необходимости менять их, но вы можете, если вам это нужно, здесь.
// ПРИМЕЧАНИЕ: Должно быть целое значение от 0 до ~4. При количестве более 4 могут возникать ошибки округления.

#define N_DECIMAL_COORDVALUE_INCH 4 // Coordinate or position value in inches // Значение координаты или положения в дюймах
#define N_DECIMAL_COORDVALUE_MM   3 // Coordinate or position value in mm // Значение координаты или положения в мм
#define N_DECIMAL_RATEVALUE_INCH  1 // Rate or velocity value in in/min // Скорость в дюймах/мин
#define N_DECIMAL_RATEVALUE_MM    0 // Rate or velocity value in mm/min // Скорость в мм/мин
#define N_DECIMAL_SETTINGVALUE    3 // Decimals for floating point setting values // Десятичные дроби для значений с плавающей запятой

// If your machine has two limits switches wired in parallel to one axis, you will need to enable
// this feature. Since the two switches are sharing a single pin, there is no way for Grbl to tell
// which one is enabled. This option only effects homing, where if a limit is engaged, Grbl will 
// alarm out and force the user to manually disengage the limit switch. Otherwise, if you have one
// limit switch for each axis, don't enable this option. By keeping it disabled, you can perform a
// homing cycle while on the limit switch and not have to move the machine off of it.

// Если на вашем компьютере установлены два концевых выключателя, подключенных параллельно одной оси, вам необходимо включить
// эту функцию. Поскольку оба переключателя имеют один общий контакт, Grbl не может определить, какой из них включен.
// Эта опция влияет только на самонаведение, где при включенном ограничении Grbl будет 
// отключите сигнал тревоги и вынудите пользователя вручную отключить концевой выключатель. В противном случае, если у вас есть один концевой выключатель
// для каждой оси, не включайте эту опцию. Отключив его, вы можете выполнить цикл
// самонаведения, находясь при включенном концевом выключателе, и вам не придется снимать машину с него.

// #define LIMITS_TWO_SWITCHES_ON_AXES

// Allows GRBL to track and report gcode line numbers.  Enabling this means that the planning buffer
// goes from 18 or 16 to make room for the additional line number data in the plan_block_t struct

// Позволяет GRBL отслеживать номера строк gcode и сообщать о них.  Включение этого параметра означает, что буфер планирования
// изменяется с 18 на 16, чтобы освободить место для дополнительных данных о номере строки в структуре plan_block_t
// #define USE_LINE_NUMBERS // Disabled by default. Uncomment to enable.

// Allows GRBL to report the real-time feed rate.  Enabling this means that GRBL will be reporting more 
// data with each status update.
// NOTE: This is experimental and doesn't quite work 100%. Maybe fixed or refactored later.
// Позволяет GRBL сообщать о скорости подачи в режиме реального времени.  Включение этого параметра означает, что GRBL будет сообщать больше данных 
// с каждым обновлением статуса.
// ПРИМЕЧАНИЕ: Это экспериментальный метод, который работает не на все 100%. Возможно, позже он будет исправлен или переработан.
// #define REPORT_REALTIME_RATE // Disabled by default. Uncomment to enable.

// Upon a successful probe cycle, this option provides immediately feedback of the probe coordinates
// through an automatically generated message. If disabled, users can still access the last probe
// coordinates through Grbl '$#' print parameters.

// После успешного выполнения цикла проверки этот параметр немедленно возвращает координаты проверки
// в автоматически сгенерированном сообщении. Если он отключен, пользователи по-прежнему могут получить доступ к последней проверке
// координаты через параметры печати Grbl '$#'.
#define MESSAGE_PROBE_COORDINATES // Enabled by default. Comment to disable.
 
// Enables a second coolant control pin via the mist coolant g-code command M7 on the Arduino Uno
// analog pin 4. Only use this option if you require a second coolant control pin.
// NOTE: The M8 flood coolant control pin on analog pin 3 will still be functional regardless.
// Активирует второй вывод управления охлаждающей жидкостью с помощью команды M7 с g-кодом mist coolant на Arduino Uno
// аналоговый вывод 4. Используйте эту опцию только в том случае, если вам требуется второй вывод управления охлаждающей жидкостью.
// ПРИМЕЧАНИЕ: Контрольный штифт подачи охлаждающей жидкости M8 на аналоговом штифте 3 все равно будет работать, несмотря ни на что.
// #define ENABLE_M7 // Disabled by default. Uncomment to enable.

// This option causes the feed hold input to act as a safety door switch. A safety door, when triggered,
// immediately forces a feed hold and then safely de-energizes the machine. Resuming is blocked until
// the safety door is re-engaged. When it is, Grbl will re-energize the machine and then resume on the
// previous tool path, as if nothing happened.

// При включении этой опции кнопка блокировки подачи действует как выключатель защитной дверцы. При срабатывании защитной дверцы
// немедленно происходит блокировка подачи, а затем безопасное обесточивание машины. Возобновление подачи блокируется до тех пор, пока
// защитная дверца не будет снова включена. Когда это произойдет, Grbl повторно включит станок и затем возобновит работу с
// предыдущей траекторией движения инструмента, как будто ничего не произошло.

// #define ENABLE_SAFETY_DOOR_INPUT_PIN // Default disabled. Uncomment to enable.

// После того, как переключатель защитной двери был переключен и восстановлен, этот параметр устанавливает задержку включения
// между восстановлением подачи охлаждающей жидкости на шпиндель и возобновлением цикла.
// ПРИМЕЧАНИЕ: Значение задержки определяется в миллисекундах от нуля до 65 535.
// After the safety door switch has been toggled and restored, this setting sets the power-up delay
// between restoring the spindle and coolant and resuming the cycle.
// NOTE: Delay value is defined in milliseconds from zero to 65,535.

#define SAFETY_DOOR_SPINDLE_DELAY 4000
#define SAFETY_DOOR_COOLANT_DELAY 1000

// Enable CoreXY kinematics. Use ONLY with CoreXY machines. 
// IMPORTANT: If homing is enabled, you must reconfigure the homing cycle #defines above to 
// #define HOMING_CYCLE_0 (1<<X_AXIS) and #define HOMING_CYCLE_1 (1<<Y_AXIS)
// NOTE: This configuration option alters the motion of the X and Y axes to principle of operation
// defined at (http://corexy.com/theory.html). Motors are assumed to positioned and wired exactly as
// described, if not, motions may move in strange directions. Grbl requires the CoreXY A and B motors
// have the same steps per mm internally.

// Включите кинематику CoreXY. Используйте ТОЛЬКО с машинами CoreXY. 
// ВАЖНО: если функция самонаведения включена, вы должны перенастроить цикл самонаведения, указанный выше, чтобы 
// #определите значение HOMING_CYCLE_0 (1<<ОСЬ X) и #определите значение HOMING_CYCLE_1 (1<<ОСЬ Y)
// ПРИМЕЧАНИЕ: Этот параметр конфигурации изменяет движение осей X и Y в соответствии с принципом работы
// определяется по адресу (http://corexy.com/theory.html). Предполагается, что двигатели расположены и подключены точно так, как
// описано, в противном случае движения могут происходить в разных направлениях. Для Grbl требуются двигатели CoreXY A и B
// с одинаковыми внутренними шагами на мм.

// #define COREXY // Default disabled. Uncomment to enable.

// Inverts pin logic of the control command pins. This essentially means when this option is enabled
// you can use normally-closed switches, rather than the default normally-open switches.
// NOTE: If you require individual control pins inverted, keep this macro disabled and simply alter
//   the CONTROL_INVERT_MASK definition in cpu_map.h files.

// Инвертирует логику выводов команд управления. По сути, это означает, что при включении этой опции
// вы можете использовать нормально замкнутые переключатели вместо нормально разомкнутых переключателей по умолчанию.
// ПРИМЕЧАНИЕ: Если вам требуется инвертировать отдельные управляющие контакты, отключите этот макрос и просто измените его.
// определение CONTROL_INVERT_MASK в файлах cpu_map.h.

// #define INVERT_ALL_CONTROL_PINS // Default disabled. Uncomment to enable.

// Inverts select limit pin states based on the following mask. This effects all limit pin functions, 
// such as hard limits and homing. However, this is different from overall invert limits setting. 
// This build option will invert only the limit pins defined here, and then the invert limits setting
// will be applied to all of them. This is useful when a user has a mixed set of limit pins with both
// normally-open(NO) and normally-closed(NC) switches installed on their machine.
// NOTE: PLEASE DO NOT USE THIS, unless you have a situation that needs it.

// Инвертирует выбор предельных значений pin-кода в соответствии со следующей маской. Это влияет на все функции предельного pin-кода,
// такие как жесткие ограничения и самонаведение. Однако это отличается от общей настройки пределов инвертирования. 
// Этот параметр построения инвертирует только указанные здесь предельные выводы, а затем параметр инвертировать ограничения
// будет применен ко всем из них. Это полезно, когда у пользователя смешанный набор предельных выводов с обоими параметрами.
// на их станке установлены нормально разомкнутые (NO) и нормально замкнутые (NC) переключатели.
// ПРИМЕЧАНИЕ: ПОЖАЛУЙСТА, НЕ ИСПОЛЬЗУЙТЕ ИХ, если только это не требуется в конкретной ситуации.

// #define INVERT_LIMIT_PIN_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)) // Default disabled. Uncomment to enable.

// Inverts the spindle enable pin from low-disabled/high-enabled to low-enabled/high-disabled. Useful
// for some pre-built electronic boards.
// NOTE: If VARIABLE_SPINDLE is enabled(default), this option has no effect as the PWM output and 
// spindle enable are combined to one pin. If you need both this option and spindle speed PWM, 
// uncomment the config option USE_SPINDLE_DIR_AS_ENABLE_PIN below.

// Изменяет положение штифта включения шпинделя с низкого уровня отключения/высокого уровня включения на низкий уровень включения/высокий уровень отключения. Полезный
// для некоторых готовых электронных плат.
// ПРИМЕЧАНИЕ: Если параметр VARIABLE_SPINDLE включен (по умолчанию), этот параметр не действует, поскольку ШИМ-выход и 
// spindle enable объединены в один вывод. Если вам нужен как этот параметр, так и ШИМ скорости вращения шпинделя,
// раскомментируйте параметр конфигурации USE_SPINDLE_DIR_AS_ENABLE_PIN ниже.

// #define INVERT_SPINDLE_ENABLE_PIN // Default disabled. Uncomment to enable.

// Enable control pin states feedback in status reports. The data is presented as simple binary of
// the control pin port (0 (low) or 1(high)), masked to show only the input pins. Non-control pins on the 
// port will always show a 0 value. See cpu_map.h for the pin bitmap. As with the limit pin reporting,
// we do not recommend keeping this option enabled. Try to only use this for setting up a new CNC.
// Включить обратную связь о состоянии управляющего контакта в отчетах о состоянии. Данные представлены в виде простого двоичного кода
// порта управляющего контакта (0 (низкий) или 1 (высокий)), замаскированного для отображения только входных контактов. На неконтролируемых выводах порта
// всегда будет отображаться значение 0. Битовую карту выводов смотрите в cpu_map.h. Как и в случае с отчетами о предельных выводах,
// мы не рекомендуем включать эту опцию. Старайтесь использовать это только для настройки нового ЧПУ.

// #define REPORT_CONTROL_PIN_STATE // Default disabled. Uncomment to enable.

// When Grbl powers-cycles or is hard reset with the Arduino reset button, Grbl boots up with no ALARM
// by default. This is to make it as simple as possible for new users to start using Grbl. When homing
// is enabled and a user has installed limit switches, Grbl will boot up in an ALARM state to indicate 
// Grbl doesn't know its position and to force the user to home before proceeding. This option forces
// Grbl to always initialize into an ALARM state regardless of homing or not. This option is more for
// OEMs and LinuxCNC users that would like this power-cycle behavior.

// При повторном включении Grbl или его жестком сбросе с помощью кнопки сброса Arduino Grbl загружается без предупреждения
// по умолчанию. Это делается для того, чтобы новым пользователям было как можно проще начать использовать Grbl. При наведении
// если / включен и пользователь установил концевые выключатели, Grbl загрузится в аварийном состоянии, указывая на то, что 
// Grbl не знает своего местоположения, и заставит пользователя вернуться домой, прежде чем продолжить. Эта опция заставляет
// Grbl всегда будет инициализироваться в аварийном состоянии независимо от того, сработал самонаведение или нет. Эта опция предназначена скорее для
// OEM-производителей и пользователей LinuxCNC, которым бы понравилось такое поведение при включении питания.

// #define FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Включает минимальный режим обратной связи с отчетами для графических интерфейсов, где строки, понятные пользователю, не так важны.
// Это экономит почти 2 КБАЙТ флэш-памяти и может обеспечить достаточно места для установки других / будущих функций.
// Графическим интерфейсам потребуется установить вместо них таблицу поиска кодов ошибок, которые Grbl отправляет обратно.
// ПРИМЕЧАНИЕ: Эта функция является новой и экспериментальной. Убедитесь, что используемый вами графический интерфейс поддерживает этот режим.
// #define REPORT_GUI_MODE // По умолчанию отключен. Раскомментируйте, чтобы включить.

// Временное разрешение подсистемы управления ускорением. Чем больше число, тем более плавным будет процесс.
// ускорение, особенно заметное на машинах, работающих с очень высокой скоростью подачи, но может отрицательно сказаться
// влияет на производительность. Правильное значение этого параметра зависит от машины, поэтому рекомендуется
// устанавливать его только на необходимое значение. Приблизительные значения успешности могут варьироваться от 50 до 200 и более.
// ПРИМЕЧАНИЕ: Изменение этого значения также приводит к изменению времени выполнения сегмента в буфере сегмента step. 
// При увеличении этого значения в буфере сегмента сохраняется меньшее общее время, и наоборот. Сделай
// убедитесь, что буфер сегмента шага увеличен/уменьшен для учета этих изменений.

// Enables minimal reporting feedback mode for GUIs, where human-readable strings are not as important.
// This saves nearly 2KB of flash space and may allow enough space to install other/future features.
// GUIs will need to install a look-up table for the error-codes that Grbl sends back in their place.
// NOTE: This feature is new and experimental. Make sure the GUI you are using supports this mode.
// #define REPORT_GUI_MODE // Default disabled. Uncomment to enable.

// The temporal resolution of the acceleration management subsystem. A higher number gives smoother
// acceleration, particularly noticeable on machines that run at very high feedrates, but may negatively
// impact performance. The correct value for this parameter is machine dependent, so it's advised to
// set this only as high as needed. Approximate successful values can widely range from 50 to 200 or more.
// NOTE: Changing this value also changes the execution time of a segment in the step segment buffer. 
// When increasing this value, this stores less overall time in the segment buffer and vice versa. Make
// certain the step segment buffer is increased/decreased to account for these changes.


#define ACCELERATION_TICKS_PER_SECOND 100 

// Adaptive Multi-Axis Step Smoothing (AMASS) is an advanced feature that does what its name implies, 
// smoothing the stepping of multi-axis motions. This feature smooths motion particularly at low step
// frequencies below 10kHz, where the aliasing between axes of multi-axis motions can cause audible 
// noise and shake your machine. At even lower step frequencies, AMASS adapts and provides even better
// step smoothing. See stepper.c for more details on the AMASS system works.

// Адаптивное многоосевое сглаживание шага (AMASS) - это расширенная функция, которая выполняет то, что следует из ее названия,
//- сглаживает шаг многоосевых движений. Эта функция сглаживает движение, особенно при низком шаге
// частоты ниже 10 кГц, на которых смещение между осями многоосевых перемещений может привести к слышимым 
// шумите и встряхивайте машину. При еще более низких частотах шага AMASS адаптируется и обеспечивает еще лучшее
// пошаговое сглаживание. Смотрите stepper.c для получения более подробной информации о работе системы AMASS.

#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Default enabled. Comment to disable.

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error 
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.

// Устанавливает максимальную скорость шага, которую можно записать в качестве параметра Grbl. Этот параметр приводит к возникновению ошибки 
// проверьте в модуле настроек, чтобы значения настроек не превышали это ограничение. Максимальное значение
// скорость шага строго ограничена скоростью процессора и изменится, если будет запущено что-то отличное от AVR
// используется частота 16 МГц.
// ПРИМЕЧАНИЕ: Пока отключено, будет включено, если позволит место на флэш-памяти.

// #define MAX_STEP_RATE_HZ 30000 // Hz

// By default, Grbl sets all input pins to normal-high operation with their internal pull-up resistors
// enabled. This simplifies the wiring for users by requiring only a switch connected to ground, 
// although its recommended that users take the extra step of wiring in low-pass filter to reduce
// electrical noise detected by the pin. If the user inverts the pin in Grbl settings, this just flips
// which high or low reading indicates an active signal. In normal operation, this means the user 
// needs to connect a normal-open switch, but if inverted, this means the user should connect a 
// normal-closed switch. 
// The following options disable the internal pull-up resistors, sets the pins to a normal-low 
// operation, and switches must be now connect to Vcc instead of ground. This also flips the meaning 
// of the invert pin Grbl setting, where an inverted setting now means the user should connect a 
// normal-open switch and vice versa.
// NOTE: All pins associated with the feature are disabled, i.e. XYZ limit pins, not individual axes.
// WARNING: When the pull-ups are disabled, this requires additional wiring with pull-down resistors!

// По умолчанию Grbl переводит все входные контакты в режим нормальной работы на высоком уровне с включенными внутренними подтягивающими резисторами
//. Это упрощает подключение для пользователей, поскольку требуется только заземление выключателя, 
// хотя пользователям рекомендуется выполнить дополнительный этап подключения фильтра нижних частот, чтобы уменьшить
// электрический шум, обнаруженный выводом. Если пользователь инвертирует вывод в настройках Grbl, он просто переключается
// высокое или низкое значение которого указывает на активный сигнал. При нормальной работе это означает, что пользователь 
// необходимо подключить нормально разомкнутый переключатель, но если он перевернут, это означает, что пользователь должен подключить 
// переключатель нормально замкнут. 
// Следующие опции отключают внутренние подтягивающие резисторы, устанавливая контакты в положение нормального низкого напряжения 
// теперь переключатели должны быть подключены к Vcc, а не к земле. Это также меняет смысл 
// из настройки инвертировать вывод Grbl, где инвертированная настройка теперь означает, что пользователь должен подключить переключатель
// нормального разомкнутого состояния и наоборот.
// ПРИМЕЧАНИЕ: Все контакты, связанные с этой функцией, отключены, т.е. контакты ограничения XYZ, а не отдельные оси.
// ВНИМАНИЕ: Если подтягивающие устройства отключены, для этого требуется дополнительная проводка с подтягивающими резисторами!

//#define DISABLE_LIMIT_PIN_PULL_UP
//#define DISABLE_PROBE_PIN_PULL_UP
//#define DISABLE_CONTROL_PIN_PULL_UP

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with 
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.

// Устанавливает, к какой оси применяется смещение длины инструмента. Предполагается, что шпиндель всегда параллелен 
// выбранная ось с инструментом, ориентированным в отрицательном направлении. Другими словами, позитивный
// значение смещения длины инструмента вычитается из текущего местоположения.

#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.

// Enables variable spindle output voltage for different RPM values. On the Arduino Uno, the spindle
// enable pin will output 5V for maximum RPM with 256 intermediate levels and 0V when disabled.
// NOTE: IMPORTANT for Arduino Unos! When enabled, the Z-limit pin D11 and spindle enable pin D12 switch!
// The hardware PWM output on pin D11 is required for variable spindle output voltages.

// Позволяет изменять выходное напряжение шпинделя при различных значениях оборотов в минуту. На Arduino Uno шпиндель
// на выводе enable выдает напряжение 5 В при максимальных оборотах в минуту с 256 промежуточными уровнями и 0 В при отключении.
// ПРИМЕЧАНИЕ: ВАЖНО для Arduino Unos! При включении переключатель Z-limit на выводе D11 и шпинделя включает вывод D12!
// Для изменения выходных напряжений шпинделя требуется аппаратный ШИМ-выход на выводе D11.

#define VARIABLE_SPINDLE // Default enabled. Comment to disable.

// Used by the variable spindle output only. These parameters set the maximum and minimum spindle speed
// "S" g-code values to correspond to the maximum and minimum pin voltages. There are 256 discrete and 
// equally divided voltage bins between the maximum and minimum spindle speeds. So for a 5V pin, 1000
// max rpm, and 250 min rpm, the spindle output voltage would be set for the following "S" commands: 
// "S1000" @ 5V, "S250" @ 0.02V, and "S625" @ 2.5V (mid-range). The pin outputs 0V when disabled.

// Используется только для переменной частоты вращения шпинделя. Эти параметры задают максимальную и минимальную частоту вращения шпинделя
// Значения g-кода "S" соответствуют максимальному и минимальному напряжениям на выводах. Имеется 256 дискретных и 
// равномерно распределенных диапазонов напряжения между максимальной и минимальной частотой вращения шпинделя. Таким образом, для вывода 5 В, 1000
// макс. оборотов в минуту и 250 мин. об/мин выходное напряжение шпинделя будет задано для следующих команд "S": 
// "S1000" при 5 В, "S250" при 0,02 В и "S625" при 2,5 В (средний диапазон). Когда вывод отключен, он выдает 0 В.

#define SPINDLE_MAX_RPM 1000.0 // Max spindle RPM. This value is equal to 100% duty cycle on the PWM. // Максимальное число оборотов шпинделя. Это значение равно 100%-ному рабочему циклу ШИМ.
#define SPINDLE_MIN_RPM 0.0    // Min spindle RPM. This value is equal to (1/256) duty cycle on the PWM. // Мин оборотов шпинделя. Это значение равно (1/256) рабочему циклу ШИМ.

// Used by variable spindle output only. This forces the PWM output to a minimum duty cycle when enabled.
// When disabled, the PWM pin will still read 0V. Most users will not need this option, but it may be 
// useful in certain scenarios. This setting does not update the minimum spindle RPM calculations. Any
// spindle RPM output lower than this value will be set to this value.

// Используется только при переменной мощности шпинделя. Это приводит к минимальному рабочему циклу ШИМ-выхода при его включении.
// При отключении ШИМ-вывод по-прежнему будет показывать значение 0V. Большинству пользователей эта опция не понадобится, но она может быть 
// полезно в определенных сценариях. Эта настройка не изменяет расчетные значения минимальных оборотов шпинделя. Какой-нибудь
// для этого значения будет установлена частота вращения шпинделя, меньшая, чем это значение.

// #define MINIMUM_SPINDLE_PWM 5 // Default disabled. Uncomment to enable. Integer (0-255)

// By default on a 328p(Uno), Grbl combines the variable spindle PWM and the enable into one pin to help 
// preserve I/O pins. For certain setups, these may need to be separate pins. This configure option uses
// the spindle direction pin(D13) as a separate spindle enable pin along with spindle speed PWM on pin D11. 
// NOTE: This configure option only works with VARIABLE_SPINDLE enabled and a 328p processor (Uno). 
// NOTE: With no direction pin, the spindle clockwise M4 g-code command will be removed. M3 and M5 still work.
// NOTE: BEWARE! The Arduino bootloader toggles the D13 pin when it powers up. If you flash Grbl with
// a programmer (you can use a spare Arduino as "Arduino as ISP". Search the web on how to wire this.), 
// this D13 LED toggling should go away. We haven't tested this though. Please report how it goes!

// По умолчанию на 328p (Uno) Grbl объединяет ШИМ с регулируемым шпинделем и функцию включения в одном выводе, чтобы помочь 
// сохраните контакты ввода-вывода. Для определенных настроек может потребоваться, чтобы это были отдельные контакты. Этот параметр настройки использует
// вывод направления вращения шпинделя (D13) является отдельным выводом управления шпинделем, а также ШИМ-регулятором скорости вращения шпинделя на выводе D11. 
// ПРИМЕЧАНИЕ: Этот параметр настройки работает только с включенной функцией VARIABLE_SPINDLE и процессором 328p (Uno). 
// ПРИМЕЧАНИЕ: При отсутствии направляющего штифта команда g-кода вращения шпинделя по часовой стрелке M4 будет удалена. M3 и M5 по-прежнему работают.
// ПРИМЕЧАНИЕ: БУДЬТЕ ОСТОРОЖНЫ! Загрузчик Arduino переключает вывод D13 при включении питания. Если вы перепрошиваете Grbl с помощью
// программатора (вы можете использовать запасной Arduino в качестве "Arduino as ISP". Поищите в Интернете, как это подключить.),
// это переключение светодиодов D13 должно прекратиться. Однако мы это не тестировали. Пожалуйста, сообщите, как это работает!

// #define USE_SPINDLE_DIR_AS_ENABLE_PIN // Default disabled. Uncomment to enable.

// With this enabled, Grbl sends back an echo of the line it has received, which has been pre-parsed (spaces
// removed, capitalized letters, no comments) and is to be immediately executed by Grbl. Echoes will not be 
// sent upon a line buffer overflow, but should for all normal lines sent to Grbl. For example, if a user 
// sendss the line 'g1 x1.032 y2.45 (test comment)', Grbl will echo back in the form '[echo: G1X1.032Y2.45]'.
// NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
// performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
// to help minimize transmission waiting within the serial write protocol.

// Если эта функция включена, Grbl отправляет обратно эхо-сообщение о полученной строке, которое было предварительно обработано (пробелы
// удалены, заглавные буквы, без комментариев) и должно быть немедленно выполнено Grbl. Эхо-сообщения не будут отображаться. 
// отправляется при переполнении буфера строк, но должен выполняться для всех обычных строк, отправляемых в Grbl. Например, если пользователь 
// отправляет строку 'g1x1.032y2.45 (тестовый комментарий)', Grbl отправит ответное сообщение в виде '[echo: G1X1.032Y2.45]'.
// ПРИМЕЧАНИЕ: Используйте это только для целей отладки!! При повторном использовании это отнимает ценные ресурсы и может привести к
// производительность. Если это абсолютно необходимо для нормальной работы, буфер последовательной записи должен быть значительно увеличен
// чтобы свести к минимуму ожидание передачи в рамках протокола последовательной записи.

// #define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
// every buffer block junction, except for starting from rest and end of the buffer, which are always
// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
// limits or angle between neighboring block line move directions. This is useful for machines that can't
// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
// should not be much greater than zero or to the minimum value necessary for the machine to work.

// Минимальная планируемая скорость соединения. Задает минимальную скорость соединения по умолчанию, которую планировщик планирует устанавливать на
// каждом стыке буферного блока, за исключением начала с конца буфера, которые всегда равны
// нулю. Это значение определяет скорость перемещения машины через стыки без учета ускорения
// пределы или угол между направлениями перемещения линий соседних блоков. Это полезно для станков, которые не могут
// удерживать инструмент в течение доли секунды, например, для 3d-принтеров или лазерных резцов. Если используется, это значение
// не должно быть намного больше нуля или минимального значения, необходимого для работы машины.

#define MINIMUM_JUNCTION_SPEED 0.0 // (mm/min)

// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
// value. This also ensures that a planned motion always completes and accounts for any floating-point
// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.

// Устанавливает минимальную скорость подачи, разрешенную планировщиком. Любое значение, меньшее этого значения, будет установлено на это минимальное значение
//. Это также гарантирует, что запланированное перемещение всегда завершается и учитывает любые ошибки округления с плавающей запятой
//. Несмотря на то, что это не рекомендуется, для станков меньшего размера, скорее всего, подойдет значение меньше 1,0 мм/мин
//, возможно, до 0,1 мм/мин, но ваш успех может зависеть от множества факторов.

#define MINIMUM_FEED_RATE 1.0 // (mm/min)

// Number of arc generation iterations by small angle approximation before exact arc trajectory 
// correction with expensive sin() and cos() calcualtions. This parameter maybe decreased if there 
// are issues with the accuracy of the arc generations, or increased if arc execution is getting
// bogged down by too many trig calculations.

// Количество итераций генерации дуги путем аппроксимации малым углом до получения точной траектории дуги 
// коррекция с помощью дорогостоящих вычислений sin() и cos(). Этот параметр может быть уменьшен, если есть 
// проблемы с точностью генерации дуги, или увеличен, если выполнение дуги становится все более сложным.
// запутался в слишком большом количестве тригонометрических вычислений.

#define N_ARC_CORRECTION 12 // Integer (1-255)

// The arc G2/3 g-code standard is problematic by definition. Radius-based arcs have horrible numerical 
// errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate 
// but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating 
// point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
// small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
// This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
// NOTE: Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
// much greater than this. The default setting should capture most, if not all, full arc error situations.

// Стандарт g-кода arc G2/3 проблематичен по определению. Дуги, основанные на радиусе, имеют ужасные числовые характеристики 
// ошибки при построении дуг в виде полукругов (pi) или полных окружностей (2*pi). Дуги, основанные на смещении, намного точнее 
// но все равно возникает проблема, когда дуги представляют собой полные окружности (2*pi). Это определяет значения с плавающей точкой 
// проблемы с точками возникают, когда дуги, основанные на смещении, задаются как полные окружности, но интерпретируются как чрезвычайно сложные.
// небольшие дуги с огибающей станка epsilon (1,2e-7rad) из-за проблем с числовым округлением и точностью.
// Это значение определяет величину эпсилон-среза станка, чтобы определить, является ли дуга полной окружностью или нет.
// ПРИМЕЧАНИЕ: Будьте очень осторожны при настройке этого значения. Оно всегда должно быть больше 1,2e-7, но не слишком
// намного больше этого значения. Настройка по умолчанию должна отражать большинство, если не все, ситуаций с ошибками полной дуги.

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

// Time delay increments performed during a dwell. The default value is set at 50ms, which provides
// a maximum time delay of roughly 55 minutes, more than enough for most any application. Increasing
// this delay will increase the maximum dwell time linearly, but also reduces the responsiveness of 
// run-time command executions, like status reports, since these are performed between each dwell 
// time step. Also, keep in mind that the Arduino delay timer is not very accurate for long delays.

// Время задержки увеличивается во время ожидания. Значение по умолчанию равно 50 мс, что обеспечивает
// максимальную задержку примерно в 55 минут, что более чем достаточно для большинства приложений. Возрастающий
// эта задержка линейно увеличивает максимальное время ожидания, но также снижает скорость отклика 
// выполнение команд во время выполнения, например, отчеты о состоянии, поскольку они выполняются между каждой остановкой 
// временной шаг. Кроме того, имейте в виду, что таймер задержки Arduino не очень точен при длительных задержках.

#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

// Creates a delay between the direction pin setting and corresponding step pulse by creating
// another interrupt (Timer2 compare) to manage it. The main Grbl interrupt (Timer1 compare) 
// sets the direction pins, and does not immediately set the stepper pins, as it would in 
// normal operation. The Timer2 compare fires next to set the stepper pins after the step 
// pulse delay time, and Timer2 overflow will complete the step pulse, except now delayed 
// by the step pulse time plus the step pulse delay. (Thanks langwadt for the idea!)
// NOTE: Uncomment to enable. The recommended delay must be > 3us, and, when added with the
// user-supplied step pulse time, the total time must not exceed 127us. Reported successful
// values for certain setups have ranged from 5 to 20us.

// Создает задержку между установкой направляющего контакта и соответствующим импульсом шага, создавая
// другое прерывание (таймер сравнения 2) для управления им. Основное прерывание Grbl (таймер сравнения 1) 
// устанавливает направляющие контакты, а не сразу устанавливает шаговые контакты, как это было бы в 
// нормальная работа. Индикатор Timer2 compare срабатывает для установки выводов шагового переключателя после шага 
// время задержки импульса, и Timer2 overflow завершит выполнение шагового импульса, только теперь с задержкой 
// на время выполнения шагового импульса плюс задержка шагового импульса. (Спасибо Лангвадту за идею!)
// ПРИМЕЧАНИЕ: Раскомментируйте, чтобы включить. Рекомендуемая задержка должна быть > 3 секунд, а при добавлении
// заданного пользователем времени пошагового импульса общее время не должно превышать 127 секунд. Сообщается об успешном завершении
// значения для определенных настроек варьировались от 5 до 20 единиц.

// #define STEP_PULSE_DELAY 10 // Step pulse delay in microseconds. Default disabled.

// The number of linear motions in the planner buffer to be planned at any give time. The vast
// majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra 
// available RAM, like when re-compiling for a Mega or Sanguino. Or decrease if the Arduino
// begins to crash due to the lack of available RAM or if the CPU is having trouble keeping
// up with planning new incoming motions as they are executed.

// Количество линейных перемещений в буфере планировщика, которые необходимо запланировать в любой момент времени. Подавляющее
// Большинство оперативной памяти, используемой Grbl, основано на размере этого буфера. Увеличивайте только при наличии дополнительной
// доступной оперативной памяти, например, при повторной компиляции для Mega или Sanguino. Или уменьшайте, если Arduino
// начинает выходить из строя из-за нехватки доступной оперативной памяти или из-за проблем с процессором.
// приступайте к планированию новых поступающих заявок по мере их выполнения.

// #define BLOCK_BUFFER_SIZE 18  // Uncomment to override default in planner.h.

// Governs the size of the intermediary step segment buffer between the step execution algorithm
// and the planner blocks. Each segment is set of steps executed at a constant velocity over a
// fixed time defined by ACCELERATION_TICKS_PER_SECOND. They are computed such that the planner
// block velocity profile is traced exactly. The size of this buffer governs how much step 
// execution lead time there is for other Grbl processes have to compute and do their thing 
// before having to come back and refill this buffer, currently at ~50msec of step moves.

// Определяет размер буфера промежуточных сегментов шага между алгоритмом выполнения шага
// и блоками планирования. Каждый сегмент представляет собой набор шагов, выполняемых с постоянной скоростью в течение
// фиксированного времени, определяемого параметром ACCELERATION_TICKS_PER_SECOND. Они рассчитываются таким образом, чтобы планировщик
// профиль скорости блока прослеживается точно. Размер этого буфера определяет, насколько велик шаг 
// время выполнения есть для того, чтобы другие процессы Grbl могли выполнить вычисления и выполнить свою работу 
// прежде чем вернуться и пополнить этот буфер, в настоящее время выполняется шаговое перемещение ~50 мс.

// #define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

// Line buffer size from the serial input stream to be executed. Also, governs the size of 
// each of the startup blocks, as they are each stored as a string of this size. Make sure
// to account for the available EEPROM at the defined memory address in settings.h and for
// the number of desired startup blocks.
// NOTE: 80 characters is not a problem except for extreme cases, but the line buffer size 
// can be too small and g-code blocks can get truncated. Officially, the g-code standards 
// support up to 256 characters. In future versions, this default will be increased, when 
// we know how much extra memory space we can re-invest into this.

// Размер буфера строк из потока последовательного ввода, который должен быть выполнен. Также определяет размер 
// каждого из блоков запуска, поскольку каждый из них хранится в виде строки такого размера. Убедиться
// для учета доступной EEPROM-памяти по указанному адресу памяти в настройках.h и для
// количества требуемых блоков запуска.
// ПРИМЕЧАНИЕ: 80 символов - это не проблема, за исключением крайних случаев, но размер строчного буфера 
// может быть слишком маленьким, и блоки g-кода могут быть усечены. Официально стандарты g-кода 
// поддерживают до 256 символов. В будущих версиях это значение по умолчанию будет увеличено, когда 
// мы знаем, сколько дополнительного места в памяти мы можем вложить в это повторно.

// #define LINE_BUFFER_SIZE 80  // Uncomment to override default in protocol.h
  
// Serial send and receive buffer size. The receive buffer is often used as another streaming
// buffer to store incoming blocks to be processed by Grbl when its ready. Most streaming
// interfaces will character count and track each block send to each block response. So, 
// increase the receive buffer if a deeper receive buffer is needed for streaming and avaiable
// memory allows. The send buffer primarily handles messages in Grbl. Only increase if large
// messages are sent and Grbl begins to stall, waiting to send the rest of the message.
// NOTE: Buffer size values must be greater than zero and less than 256.

// Размер буфера последовательной отправки и приема. Буфер приема часто используется в качестве другого буфера потоковой передачи данных
// для хранения входящих блоков, которые будут обработаны Grbl, когда они будут готовы. Большинство интерфейсов потоковой передачи данных
// будут подсчитывать символы и отслеживать каждый отправленный блок в ответ на каждый блок. Таким образом,
// увеличьте буфер приема, если для потоковой передачи требуется более глубокий буфер приема и он доступен
// позволяет память. Буфер отправки в основном обрабатывает сообщения в формате Grbl. Увеличивайте только при больших размерах
// сообщения отправляются, и Grbl начинает зависать в ожидании отправки остальной части сообщения.
// ПРИМЕЧАНИЕ: Значения размера буфера должны быть больше нуля и меньше 256.

// #define RX_BUFFER_SIZE 128 // Uncomment to override defaults in serial.h
// #define TX_BUFFER_SIZE 64
  

// A simple software debouncing feature for hard limit switches. When enabled, the interrupt 
// monitoring the hard limit switch pins will enable the Arduino's watchdog timer to re-check 
// the limit pin state after a delay of about 32msec. This can help with CNC machines with 
// problematic false triggering of their hard limit switches, but it WILL NOT fix issues with 
// electrical interference on the signal cables from external sources. It's recommended to first
// use shielded signal cables with their shielding connected to ground (old USB/computer cables 
// work well and are cheap to find) and wire in a low-pass circuit into each limit pin.

// Простая функция отключения программного обеспечения для жестких концевых выключателей. Когда этот параметр включен, прерывание 
// контроль контактов жесткого концевого выключателя позволит сторожевому таймеру Arduino повторно проверить 
// состояние предельного вывода после задержки примерно в 32 секунды. Это может помочь в работе со станками с ЧПУ, у которых
// возникают проблемы с ложным срабатыванием жестких концевых выключателей, но не устранит проблемы с 
// электрическими помехами на сигнальных кабелях от внешних источников. Рекомендуется сначала
// используйте экранированные сигнальные кабели с заземлением (старые USB/компьютерные кабели 
// работают хорошо и стоят недорого) и подключите низкочастотную цепь к каждому ограничивающему контакту.

// #define ENABLE_SOFTWARE_DEBOUNCE // Default disabled. Uncomment to enable.

// Force Grbl to check the state of the hard limit switches when the processor detects a pin
// change inside the hard limit ISR routine. By default, Grbl will trigger the hard limits
// alarm upon any pin change, since bouncing switches can cause a state check like this to 
// misread the pin. When hard limits are triggered, they should be 100% reliable, which is the
// reason that this option is disabled by default. Only if your system/electronics can guarantee
// that the switches don't bounce, we recommend enabling this option. This will help prevent
// triggering a hard limit when the machine disengages from the switch.
// NOTE: This option has no effect if SOFTWARE_DEBOUNCE is enabled.

// Принудительный запуск Grbl для проверки состояния жестких ограничительных переключателей при обнаружении процессором pin-кода
// изменение в процедуре ISR с жестким ограничением. По умолчанию Grbl запускает жесткие ограничения
// сигнал тревоги при любом изменении pin-кода, поскольку переключатели с отскакивающими переключателями могут привести к такой проверке состояния, как эта. 
// неправильно ввел пин-код. Когда срабатывают жесткие ограничения, они должны быть надежными на 100%, что является
// причиной того, что эта опция по умолчанию отключена. Только в том случае, если ваша система/электроника могут гарантировать
// чтобы переключатели не отскакивали, мы рекомендуем включить эту опцию. Это поможет предотвратить
// запуск жесткого ограничения при отключении устройства от коммутатора.
// ПРИМЕЧАНИЕ: Эта опция не действует, если включена функция SOFTWARE_DEBOUNCE.

// #define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.


// ---------------------------------------------------------------------------------------
// COMPILE-TIME ERROR CHECKING OF DEFINE VALUES:

#ifndef HOMING_CYCLE_0
  #error "Required HOMING_CYCLE_0 not defined."
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(VARIABLE_SPINDLE)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with VARIABLE_SPINDLE enabled"
#endif

// #if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(CPU_MAP_ATMEGA328P)
//   #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with a 328p processor"
// #endif

// ---------------------------------------------------------------------------------------


#endif
