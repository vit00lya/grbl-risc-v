#pragma once
/*
  settings.h - eeprom configuration handling 
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

#ifndef settings_h
#define settings_h

#include "grbl.h"


// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
// Версия данных EEPROM. Будет использоваться для переноса существующих данных из более старых версий Grbl
// при обновлении встроенного ПО. Всегда сохраняется в 0-м байте eeprom
#define SETTINGS_VERSION 9  // NOTE: Check settings_reset() when moving to next version. // ПРИМЕЧАНИЕ: Проверьте settings_reset() при переходе к следующей версии.

// Define bit flag masks for the boolean settings in settings.flag. // Определите маски битовых флагов для логических настроек в settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_AUTO_START         bit(1) // Obsolete. Don't alter to keep back compatibility. // Устарел. Не изменяйте, чтобы сохранить совместимость.
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)
#define BITFLAG_INVERT_PROBE_PIN   bit(7)

// Define status reporting boolean enable bit flags in settings.status_report_mask // Определите логическое значение отчета о состоянии, включив битовые флаги в настройках.status_report_mask
#define BITFLAG_RT_STATUS_MACHINE_POSITION  bit(0)
#define BITFLAG_RT_STATUS_WORK_POSITION     bit(1)
#define BITFLAG_RT_STATUS_PLANNER_BUFFER    bit(2)
#define BITFLAG_RT_STATUS_SERIAL_RX         bit(3)
#define BITFLAG_RT_STATUS_LIMIT_PINS        bit(4)

// Define settings restore bitflags. // Определить настройки восстановления битовых флагов
#define SETTINGS_RESTORE_ALL 0xFF // All bitflags // Все битовые флаги
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future 
// developments.
// Определите значения адресов памяти EEPROM для настроек и параметров Grbl
// ПРИМЕЧАНИЕ: В Atmega328p имеется 1 КБАЙТ памяти EEPROM. Верхняя половина зарезервирована для параметров и
// сценария запуска. Нижняя половина содержит глобальные настройки и пространство для будущих настроек. 
// события.
// #define EEPROM_ADDR_GLOBAL         1U
// #define EEPROM_ADDR_PARAMETERS     512U
// #define EEPROM_ADDR_STARTUP_BLOCK  768U
// #define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
// Определите индексацию адреса EEPROM для параметров координат
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1) // Количество поддерживаемых рабочих систем координат (начиная с индекса 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0) // Общее количество сохраненных системных данных (начиная с индекса 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59) // ПРИМЕЧАНИЕ: Индексы рабочих координат (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported) // Смещение координат (G92.2,G92.3 не поддерживаются)

// Определите схему нумерации настроек оси Grbl. Начинается с START_VAL, с каждым ШАГОМ, через N_SETTINGS.
// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255. // ПРИМЕЧАНИЕ: Сохраните значения настроек >= 100 для настроек оси. До 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings // Должно быть больше, чем количество настроек оси

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards) // Глобальные постоянные настройки (сохраняются начиная с байта EEPROM_ADDR_GLOBAL и далее)
struct settings_t
{
  // Axis settings // Настройки оси
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];

  // Remaining Grbl settings // Остальные настройки Grbl
  u8 pulse_microseconds;
  u8 step_invert_mask;
  u8 dir_invert_mask;
  u8 stepper_idle_lock_time; // If max value 255, steppers do not disable. // При максимальном значении 255 степперы не отключаются.
  u8 status_report_mask; // Mask to indicate desired report data. // Маска для указания желаемых данных отчета.
  float junction_deviation;
  float arc_tolerance;
  
  u8 flags;  // Contains default boolean settings // Содержит логические настройки по умолчанию

  u8 homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  u16 homing_debounce_delay;
  float homing_pulloff;
};

void SettingsInit(settings_t& settings);
bool ReadGlobalSettings();
void SettingsRestore(u8 restore_flag, settings_t& settings);

// Initialize the configuration subsystem (load settings from EEPROM) // Инициализировать конфигурационную подсистему (загрузить настройки из EEPROM)
void settings_init();


// A helper method to set new settings from command line // Вспомогательный метод для установки новых настроек из командной строки
uint8_t settings_store_global_setting(uint8_t parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM // Сохраняет переменную строки протокола в качестве начальной строки в EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable // Считывает строку запуска EEPROM в переменную строки протокола
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Stores build info user-defined string // Хранит информацию о сборке в пользовательской строке
void settings_store_build_info(char *line);

// Reads build info user-defined string // Считывает пользовательскую строку информации о сборке
uint8_t settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM // Записывает выбранные данные о координатах в EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM // Считывает выбранные координатные данные из EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

// Returns the step pin mask according to Grbl's internal axis numbering // Возвращает маску ступенчатого штифта в соответствии с нумерацией внутренних осей Grbl
uint8_t get_step_pin_mask(uint8_t i);

// Returns the direction pin mask according to Grbl's internal axis numbering // Возвращает маску направляющего штифта в соответствии с нумерацией внутренних осей Grbl
uint8_t get_direction_pin_mask(uint8_t i);

// Returns the limit pin mask according to Grbl's internal axis numbering // Возвращает маску предельного штифта в соответствии с нумерацией внутренних осей Grbl
uint8_t get_limit_pin_mask(uint8_t i);

 
 
#endif
