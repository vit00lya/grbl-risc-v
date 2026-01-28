#pragma once
/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

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

/* The cpu_map.h files serve as a central pin mapping selection file for different processor
   types, i.e. AVR 328p or AVR Mega 2560. Each processor has its own pin mapping file.
   (i.e. cpu_map_atmega328p.h)  Grbl officially supports the Arduino Uno, but the 
   other supplied pin mappings are supplied by users, so your results may vary. */

// NOTE: With new processors, only add the define name and filename to use.

/* Файлы cpu_map.h служат в качестве центрального файла для выбора pin-кода для различных типов процессоров
   , например, AVR 328p или AVR Mega 2560. У каждого процессора есть свой собственный файл для отображения pin-кода.
   (например, cpu_map_atmega328p.h) Grbl официально поддерживает Arduino Uno, но
другие прилагаемые схемы сопоставления выводов предоставляются пользователями, поэтому ваши результаты могут отличаться. */

// ПРИМЕЧАНИЕ: При использовании новых процессоров добавляйте только имя define и имя файла для использования.

#ifndef cpu_map_h
#define cpu_map_h

#ifdef CPU_MAP_ELRON_UNO_AMUR
  #include "cpu_map/cpu_map_elron_uno_amur.h"
#endif


#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.
  #include "cpu_map/cpu_map_atmega328p.h"
#endif

#ifdef CPU_MAP_ATMEGA2560 // (Arduino Mega 2560) Working @EliteEng
  #include "cpu_map/cpu_map_atmega2560.h"
#endif

/* 
#ifdef CPU_MAP_CUSTOM_PROC

  // Для пользовательской пин-карты или другого процессора скопируйте и отредактируйте один из доступных процессоров
  // файлы карты и измените их в соответствии с вашими потребностями. Убедитесь, что заданное имя также изменено в
  // файле config.h.

  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
