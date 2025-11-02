/*
  defaults.h - defaults settings configuration file
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

/* The defaults.h file serves as a central default settings selector for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings 
   files listed here are supplied by users, so your results may vary. However, this should
   give you a good starting point as you get to know your machine and tweak the settings for
   your nefarious needs.
   Ensure one and only one of these DEFAULTS_XXX values is defined in config.h */

/* Файл defaults.h служит основным средством выбора настроек по умолчанию для различных
типов станков, от фрезерных станков с ЧПУ "СДЕЛАЙ сам" до готовых станков с ЧПУ для преобразования в ЧПУ. Перечисленные здесь файлы настроек 
   предоставляются пользователями, поэтому результаты могут отличаться. Однако это должно
   это послужит хорошей отправной точкой для изучения вашего компьютера и настройки параметров
в соответствии с вашими потребностями.
   Убедитесь, что в файле config.h указано одно и только одно из значений DEFAULTS_XXX. */

#ifndef defaults_h

// Only define the DEFAULT_XXX with where to find the corresponding default_XXX.h file.
// Don't #define defaults_h here, let the selected file do it. Prevents including more than one.

// Определите только значение DEFAULT_XXX, указав, где найти соответствующий файл default_XXX.h.
// Не #определяйте здесь значение defaults_h, пусть это сделает выбранный файл. Не допускается включение более одного файла.

#ifdef DEFAULTS_GENERIC
  // Общие настройки / Grbl по умолчанию. Должны работать на разных компьютерах.
  // Grbl generic default settings. Should work across different machines.
  #include "defaults/defaults_generic.h"
#endif

#ifdef DEFAULTS_SHERLINE_5400
  // Описание:  Sherline 5400 с тремя шаговыми двигателями NEMA 23 Keling KL23H256-21-8B емкостью 185 унций,
  // приводится в действие тремя шаговыми приводами Pololu A4988 с питанием 30 В, 6 А при напряжении 1,5 А на обмотку.
  // Description: Sherline 5400 mill with three NEMA 23 Keling  KL23H256-21-8B 185 oz-in stepper motors,
  // driven by three Pololu A4988 stepper drivers with a 30V, 6A power supply at 1.5A per winding.
  #include "defaults/defaults_sherline.h"
#endif

#ifdef DEFAULTS_SHAPEOKO
  // Описание: Фрезерный станок с ЧПУ Shapeoko с тремя шаговыми двигателями NEMA 17, приводимыми в движение компанией Synthetos
  // grblShield с блоком питания 24 В, 4,2 А.
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield with a 24V, 4.2A power supply.
  #include "defaults/defaults_shapeoko.h"
#endif

#ifdef DEFAULTS_SHAPEOKO_2
  // Описание: Фрезерный станок с ЧПУ Shapeoko с тремя шаговыми двигателями NEMA 17, приводимыми в движение Synthetos
  // grblShield на 28 В.
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield at 28V.
  #include "defaults/defaults_shapeoko2.h"
#endif

#ifdef DEFAULTS_SHAPEOKO_3
   // Описание: Фрезерный станок с ЧПУ Shapeoko с тремя шаговыми двигателями NEMA 23, приводимыми в движение системой CarbideMotion
  // Description: Shapeoko CNC mill with three NEMA 23 stepper motors, driven by CarbideMotion
  #include "defaults/defaults_shapeoko3.h"
#endif

#ifdef DEFAULTS_X_CARVE_500MM
  // Описание: Фрезерный станок с ЧПУ X-Carve 3D Carver с тремя двигателями с шагом 200 об/мин, приводимыми в движение Synthetos
  // grblShield на 24 В.
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #include "defaults/defaults_x_carve_500mm.h"
#endif

#ifdef DEFAULTS_X_CARVE_1000MM
  // Описание: Фрезерный станок с ЧПУ X-Carve 3D Carver с тремя двигателями с шагом 200 об/мин, приводимыми в движение Synthetos
  // grblShield на 24 В.
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #include "defaults/defaults_x_carve_1000mm.h"
#endif

#ifdef DEFAULTS_ZEN_TOOLWORKS_7x7

// Описание: Фреза Zen Toolworks 7x7 с тремя шаговыми двигателями NEMA 17 Shinano SST43D2121 емкостью 65 унций.
  // Ходовой винт отличается от некоторых комплектов ZTW, где в большинстве случаев скорость составляет 1,25 мм/ об / мин, а не 8,0 мм / об / мин.
  // Привод осуществляется от источника питания 30 В, 6 А и драйверов шаговых двигателей TI DRV8811.

  // Description: Zen Toolworks 7x7 mill with three Shinano SST43D2121 65oz-in NEMA 17 stepper motors.
  // Leadscrew is different from some ZTW kits, where most are 1.25mm/rev rather than 8.0mm/rev here.
  // Driven by 30V, 6A power supply and TI DRV8811 stepper motor drivers.
  #include "defaults/defaults_zen_toolworks_7x7.h"
#endif

#ifdef DEFAULTS_OXCNC
  // Настройки Grbl для станка с ЧПУ OpenBuilds OX
  // http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  // Grbl settings for OpenBuilds OX CNC Machine
  // http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  #include "defaults/defaults_oxcnc.h"
#endif

#ifdef DEFAULTS_SIMULATOR
  // Настройки только для симулятора Grbl (www.github.com/grbl/grbl-sim)
  // Settings only for Grbl Simulator (www.github.com/grbl/grbl-sim)
  #include "defaults/defaults_simulator.h"
#endif

#endif
