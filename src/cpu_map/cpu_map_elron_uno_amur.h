/*
  cpu_map_elron_uno_amur.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2025 Sandalov Vitaly (vit00lya@yandex.ru)

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

   
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

/* чтобы «протащить» через несколько макросов несколько аргументов как один аргумент */
#define  _(...)  __VA_ARGS__

/* превращает число в строку средствами препроцессора */
#ifndef  stringify
    #define  pro_stringify(a)  #a
    #define  stringify(a)      pro_stringify(a)
#endif

/* упрощённая работа с портами ввода/вывода */

#define  io_RCC_EN(_p_,_b_)  PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_##_p_##_M
#define  io_port(_p_,_b_)    (GPIO_##_p_)
#define  io_bit(_p_,_b_)     (_b_)
#define  io_bit_n(port_bit)  io_bit(port_bit)

#define  io_inp(port_bit)  do { io_RCC_EN(port_bit); io_port(port_bit)->DIRECTION_IN  = 1 << io_bit(port_bit); } while(0)
#define  io_out(port_bit)  do { io_RCC_EN(port_bit); io_port(port_bit)->DIRECTION_OUT = 1 << io_bit(port_bit); } while(0)
#define  io_set(port_bit)    io_port(port_bit)->SET   = 1 << io_bit(port_bit)
#define  io_clr(port_bit)    io_port(port_bit)->CLEAR = 1 << io_bit(port_bit)
#define  io_read(port_bit)  (io_port(port_bit)->STATE >> io_bit(port_bit)  &  1)
#define  io_SET_R(port_bit)  (&io_port(port_bit)->SET  )
#define  io_CLR_R(port_bit)  (&io_port(port_bit)->CLEAR)


#define GRBL_PLATFORM "RiscV-Amur32"
#define ELRON_ACE_UNO

#define  USER_LED  2,7

// Define serial port pins and interrupt vectors.
// Определите контакты последовательного порта и векторы прерываний.
#define SERIAL_RX     USART_RX_vect
#define SERIAL_UDRE   USART_UDRE_vect

// #define axes 'X','Y','Z'

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
// Определите выводы ступенчатого импульсного выхода. ПРИМЕЧАНИЕ: Все выводы ступенчатого разряда должны быть подключены к одному порту.
//#define STEP_DDR        DDRD
//#define STEP_PORT       PORTD
#define X_STEP_BIT      0,10  // Uno Digital Pin 2
#define Y_STEP_BIT      0,0  // Uno Digital Pin 3
#define Z_STEP_BIT      0,8  // Uno Digital Pin 4
//#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
// Определите выходные контакты, указывающие направление шага. ПРИМЕЧАНИЕ: Все контакты, указывающие направление, должны быть подключены к одному порту.
//#define DIRECTION_DDR     DDRD
//#define DIRECTION_PORT    PORTD
#define X_DIRECTION_BIT   0,1  // Uno Digital Pin 5
#define Y_DIRECTION_BIT   0,2  // Uno Digital Pin 6
#define Z_DIRECTION_BIT   1,8  // Uno Digital Pin 7
//#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
// Определите, как драйвер шагового двигателя включает/отключает выходной вывод.
//#define STEPPERS_DISABLE_DDR    DDRB
//#define STEPPERS_DISABLE_PORT   PORTB
#define STEPPERS_DISABLE_BIT    1,9  // Uno Digital Pin 8
//#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
// Определите входные контакты самонаведения/жесткого концевого выключателя и векторы ограничения прерываний. 
// ПРИМЕЧАНИЕ: Все контакты концевых битов должны быть подключены к одному порту, но не к порту с другими входными контактами (УПРАВЛЯЮЩИМИ).
//#define LIMIT_DDR        DDRB
//#define LIMIT_PIN        PINB
//#define LIMIT_PORT       PORTB

#define X_LIMIT_BIT      0,3  // Uno Digital Pin 9
#define Y_LIMIT_BIT      1,3  // Uno Digital Pin 10
#ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define Z_LIMIT_BIT	   1,0 // Uno Digital Pin 12
#else
  #define Z_LIMIT_BIT    1,1  // Uno Digital Pin 11
#endif

// Линия 3 - X
// Линия 7 - Y
// Линия 0 - Z
#define X_LIMIT_BIT_PORT      GPIO_0
#define X_LIMIT_BIT_PIN       GPIO_PIN_3
#define X_LIMIT_BIT_LINE_IRQ  GPIO_MUX_LINE_3_PORT0_3
#define X_LIMIT_LINE_IRQ      GPIO_LINE_3

#define Y_LIMIT_BIT_PORT      GPIO_1
#define Y_LIMIT_BIT_PIN       GPIO_PIN_3
#define Y_LIMIT_BIT_LINE_IRQ  GPIO_MUX_LINE_7_PORT1_3
#define Y_LIMIT_LINE_IRQ      GPIO_LINE_7

#ifdef VARIABLE_SPINDLE
  #define Z_LIMIT_BIT_PORT      GPIO_1
  #define Z_LIMIT_BIT_PIN       GPIO_PIN_0
  #define Z_LIMIT_BIT_LINE_IRQ  GPIO_MUX_LINE_0_PORT1_0
  #define Z_LIMIT_LINE_IRQ      GPIO_LINE_0
#else
  #define Z_LIMIT_BIT_PORT      GPIO_1
  #define Z_LIMIT_BIT_PIN       GPIO_PIN_1
  #define Z_LIMIT_BIT_LINE_IRQ  GPIO_MUX_LINE_5_PORT1_1
  #define Z_LIMIT_LINE_IRQ      GPIO_LINE_5
#endif

#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
// #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
// #define LIMIT_INT_vect   PCINT0_vect 
// #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

// Define spindle enable and spindle direction output pins.
//#define SPINDLE_ENABLE_DDR    DDRB
//#define SPINDLE_ENABLE_PORT   PORTB
// Z Limit pin and spindle PWM/enable pin swapped to access hardware PWM on Pin 11.
// Z Ограничьте вывод и ШИМ шпинделя/включите замену вывода для доступа к аппаратной ШИМ на выводе 11.
#ifdef VARIABLE_SPINDLE 
  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
    // If enabled, spindle direction pin now used as spindle enable, while PWM remains on D11.
    #define SPINDLE_ENABLE_BIT    1,2  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
  #else
    #define SPINDLE_ENABLE_BIT    1,1  // Uno Digital Pin 11
  #endif
#else
  #define SPINDLE_ENABLE_BIT    1,0  // Uno Digital Pin 12
#endif
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
  //#define SPINDLE_DIRECTION_DDR   DDRB
  //#define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   1,2  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.) // Цифровой вывод Uno 13 (ПРИМЕЧАНИЕ: D13 не может быть извлечен - высокий вход из-за светодиода).
#endif 
  
// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
// Определите выходные контакты для подключения охлаждающей жидкости типа flood и mist.
// ПРИМЕЧАНИЕ: Аналоговые контакты Uno 4 и 5 зарезервированы для интерфейса i2c и могут быть установлены на
// более позднем этапе, если позволят флэш-память и объем оперативной памяти.
//#define COOLANT_FLOOD_DDR   DDRC
//#define COOLANT_FLOOD_PORT  PORTC
#define COOLANT_FLOOD_BIT   0,7  // Uno Analog Pin 3
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
 // #define COOLANT_MIST_DDR   DDRC
  //#define COOLANT_MIST_PORT  PORTC
  #define COOLANT_MIST_BIT   0,9 // Uno Analog Pin 4
#endif  

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
// Определите входные контакты для управления пользователем (запуск цикла, сброс, удержание подачи).
// ПРИМЕЧАНИЕ: Все управляющие контакты должны быть подключены к одному порту, а не к порту с другими входными контактами (ограничения).
//#define CONTROL_DDR       DDRC
//#define CONTROL_PIN       PINC
//#define CONTROL_PORT      PORTC
#define RESET_BIT         1,5  // Uno Analog Pin 0
#define FEED_HOLD_BIT     1,7  // Uno Analog Pin 1
#define CYCLE_START_BIT   0,4  // Uno Analog Pin 2
#define SAFETY_DOOR_BIT   1,7  // Uno Analog Pin 1 NOTE: Safety door is shared with feed hold. Enabled by config define.
#define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT1_vect
#define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.
  
// Определите входной контакт датчика высоты стола.
// Define probe switch input pin.
#define PROBE_DDR       DDRC
#define PROBE_PIN       PINC
#define PROBE_PORT      PORTC
#define PROBE_BIT       0,9  // Uno Analog Pin 5
#define PROBE_MASK      (1<<PROBE_BIT)

// Запуск шпинделя с поддержкой ШИМ и шагового управления
// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  #define PWM_MAX_VALUE    255.0
  #define TCCRA_REGISTER	 TCCR2A
  #define TCCRB_REGISTER	 TCCR2B
  #define OCR_REGISTER     OCR2A
  
  #define COMB_BIT	     COM2A1
  #define WAVE0_REGISTER	 WGM20
  #define WAVE1_REGISTER	 WGM21
  #define WAVE2_REGISTER	 WGM22
  #define WAVE3_REGISTER	 WGM23
      
  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
  #define SPINDLE_PWM_DDR	  DDRB
  #define SPINDLE_PWM_PORT  PORTB
  #define SPINDLE_PWM_BIT	  1,1    // Uno Digital Pin 11
#endif // End of VARIABLE_SPINDLE
