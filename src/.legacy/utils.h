#pragma once

#ifndef utils_h
#define utils_h

#include <math.h>
#include <optional>
#include "grbl.h"
// #include "machine.h"


/* чтобы «протащить» через несколько макросов несколько аргументов как один аргумент */
// #define  _(...)  __VA_ARGS__

/* превращает число в строку средствами препроцессора */
// #ifndef  stringify
//     #define  pro_stringify(a)  #a
//     #define  stringify(a)      pro_stringify(a)
// #endif

/* упрощённая работа с портами ввода/вывода */

// #define  io_RCC_EN(_p_,_b_)  PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_GPIO_##_p_##_M
// #define  io_port(_p_,_b_)    (GPIO_##_p_)
// #define  io_bit(_p_,_b_)     (_b_)
// #define  io_bit_n(port_bit)  io_bit(port_bit)

// #define  io_inp(port_bit)  do { io_RCC_EN(port_bit); io_port(port_bit)->DIRECTION_IN  = 1 << io_bit(port_bit); } while(0)
// #define  io_out(port_bit)  do { io_RCC_EN(port_bit); io_port(port_bit)->DIRECTION_OUT = 1 << io_bit(port_bit); } while(0)
// #define  io_set(port_bit)    io_port(port_bit)->SET   = 1 << io_bit(port_bit)
// #define  io_clr(port_bit)    io_port(port_bit)->CLEAR = 1 << io_bit(port_bit)
// #define  io_read(port_bit)  (io_port(port_bit)->STATE >> io_bit(port_bit)  &  1)
// #define  io_SET_R(port_bit)  (&io_port(port_bit)->SET  )
// #define  io_CLR_R(port_bit)  (&io_port(port_bit)->CLEAR)


// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
// Назначение электродвигателей CoreXY. НЕ ИЗМЕНЯТЬ.
// ПРИМЕЧАНИЕ: Если изменить привязку осей электродвигателей A и B, это повлияет на уравнения CoreXY.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
// Конверсии
//#define MM_PER_INCH (25.40)
//#define INCH_PER_MM (0.0393701)
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

// Useful macros
// Полезные макросы
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
// #define max(a,b) (((a) > (b)) ? (a) : (b))
// #define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
// Битовое поле и маскирующие макросы
#define bit(n) (1 << n) 
// #define bit_true_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) |= (mask); SREG = sreg; }
// #define bit_false_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) &= ~(mask); SREG = sreg; }
// #define bit_toggle_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) ^= (mask); SREG = sreg; }
// #define bit_true(x,mask) (x) |= (mask)
// #define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
// #define bit_isfalse(x,mask) ((x & mask) == 0)

void SystemClockConfig(Timer16_HandleTypeDef& timer_step);

GPIO_PinState ReadPin(GPIO_TypeDef *GPIO_x, HAL_PinsTypeDef pin);
void PinInitInputIRQ(const HAL_PinsTypeDef pin, GPIO_TypeDef* port, HAL_GPIO_PullTypeDef pull, HAL_GPIO_Line_Config irq_line);
HAL_StatusTypeDef PinInitInput(const HAL_PinsTypeDef pin, GPIO_TypeDef* port, HAL_GPIO_PullTypeDef pull);
HAL_StatusTypeDef PinInitOutput(const HAL_PinsTypeDef pin, GPIO_TypeDef* port);
bool PinHightLevel(const HAL_PinsTypeDef pin, GPIO_TypeDef* port);

// Функция для сброса произвольных линий прерываний
void ClearGPIOInterrupts(uint8_t line_mask);

// Функция для сброса конкретных линий прерываний, заданных через HAL_GPIO_Line
void ClearGPIOInterruptLines(uint8_t mask);

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
// Считывает значение с плавающей запятой из строки. Строка указывает на входной буфер, char_counter 
// - это индексатор, указывающий на текущий символ строки, а float_ptr - это 
// указатель на результирующую переменную. Возвращает значение true в случае успешного выполнения
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
// Задержки, определяемые переменной в миллисекундах. Исправлена ошибка совместимости компилятора с _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
// Задержка определяется переменной в микросекундах. Исправлена ошибка совместимости компилятора с _delay_us().
void delay_us(uint32_t us);

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
// Вычисляет гипотенузу, избегая раздутой версии avr-gcc и дополнительной проверки ошибок.
float hypot_f(float x, float y);

#endif