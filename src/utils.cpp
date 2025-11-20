/*
  nuts_bolts.c - Shared functions
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


#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float) // Максимальное количество цифр в int32 (с плавающей точкой)


// Extracts a floating point value from a string. The following code is based loosely on
// the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
// available conversion method examples, but has been highly optimized for Grbl. For known
// CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
// Scientific notation is officially not supported by g-code, and the 'E' character may
// be a g-code word on some CNC systems. So, 'E' notation will not be recognized. 
// NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
// Извлекает значение с плавающей запятой из строки. Приведенный ниже код в общих чертах основан на
// функции avr-libc strtod() Михаэля Штумпфа и Дмитрия Хмелькова и многих других
// доступных примерах методов преобразования, но был оптимизирован для Grbl. Для известных систем ЧПУ
// типичное десятичное значение, как ожидается, будет находиться в диапазоне от E0 до E-4.
// Научная нотация официально не поддерживается g-кодом, и символ "E" может
// быть g-кодовым словом в некоторых системах ЧПУ. Таким образом, обозначение "E" распознаваться не будет. 
// ПРИМЕЧАНИЕ: Спасибо Раду-Иосифу Михайлеску за выявление проблем с использованием strtod().
u8 read_float(char *line, u8 *char_counter, float *float_ptr)                  
{
  char *ptr = line + *char_counter;
  unsigned char c;
    
  // Grab first character and increment pointer. No spaces assumed in line.
  // Захватите первый символ и увеличьте указатель. Пробелы в строке не допускаются.
  c = *ptr++;
  
  // Capture initial positive/minus character
  // Фиксировать начальный положительный/отрицательный символ
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }
  
  // Extract number into fast integer. Track decimal in terms of exponent value.
  // Преобразуйте число в быстрое целое число. Отследите десятичную дробь по значению экспоненты.
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  // Drop overflow digits // Удалить цифры переполнения
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }
  
  // Return if no digits have been read. // Возвращает, если ни одна цифра не была считана.
  if (!ndigit) { return(false); };
  
  // Convert integer into floating point. // Преобразовать целое число в число с плавающей запятой.
  float fval;
  fval = (float)intval;
  
  // Apply decimal. Should perform no more than two floating point multiplications for the
  // expected range of E0 to E-4.
  // Примените десятичную дробь. Следует выполнить не более двух умножений с плавающей запятой для значения
  // Ожидаемый диапазон от E0 до E-4.
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01; 
      exp += 2;
    }
    if (exp < 0) { 
      fval *= 0.1; 
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    } 
  }

  // Assign floating point value with correct sign.   // Присвоить значение с плавающей запятой с правильным знаком.  
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; // Set char_counter to next statement // Установите char_counter в значение следующего оператора
  
  return(true);
}


// Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
// which only accepts constants in future compiler releases.
// Задерживает переменную, определяемую в миллисекундах. Исправлена ошибка совместимости с компилятором для функции _delay_ms(),
// которая принимает константы только в будущих версиях компилятора.

void delay_ms(u16 ms) 
{
   HAL_DelayMs(ms);
}


// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases. Written to perform more 
// efficiently with larger delays, as the counter adds parasitic time in each iteration.

// Задержка определяется переменной в микросекундах. Исправлена ошибка совместимости с компилятором для _delay_us(),
// которая принимает константы только в будущих версиях компилятора. Написано для выполнения большего 
// эффективно с большими задержками, поскольку счетчик добавляет паразитное время на каждой итерации.
void delay_us(u32 us) 
{
  while (us) {
    if (us < 10) { 
      HAL_DelayUs(1);
      us--;
    } else if (us < 100) {
      HAL_DelayUs(10);
      us -= 10;
    } else if (us < 1000) {
      HAL_DelayUs(100);
      us -= 100;
    } else {
      HAL_DelayMs(1);
      us -= 1000;
    }
  }
}


// Simple hypotenuse computation function.
// Простая функция вычисления гипотенузы.
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }
