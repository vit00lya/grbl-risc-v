#include "grbl.h"
#include "print.h"
#include "serial.h"

Printer::Printer(Serial& serial) : serial_(serial) {}

void Printer::String(const char *s)
{
    xprintf(s);
}


// Print a string stored in PGM-memory
// Вывести строку, сохраненную в PGM-памяти
void Printer::PgmString(const char *s)
{
    xprintf(s);
}


// void printIntegerInBase(unsigned long n, unsigned long base)
// {
// 	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
// 	unsigned long i = 0;
//
// 	if (n == 0) {
// 		serial_write('0');
// 		return;
// 	}
//
// 	while (n > 0) {
// 		buf[i++] = n % base;
// 		n /= base;
// 	}
//
// 	for (; i > 0; i--)
// 		serial_write(buf[i - 1] < 10 ?
// 			'0' + buf[i - 1] :
// 			'A' + buf[i - 1] - 10);
// }


// Prints an uint8 variable with base and number of desired digits.
// Выводит переменную uint8 с основанием и количеством нужных цифр.
void Printer::UnsignedInt8(u8 n, u8 base, u8 digits)
{
  unsigned char buf[digits];
  u8 i = 0;

  for (; i < digits; i++) {
      buf[i] = n % base ;
      n /= base;
  }

  for (; i > 0; i--)
      serial_.Write('0' + buf[i - 1]);
}


// Prints an uint8 variable in base 2.
// Выводит переменную uint8 в базе данных 2.
void Printer::Uint8Base2(u8 n) {
  UnsignedInt8(n,2,8);
}


// Prints an uint8 variable in base 10.
// Выводит переменную uint8 в базе данных 10.
void Printer::Uint8Base10(u8 n)
{
  u8 digits;
  if (n < 10) { digits = 1; }
  else if (n < 100) { digits = 2; }
  else { digits = 3; }
  UnsignedInt8(n,10,digits);
}

void Printer::SerialGetRxBufferCount(){
  Uint8Base10(serial_.SerialGetRxBufferCount());
}


void Printer::Uint32Base10(u32 n)
{
  if (n == 0) {
    serial_.Write('0');
    return;
  }

  unsigned char buf[10];
  u8 i = 0;
  
  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }
    
  for (; i > 0; i--)
    serial_.Write('0' + buf[i-1]);
}


void Printer::Integer(long n)
{
  if (n < 0) {
    serial_.Write('-');
    Uint32Base10(-n);
  } else {
    Uint32Base10(n);
  }
}


// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
// NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up
// techniques are actually just slightly slower. Found this out the hard way.
// Преобразовать значение с плавающей точкой в строку, немедленно преобразовав в длинное целое число, которое содержит
// больше цифр, чем число с плавающей точкой. Количество знаков после запятой, которое отслеживается счетчиком,
// может быть задано пользователем. Затем целое число эффективно преобразуется в строку.
// ПРИМЕЧАНИЕ: Операции с целыми числами AVR '%' и '/' очень эффективны. Это ускоряет переключение битов
// на самом деле эти методы лишь немного медленнее. В этом я убедился на собственном горьком опыте.
void Printer::Float(float n, u8 decimal_places)
{
  if (n < 0) {
    serial_.Write('-');
    n = -n;
  }

  u8 decimals = decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4. // Быстро преобразуйте ожидаемые значения E0 в E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value. // Добавьте коэффициент округления. Обеспечивает перенос всего значения.
    
  // Generate digits backwards and store in string. // Генерировать цифры в обратном порядке и сохранять в виде строки.
  unsigned char buf[10];
  u8 i = 0;
  u32 a = (long)n;
  buf[decimal_places] = '.'; // Place decimal point, even if decimal places are zero. // Поставьте десятичную точку, даже если десятичные разряды равны нулю.
  while(a > 0) {
    if (i == decimal_places) { i++; } // Skip decimal point location // Пропустить расположение десятичной точки
    buf[i++] = (a % 10) + '0'; // Get digit // Получить цифру
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1) // Введите нули с точностью до десятичной точки для (n < 1)
  }
  if (i == decimal_places) { // Fill in leading zero, if needed. // При необходимости введите начальный ноль.
    i++;
    buf[i++] = '0';
  }
  
  // Print the generated string. // Выведите сгенерированную строку.
  for (; i > 0; i--)
    serial_.Write(buf[i-1]);
}


// Floating value printing handlers for special variables types used in Grbl and are defined
// in the config.h.
//  - CoordValue: Handles all position or coordinate values in inches or mm reporting.
//  - RateValue: Handles feed rate and current velocity in inches or mm reporting.
//  - SettingValue: Handles all floating point settings values (always in mm.)
// Обработчики печати с плавающим значением для специальных типов переменных, используемых в Grbl и определенных
// в файле config.h.
// - CoordValue: Обрабатывает все значения положения или координат в дюймах или мм для создания отчетов.
// - RateValue: Отображает скорость подачи и текущую скорость в дюймах или мм.
// - SettingValue: Отображает все значения настроек с плавающей запятой (всегда в мм).
void Printer::FloatCoordValue(float n) {
    Float(n,N_DECIMAL_COORDVALUE_MM);
}

void Printer::FloatRateValue(float n) {
    Float(n,N_DECIMAL_RATEVALUE_MM);
}

void Printer::FloatSettingValue(float n) {
    Float(n,N_DECIMAL_SETTINGVALUE);
}
