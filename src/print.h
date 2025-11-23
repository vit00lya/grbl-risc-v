#pragma once
#ifndef print_h
#define print_h

#include "grbl.h"

void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint32_base10(u32 n);

// Prints uint8 variable with base and number of desired digits.
// Выводит переменную uint8 с основанием и количеством нужных цифр.
void print_unsigned_int8(u8 n, u8 base, u8 digits); 

// Prints an uint8 variable in base 2.
void print_uint8_base2(u8 n);

// Prints an uint8 variable in base 10.
void print_uint8_base10(u8 n);

void printFloat(float n, u8 decimal_places);

// Floating value printing handlers for special variables types used in Grbl. 
//  - CoordValue: Handles all position or coordinate values in inches or mm reporting.
//  - RateValue: Handles feed rate and current velocity in inches or mm reporting.
//  - SettingValue: Handles all floating point settings values (always in mm.)
// Обработчики печати с плавающим значением для специальных типов переменных, используемых в Grbl. 
// - CoordValue: Отображает все значения положения или координат в дюймах или мм.
// - RateValue: Отображает скорость подачи и текущую скорость в дюймах или мм.
// - SettingValue: Обрабатывает все значения настроек с плавающей запятой (всегда в мм).
void printFloat_CoordValue(float n);

void printFloat_RateValue(float n);

void printFloat_SettingValue(float n);

// Debug tool to print free memory in bytes at the called point. Not used otherwise.
// Инструмент отладки для вывода данных о свободной памяти в байтах в вызываемой точке. В противном случае не используется.
void printFreeMemory();

#endif