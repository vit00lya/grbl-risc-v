#pragma once
#ifndef print_h
#define print_h

#include "grbl.h"
#include "serial.h"

class Printer {
private:
    Serial& serial_;

public:
    Printer(Serial& serial);
    
    void String(const char *s);
    void PgmString(const char *s);
    void UnsignedInt8(uint8_t n, uint8_t base, uint8_t digits);
    void Uint8Base2(uint8_t n);
    void Uint8Base10(uint8_t n);
    void Uint32Base10(uint32_t n);
    void Integer(long n);
    void Float(float n, uint8_t decimal_places);
    void FloatCoordValue(float n);
    void FloatRateValue(float n);
    void FloatSettingValue(float n);
    void SerialGetRxBufferCount();
    void IntegerInBase(unsigned long n, unsigned long base);
};

#endif