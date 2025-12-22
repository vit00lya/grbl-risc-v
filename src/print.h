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
    void UnsignedInt8(u8 n, u8 base, u8 digits);
    void Uint8Base2(u8 n);
    void Uint8Base10(u8 n);
    void Uint32Base10(u32 n);
    void Integer(long n);
    void Float(float n, u8 decimal_places);
    void FloatCoordValue(float n);
    void FloatRateValue(float n);
    void FloatSettingValue(float n);
    void SerialGetRxBufferCount();
};

#endif