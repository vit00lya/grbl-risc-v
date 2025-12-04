#include "eeprom_utils.h"

#define EEPROM_OP_TIMEOUT 100000
#define EEPROM_PAGE_WORDS 32
#define EEPROM_PAGE_COUNT 64


// Глобальный указатель на структуру EEPROM
static HAL_EEPROM_HandleTypeDef* eeprom_handle = NULL;

// Инициализация EEPROM
void EEPROM_Library_Init(HAL_EEPROM_HandleTypeDef* heeprom) {
    heeprom->Instance = EEPROM_REGS;
    heeprom->Mode = HAL_EEPROM_MODE_TWO_STAGE;
    heeprom->ErrorCorrection = HAL_EEPROM_ECC_ENABLE;
    heeprom->EnableInterrupt = HAL_EEPROM_SERR_DISABLE;

    HAL_EEPROM_Init(heeprom);
    HAL_EEPROM_CalculateTimings(heeprom, OSC_SYSTEM_VALUE);
    eeprom_handle = heeprom;
}

// Чтение одного 32-битного значения по адресу
uint32_t EEPROM_ReadWord(uint32_t address) {
    if (eeprom_handle == NULL) {
        return 0; // Возвращаем 0 если библиотека не инициализирована
    }
    
    // Проверка корректности адреса
    if (address >= (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return 0; // Возвращаем 0 если адрес вне диапазона
    }
    
    uint32_t data = 0;
    HAL_EEPROM_Read(eeprom_handle, address, &data, 1, EEPROM_OP_TIMEOUT);
    return data;
}

// Запись одного 32-битного значения по адресу
HAL_StatusTypeDef EEPROM_WriteWord(uint32_t address, uint32_t data) {
    if (eeprom_handle == NULL) {
        return HAL_ERROR; // Возвращаем ошибку если библиотека не инициализирована
    }
    
    // Проверка корректности адреса
    if (address >= (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return HAL_ERROR; // Возвращаем ошибку если адрес вне диапазона
    }
    
    return HAL_EEPROM_Write(eeprom_handle, address, &data, 1, HAL_EEPROM_WRITE_ALL, EEPROM_OP_TIMEOUT);
}

// Чтение массива данных
HAL_StatusTypeDef EEPROM_ReadBuffer(uint32_t address, uint32_t* data, uint32_t size) {
    if (eeprom_handle == NULL) {
        return HAL_ERROR; // Возвращаем ошибку если библиотека не инициализирована
    }
    
    // Проверка корректности адреса и размера
    if ((address + size * sizeof(uint32_t)) > (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return HAL_ERROR; // Возвращаем ошибку если данные выходят за пределы EEPROM
    }
    
    return HAL_EEPROM_Read(eeprom_handle, address, data, size, EEPROM_OP_TIMEOUT);
}

// Запись массива данных
HAL_StatusTypeDef EEPROM_WriteBuffer(uint32_t address, uint32_t* data, uint32_t size) {
    if (eeprom_handle == NULL) {
        return HAL_ERROR; // Возвращаем ошибку если библиотека не инициализирована
    }
    
    // Проверка корректности адреса и размера
    if ((address + size * sizeof(uint32_t)) > (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return HAL_ERROR; // Возвращаем ошибку если данные выходят за пределы EEPROM
    }
    
    return HAL_EEPROM_Write(eeprom_handle, address, data, size,HAL_EEPROM_WRITE_ALL, EEPROM_OP_TIMEOUT);
}