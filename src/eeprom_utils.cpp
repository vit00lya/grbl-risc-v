#include "eeprom_utils.h"

#define EEPROM_OP_TIMEOUT 100000
#define EEPROM_PAGE_WORDS 32
#define EEPROM_PAGE_COUNT 64

// Инициализация EEPROM
EEPROM::EEPROM() {

    HAL_EEPROM_HandleTypeDef heeprom;
    heeprom.Instance = EEPROM_REGS;
    heeprom.Mode = HAL_EEPROM_MODE_TWO_STAGE;
    heeprom.ErrorCorrection = HAL_EEPROM_ECC_ENABLE;
    heeprom.EnableInterrupt = HAL_EEPROM_SERR_DISABLE;

    HAL_EEPROM_Init(&heeprom);
    HAL_EEPROM_CalculateTimings(&heeprom, OSC_SYSTEM_VALUE);
    eeprom_handle_ = std::move(heeprom);
}

// Проверка инициализации EEPROM
bool EEPROM::IsInitialized() const {
    return eeprom_handle_.Instance != nullptr;
}

// Чтение одного 32-битного значения по адресу
HAL_StatusTypeDef EEPROM::Get(uint32_t address, uint32_t *data, uint32_t size){
    if (!IsInitialized()) {
        return HAL_ERROR;
    }
    
    // Проверка корректности адреса
    if (address >= (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return HAL_ERROR;
    }
    
    return HAL_EEPROM_Read(&eeprom_handle_, address, data, size, EEPROM_OP_TIMEOUT);
}

// Запись одного 32-битного значения по адресу
HAL_StatusTypeDef EEPROM::Set(uint32_t address, const uint32_t *data, uint32_t size) {
    if (!IsInitialized()) {
        return HAL_ERROR; // Возвращаем ошибку если библиотека не инициализирована
    }
    
    // Проверка корректности адреса
    if (address >= (EEPROM_PAGE_COUNT * EEPROM_PAGE_WORDS)) {
        return HAL_ERROR; // Возвращаем ошибку если адрес вне диапазона
    }
    
    return HAL_EEPROM_Write(&eeprom_handle_, address, const_cast<uint32_t*>(data), size, HAL_EEPROM_WRITE_ALL, EEPROM_OP_TIMEOUT);
}