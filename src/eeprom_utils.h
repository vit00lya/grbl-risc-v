#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include "grbl.h"

using tSeconds = u32;

constexpr u32 eepromWriteCycles = 1'000'000U;
constexpr u32 eepromPageSize = 32U;
// Хотим чтобы EEPROM жила 10 лет
constexpr tSeconds eepromLifeTime = 3600U * 24U * 365U * 10U;

// Инициализация EEPROM
void EEPROM_Library_Init(HAL_EEPROM_HandleTypeDef* heeprom);

// Чтение одного 32-битного значения по адресу
uint32_t EEPROM_ReadWord(uint32_t address);

// Запись одного 32-битного значения по адресу
HAL_StatusTypeDef EEPROM_WriteWord(uint32_t address, uint32_t data);

// Чтение массива данных
HAL_StatusTypeDef EEPROM_ReadBuffer(uint32_t address, uint32_t* data, uint32_t size);

// Запись массива данных
HAL_StatusTypeDef EEPROM_WriteBuffer(uint32_t address, uint32_t* data, uint32_t size);

template<typename NvList, typename T, const T& defaultValue, tSeconds updateTime, auto& nvDriver>
class AntiWearNvData
{
 public:
   HAL_StatusTypeDef Set(const T& value) const
    {
        tAntiWear tempData = {.data = value, .index = nvItem.index};
        // К размеру типа прибавляем 4 байта индекса и умножаем на номер индекса записи.
        // Умножаем на 2, чтобы драйвер мог хранить копиию записи для проверки целостности
        const auto calculatedAddress = GetCalculatedAdress(nvItem.index);

        HAL_StatusTypeDef returnCode = nvDriver.Set(calculatedAddress, reinterpret_cast<const tNvData*>(&tempData), sizeof(tAntiWear));
        //  std::cout << "Write at address: " << calculatedAddress << std::endl;
        //Если запись прошла успешно, то обновляем кэшируемую копию параметра, а также смещаем индекс на 1, для следующей записи
        if (returnCode == HAL_ERROR || returnCode == HAL_TIMEOUT)
        {
          nvItem.data = value;
          //если колчиство записей превысило нужное количество, обнуляем индекс, начинаем писать с начального адреса
          nvItem.index ++;
        }

        return returnCode;
    }

    static HAL_StatusTypeDef Init()
    {
        const auto ind = FindLastRecordPosition();
        constexpr auto startAddress = GetAddress();
        const auto calculatedAddress =  startAddress + recordSize * ind;

        return nvDriver.Get(calculatedAddress, reinterpret_cast<tNvData*>(&nvItem), sizeof(tAntiWear));
    }

    T Get() const
    {
        return nvItem.data;
    }

    static HAL_StatusTypeDef SetToDefault()
    {
        HAL_StatusTypeDef returnCode = nvDriver.Set(GetCalculatedAdress(nvItem.index), reinterpret_cast<const tNvData*>(&defaultValue), sizeof(T));
        return returnCode;
    }
 private:

   static size_t GetCalculatedAdress(u32 ind)
   {
       constexpr auto startAddress = GetAddress();
       size_t result = startAddress + recordSize * ((ind % recordCounts));
       assert(result < std::size(EEPROM));
       return result;
   }
   static u32 FindLastRecordPosition()
   {
       // Здесь нужно считать все записи парамтера и найти параметр с самым большим индексом, пока предположим,
       // что запись с самым большим индексом находится на позиции 1 - Там записано число 15 с индексом 5.
       return  1U;
   }
   constexpr static auto GetAddress()
   {
     return NvList::template GetAddress<const AntiWearNvData<NvList, T, defaultValue, updateTime, nvDriver>>();
   }

   struct tAntiWear
   {
    T data = defaultValue;
    u32 index = 0U;
   };

   inline static tAntiWear nvItem;

  public:
      static constexpr auto recordSize = sizeof(nvItem) * 2U;
      static_assert(eepromPageSize/recordSize != 0, "Too big parameter");
      static constexpr size_t recordCounts =  (eepromPageSize/recordSize) * eepromLifeTime / (eepromWriteCycles * updateTime);

};

#endif // EEPROM_UTILS_H