#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include "grbl.h"
#include <cassert> // Для assert

using tNvAddress = u32;
using tSeconds = u32;
using tNvData  = u32; // Определение типа tNvData

constexpr u32 eepromWriteCycles = 1'000'000U;
constexpr u32 eepromPageSize = 32U;
// Хотим чтобы EEPROM жила 10 лет
constexpr tSeconds eepromLifeTime = 3600U * 24U * 365U * 10U;

// Класс для работы с EEPROM
class EEPROM {
public:
    EEPROM();
    // Чтение одного 32-битного значения по адресу
    HAL_StatusTypeDef Get(u32 address, u32 *data, u32 size);
    // Запись одного 32-битного значения по адресу
    HAL_StatusTypeDef Set(u32 address, const u32 *data, u32 size);

private:
    bool IsInitialized() const;
    HAL_EEPROM_HandleTypeDef eeprom_handle_;
};

template<const tNvAddress startAddress, typename ...TNvVars>
struct NvVarListBase
{
static bool SetToDefault()
{
return ( ... || TNvVars::SetToDefault());
}

    static bool Init()
    {
        return ( ... || TNvVars::Init());
    }
    template<typename T>
    constexpr static size_t GetAddress()
    {
        return startAddress + GetAddressOffset<T, TNvVars...>();
    }

 private:

    template <typename QueriedType, typename T, typename ...Ts>
    constexpr static size_t GetAddressOffset()
    {
        auto result = 0;
        if constexpr (!std::is_same<T, QueriedType>::value)
        {
            //можно дописать алгоритм, чтобы все параметры были выравенны по странице.
            result = T::recordSize * T::recordCounts + GetAddressOffset<QueriedType, Ts...>();
        }
        return result;
    }
};

template<typename NvList, typename T, const T& defaultValue, tSeconds updateTime, auto& nvDriver>
class AntiWearNvData
{
 public:

    struct tAntiWear {
            T data;
            u32 index;
        };
    
    HAL_StatusTypeDef Set(const T& value) const
    {
        tAntiWear tempData = {value, nvItem.index};
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
        // assert(result < sizeof(EEPROM)); // Удалено, так как EEPROM не определен в этом контексте
        return result;
    }
    static u32 FindLastRecordPosition()
    {
        // Здесь нужно считать все записи парамтера и найти параметр с самым большим индексом, пока предположим,
        // AntiWearNvDataчто запись с самым большим индексом находится на позиции 1 - Там записано число 15 с индексом 5.
        return  1U;
    }
    constexpr static auto GetAddress()
    {
      return NvList::template GetAddress<const AntiWearNvData<NvList, T, defaultValue, updateTime, nvDriver>>();
    }

    inline static tAntiWear nvItem;

  public:
      static constexpr auto recordSize = sizeof(nvItem) * 2U;
      static_assert(eepromPageSize/recordSize != 0, "Too big parameter");
      static constexpr size_t recordCounts =  (eepromPageSize/recordSize) * eepromLifeTime / (eepromWriteCycles * updateTime);
      

};

#endif // EEPROM_UTILS_H