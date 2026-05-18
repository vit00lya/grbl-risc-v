// This file has been prepared for Doxygen automatic documentation generation.
/**
 * @file eeprom.cpp
 * @brief EEPROM implementation for MIK32 platform (ELRON_ACE_UNO)
 */

#include <cstdint>

extern "C"{
#include <mik32_hal_eeprom.h>
#include <mik32_memory_map.h>
}
#include "grbl.hpp"

// Настройки для АМУРА
#define EEPROM_OP_TIMEOUT 100000
#define EEPROM_PAGE_WORDS 32
#define EEPROM_PAGE_COUNT 64

// EEPROM handle
static HAL_EEPROM_HandleTypeDef heeprom;
static bool eeprom_initialized = false;

/**
 * @brief Инициализация EEPROM.
 *
 * Эта функция инициализирует аппаратуру EEPROM и настраивает дескриптор
 * для платформы MIK32. Она устанавливает EEPROM в двухстадийном режиме
 * с включённой коррекцией ошибок и отключёнными прерываниями. После инициализации
 * вычисляются тайминги EEPROM для системной частоты 32 МГц.
 *
 * @note Эту функцию необходимо вызвать перед любыми другими операциями с EEPROM.
 *       При успехе она устанавливает глобальный флаг `eeprom_initialized` в true.
 */
void eeprom_init()
{
	heeprom.Instance = EEPROM_REGS;
	heeprom.Mode = HAL_EEPROM_MODE_TWO_STAGE;
	heeprom.ErrorCorrection = HAL_EEPROM_ECC_ENABLE;
	heeprom.EnableInterrupt = HAL_EEPROM_SERR_DISABLE;
	
    HAL_EEPROM_Init(&heeprom);
    HAL_EEPROM_CalculateTimings(&heeprom, 32000000); // Настраиваем часы на 32MHz
    eeprom_initialized = true;
}


/**
 * @brief Очистка страниц EEPROM.
 *
 * Эта функция очищает (стирает) указанное количество 32-битных слов, начиная с заданной страницы.
 * Если EEPROM не инициализирована, функция возвращает HAL_ERROR.
 * Функция проверяет корректность номера страницы и количества слов.
 * Если word_count равно 0, функция возвращает HAL_OK без выполнения операций.
 * Параметр data не используется, но требуется для совместимости с HAL.
 *
 * @param start_page Номер начальной страницы (0‑EEPROM_PAGE_COUNT-1).
 * @param word_count Количество 32-битных слов для очистки.
 * @return HAL_StatusTypeDef Статус выполнения операции (HAL_OK, HAL_ERROR).
 */
HAL_StatusTypeDef eeprom_erase_from_page(
    uint8_t start_page,
    uint8_t word_count
) {
    if (!eeprom_initialized) {
        return HAL_ERROR;
    }
    
    // Проверка параметров
    if (start_page >= EEPROM_PAGE_COUNT) {
        return HAL_ERROR; // Некорректный номер страницы
    }
    
    if (word_count == 0) {
        return HAL_OK; // Ничего не нужно записывать
    }
    
    // Вызов стандартной функции очистки HAL
    return HAL_EEPROM_Erase(&heeprom, start_page, word_count, HAL_EEPROM_WRITE_SINGLE, EEPROM_OP_TIMEOUT);
}

/**
 * @brief Запись данных в EEPROM начиная с указанной страницы.
 *
 * Эта функция записывает массив 32-битных слов в EEPROM, начиная с указанной страницы.
 * Если EEPROM не инициализирована, функция возвращает HAL_ERROR.
 * Функция автоматически вычисляет адрес в байтах на основе номера страницы.
 * Используется режим записи HAL_EEPROM_WRITE_SINGLE.
 *
 * @param start_page Номер начальной страницы (0‑EEPROM_PAGE_COUNT-1).
 * @param data Указатель на массив данных для записи.
 * @param word_count Количество 32-битных слов для записи.
 * @return HAL_StatusTypeDef Статус выполнения операции (HAL_OK, HAL_ERROR, HAL_TIMEOUT).
 */
HAL_StatusTypeDef eeprom_write_from_page(
    uint8_t start_page,
    uint32_t *data,
    uint8_t word_count
) {
    if (!eeprom_initialized) {
        return HAL_ERROR;
    }
    
    // Проверка параметров
    if (start_page >= EEPROM_PAGE_COUNT) {
        return HAL_ERROR; // Некорректный номер страницы
    }
    
    if (word_count == 0) {
        return HAL_OK; // Ничего не нужно записывать
    }
    
    // Вычисление начального адреса в байтах
    uint16_t start_address = start_page * EEPROM_PAGE_WORDS * 4;
    
    // Проверка, не выходим ли мы за пределы EEPROM
    uint16_t max_words = (EEPROM_PAGE_COUNT - start_page) * EEPROM_PAGE_WORDS;
    if (word_count > max_words) {
        word_count = max_words; // Ограничиваем количество слов
    }
    
    // Вызов стандартной функции записи HAL
    return HAL_EEPROM_Write(&heeprom, start_address, data, word_count, HAL_EEPROM_WRITE_SINGLE, EEPROM_OP_TIMEOUT);
}

/**
 * @brief Чтение страницы EEPROM.
 *
 * Эта функция читает одну страницу EEPROM (размером EEPROM_PAGE_WORDS слов) в буфер.
 * Если EEPROM не инициализирована, функция ничего не делает (буфер остаётся неизменным).
 * Если номер страницы некорректен, функция также ничего не делает.
 *
 * @param page_number Номер страницы (0..EEPROM_PAGE_COUNT-1)
 * @param data Указатель на буфер размером EEPROM_PAGE_WORDS слов (uint32_t) для сохранения данных
 */
void read_eeprom_page(unsigned int page_number, uint32_t *data)
{
    if (!eeprom_initialized) {
        return;
    }
    
    // Проверка параметров
    if (page_number >= EEPROM_PAGE_COUNT) {
        return; // Некорректный номер страницы
    }
    
    // Вычисление начального адреса в байтах
    uint16_t start_address = page_number * EEPROM_PAGE_WORDS * 4;
    
    // Чтение всей страницы за один вызов HAL_EEPROM_Read
    if (HAL_EEPROM_Read(&heeprom, start_address, data, EEPROM_PAGE_WORDS, EEPROM_OP_TIMEOUT) != HAL_OK) {
        // При ошибке чтения заполняем буфер нулями
        for (unsigned int i = 0; i < EEPROM_PAGE_WORDS; ++i) {
            data[i] = 0;
        }
        return;
    }
}

// Extensions added as part of Grbl

/**
 * @brief Копирование данных в EEPROM с контрольной суммой.
 *
 * Эта функция копирует `size` байт из оперативной памяти `source` в EEPROM,
 * начиная со страницы `destination`. Во время копирования вычисляется 8‑битная
 * циклическая контрольная сумма (вращение влево и сложение). После данных
 * контрольная сумма сохраняется в следующий байт EEPROM.
 *
 * @param destination Начальная страница EEPROM для записи данных (0..EEPROM_PAGE_COUNT-1).
 * @param count_page  Количество выделенных страниц для записи.
 * @param source Указатель на исходные данные в ОЗУ.
 * @param offset смещение начала размещения исходных данных в странице 
 * @param size Количество байт для копирования.
 */
void memcpy_to_eeprom_with_checksum(unsigned int destination, unsigned int count_page, char *source, unsigned int offset, unsigned int size) {

   // Проверка на нулевой размер
    if (size == 0) {
        return;
    }

    // Проверка, что EEPROM инициализирована
    if (!eeprom_initialized) {
        return;
    }

    // Проверка, что начальная страница допустима
    if (destination >= EEPROM_PAGE_COUNT) {
        return;
    }

    // Проверка, что выделено хотя бы одна страница
    if (count_page == 0) {
        return;
    }

    // Вычисляем общий размер данных с контрольной суммой
    unsigned int total_size = size + 1; // данные + контрольная сумма
    
    // Вычисляем размер страницы в байтах
    unsigned int page_size_bytes = EEPROM_PAGE_WORDS * 4;
    
    // Проверка, что не выходим за пределы EEPROM
    if (destination + count_page > EEPROM_PAGE_COUNT) {
        return;
    }

    // Проверка, что offset находится в пределах выделенных страниц
    if (offset >= page_size_bytes * count_page) {
        return; // Некорректное смещение
    }
    
    // Максимальный размер данных, который можно записать в выделенные страницы с учетом offset
    unsigned int max_data_size = count_page * page_size_bytes - offset - 1; // минус 1 байт для контрольной суммы
    
    // Проверка, что данные помещаются в выделенные страницы
    if (size > max_data_size) {
        // Данные слишком большие для выделенного пространства
        return;
    }

    // Данные начинаются с указанного смещения внутри начальной страницы
    unsigned int start_page = destination;
    // offset_in_page теперь равен переданному параметру offset
    
    // Вычисляем количество затронутых страниц (данные могут занимать несколько страниц) с учетом offset
    unsigned int needed_pages = (offset + total_size + page_size_bytes - 1) / page_size_bytes; // округление вверх
    
    // Ограничиваем needed_pages количеством выделенных страниц
    if (needed_pages > count_page) {
        needed_pages = count_page;
    }
    
    // Статический буфер для затронутых страниц (в словах)
    uint32_t buffer[needed_pages * EEPROM_PAGE_WORDS] = {0};


    // 1. Чтение данных из EEPROM во временный буфер с использованием read_eeprom_page
    for (unsigned int page = 0; page < needed_pages; ++page) {
        unsigned int global_page = start_page + page;
        read_eeprom_page(global_page, buffer + page * EEPROM_PAGE_WORDS);
    }

    // Вспомогательный байтовый буфер для работы с отдельными байтами
    char *byte_buffer = (char*)buffer;

    // 2. Модификация буфера новыми данными и вычисление контрольной суммы
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < size; ++i) {
        uint8_t byte = static_cast<uint8_t>(source[i]);
        
        // Циклический сдвиг влево на 1 бит (вращение)
        checksum = (checksum << 1) | (checksum >> 7);
        // Сложение с байтом (по модулю 256)
        checksum += byte;

        // Замена байта в буфере
        unsigned int buffer_index = offset + i;
        // Проверка границ буфера (на всякий случай)
        if (buffer_index < needed_pages * page_size_bytes) {
            byte_buffer[buffer_index] = byte;
        }
    }

    // 3. Запись контрольной суммы в буфер
    unsigned int checksum_index = offset + size;
    if (checksum_index < needed_pages * page_size_bytes) {
        byte_buffer[checksum_index] = checksum;
    }

    HAL_StatusTypeDef result;

    // 4. Запись буфера обратно в EEPROM по одной странице за проход
    for (unsigned int page = 0; page < needed_pages; ++page) {
       unsigned int global_page = start_page + page;
       uint32_t *page_data = buffer + page * EEPROM_PAGE_WORDS;
       result = eeprom_write_from_page(global_page, page_data, EEPROM_PAGE_WORDS);
       if (result == HAL_OK)
       {
        delay_ms(1);
       }
    }
}

/**
 * @brief Копирование данных из EEPROM с проверкой контрольной суммы.
 *
 * @param destination Указатель на буфер назначения в ОЗУ.
 * @param source_page Начальная страница EEPROM.
 * @param count_page  Количество страниц для чтения.
 * @param offset  Смещение для чтения.
 * @param size Количество байт для чтения.
 * @return 1 (истина), если контрольная сумма совпадает, иначе 0 (ложь).
 */
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source_page, unsigned int count_page, unsigned int offset, unsigned int size) {
    //Проверка на нулевой размер
    if (size == 0) {
        return 0;
    }

    // Проверка, что EEPROM инициализирована
    if (!eeprom_initialized) {
        return 0;
    }

    // Проверка, что начальная страница допустима
    if (source_page >= EEPROM_PAGE_COUNT) {
        return 0;
    }

    // Проверка, что выделено хотя бы одна страница
    if (count_page == 0) {
        return 0;
    }

    // Вычисляем общий размер данных с контрольной суммой
    unsigned int total_size = size + 1; // данные + контрольная сумма
    
    // Вычисляем размер страницы в байтах
    unsigned int page_size_bytes = EEPROM_PAGE_WORDS * 4;
    
    // Проверка, что не выходим за пределы EEPROM
    if (source_page + count_page > EEPROM_PAGE_COUNT) {
        return 0;
    }

    // Проверка, что offset находится в пределах страниц
    if (offset >= count_page * page_size_bytes) {
        return 0; // Некорректное смещение
    }
    
    // Максимальный размер данных, который можно прочитать из выделенных страниц с учетом offset
    unsigned int max_data_size = count_page * page_size_bytes - offset - 1; // минус 1 байт для контрольной суммы
    
    // Проверка, что данные помещаются в выделенные страницы
    if (size > max_data_size) {
        // Данные слишком большие для выделенного пространства
        return 0;
    }

    // Вычисляем количество затронутых страниц (данные могут занимать несколько страниц) с учетом offset
    unsigned int needed_pages = (offset + total_size + page_size_bytes - 1) / page_size_bytes; // округление вверх
    
    // Ограничиваем needed_pages количеством выделенных страниц
    if (needed_pages > count_page) {
        needed_pages = count_page;
    }


    // Статический буфер для затронутых страниц (в словах)
    uint32_t buffer[needed_pages * EEPROM_PAGE_WORDS] = {0};

    // 1. Чтение данных из EEPROM во временный буфер с использованием read_eeprom_page
    for (unsigned int page = 0; page < needed_pages; ++page) {
        unsigned int global_page = source_page + page;
        read_eeprom_page(global_page, buffer + page * EEPROM_PAGE_WORDS);
    }

    // Вспомогательный байтовый буфер для работы с отдельными байтами
    char *byte_buffer = (char*)buffer;

    // 2. Извлечение данных из буфера и вычисление контрольной суммы
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < size; ++i) {
        unsigned int buffer_index = offset + i;
        // Проверка границ буфера (на всякий случай)
        if (buffer_index >= needed_pages * page_size_bytes) {
            return 0; // Ошибка: выход за границы буфера
        }
        uint8_t byte = static_cast<uint8_t>(byte_buffer[buffer_index]);
        
        // Циклический сдвиг влево на 1 бит (вращение)
        checksum = (checksum << 1) | (checksum >> 7);
        // Сложение с байтом (по модулю 256)
        checksum += byte;

        // Копирование байта в destination
        destination[i] = byte;
    }

    // 3. Чтение сохранённой контрольной суммы из буфера
    unsigned int checksum_index = offset + size;
    if (checksum_index >= needed_pages * page_size_bytes) {
        return 0; // Ошибка: контрольная сумма вне буфера
    }
    uint8_t stored_checksum = static_cast<uint8_t>(byte_buffer[checksum_index]);

    // 4. Сравнение контрольных сумм
    return (checksum == stored_checksum) ? 1 : 0;

}
// end of file
