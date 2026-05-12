// This file has been prepared for Doxygen automatic documentation generation.
/**
 * @file eeprom.cpp
 * @brief EEPROM implementation for MIK32 platform (ELRON_ACE_UNO)
 */

extern "C"{
#include <mik32_hal_eeprom.h>
#include <mik32_memory_map.h>
}
#include "grbl.hpp"

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
 * @brief Чтение байта из EEPROM.
 *
 * Эта функция читает один байт по заданному адресу EEPROM.
 * Если EEPROM не инициализирована, возвращает 0.
 * Если операция чтения завершается ошибкой HAL, также возвращает 0.
 *
 * @param addr Адрес EEPROM для чтения (0‑EEPROM_SIZE‑1).
 * @return Прочитанный байт или 0 при ошибке/неинициализированной EEPROM.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	if (!eeprom_initialized) {
		return 0;
	}
	
	uint32_t data;
    if (HAL_EEPROM_Read(&heeprom, addr, &data, 1, EEPROM_OP_TIMEOUT) == HAL_OK) {
	 	return (unsigned char)data;
	 }
	return 0; // Return 0 on error
}

/**
 * @brief Запись байта в EEPROM.
 *
 * Эта функция записывает один байт по заданному адресу EEPROM.
 * Если EEPROM не инициализирована, функция ничего не делает.
 * Операция записи использует таймаут 100 мс и записывает всё слово целиком.
 *
 * @param addr Адрес EEPROM для записи (0‑EEPROM_SIZE‑1).
 * @param new_value Новое значение байта для сохранения.
 */
//void eeprom_put_char( unsigned int addr, unsigned char new_value )
//{
//	if (!eeprom_initialized) {
//		return;
//	}
	
//	uint32_t data = (uint32_t)new_value;
//    HAL_EEPROM_Write(&heeprom, addr, &data, 1, HAL_EEPROM_WRITE_SINGLE, EEPROM_OP_TIMEOUT);
//}


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

// Extensions added as part of Grbl

/**
 * @brief Копирование данных в EEPROM с контрольной суммой.
 *
 * Эта функция копирует `size` байт из оперативной памяти `source` в EEPROM,
 * начиная с адреса `destination`. Во время копирования вычисляется 8‑битная
 * циклическая контрольная сумма (вращение влево и сложение). После данных
 * контрольная сумма сохраняется в следующий байт EEPROM (по адресу `destination + size`).
 * Функция использует страничную запись через eeprom_write_from_page для выровненных блоков,
 * что повышает производительность.
 *
 * @param destination Начальный адрес EEPROM для данных.
 * @param source Указатель на исходные данные в ОЗУ.
 * @param size Количество байт для копирования.
 */
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  // Вычисляем контрольную сумму по исходным данным
  for (unsigned int i = 0; i < size; i++) {
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += (unsigned char)source[i];
  }

  // Определяем диапазон слов, которые нужно записать (данные + контрольная сумма)
  unsigned int total_bytes = size + 1; // данные + checksum
  unsigned int start_word = destination / 4;
  unsigned int end_word = (destination + total_bytes + 3) / 4; // exclusive
  unsigned int word_count = end_word - start_word;

  if (word_count == 0) {
    return; // нечего записывать
  }

  // Буфер для слов (максимум 256 слова, что соответствует 8 страницам)
  uint32_t word_buffer[256];
  if (word_count > 256) {
    // На случай очень больших данных - упрощённо обработаем по частям (но в Grbl размеры небольшие)
    // Для простоты ограничимся 256 словами (1024 байт)
    word_count = 256;
  }

  // Заполняем буфер слов
  for (unsigned int w = 0; w < word_count; w++) {
    uint32_t word = 0;
    unsigned int word_addr = (start_word + w) * 4; // начальный байтовый адрес слова
    // Для каждого байта в слове (0..3)
    for (unsigned int b = 0; b < 4; b++) {
      unsigned int byte_addr = word_addr + b;
      if (byte_addr >= destination && byte_addr < destination + size) {
        // Байт принадлежит исходным данным
        unsigned int src_idx = byte_addr - destination;
        word |= (uint32_t)((unsigned char)source[src_idx]) << (b * 8);
      } else if (byte_addr == destination + size) {
        // Байт контрольной суммы
        word |= (uint32_t)checksum << (b * 8);
      } else {
        // Байт не входит в наш диапазон, читаем существующее значение из EEPROM
        word |= (uint32_t)eeprom_get_char(byte_addr) << (b * 8);
      }
    }
    word_buffer[w] = word;
  }

  // Записываем слова через eeprom_write_from_page, разбивая по страницам
  unsigned int page_size_words = EEPROM_PAGE_WORDS;
  unsigned int current_page = start_word / page_size_words;
  unsigned int offset_in_page = start_word % page_size_words;
  unsigned int words_remaining = word_count;
  unsigned int buffer_index = 0;

  while (words_remaining > 0) {
    unsigned int words_in_page = page_size_words - offset_in_page;
    unsigned int chunk = (words_remaining < words_in_page) ? words_remaining : words_in_page;
    eeprom_write_from_page(current_page, &word_buffer[buffer_index], chunk);
    buffer_index += chunk;
    words_remaining -= chunk;
    current_page++;
    offset_in_page = 0;
  }
}

/**
 * @brief Копирование данных из EEPROM с проверкой контрольной суммы.
 *
 * Эта функция читает `size` байт из EEPROM, начиная с адреса `source`,
 * в буфер оперативной памяти `destination`. Во время чтения вычисляется
 * такая же циклическая контрольная сумма, как в `memcpy_to_eeprom_with_checksum`.
 * После чтения данных функция читает сохранённую контрольную сумму из следующего
 * байта EEPROM и сравнивает её с вычисленной.
 *
 * @param destination Указатель на буфер назначения в ОЗУ.
 * @param source Начальный адрес данных в EEPROM.
 * @param size Количество байт для чтения.
 * @return 1 (истина), если контрольная сумма совпадает, иначе 0 (ложь).
 */
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) {
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) | (checksum >> 7);
    checksum += data;
    *(destination++) = data;
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
