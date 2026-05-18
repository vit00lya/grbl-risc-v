/*
 * eeprom_test.cpp - Тесты для функций memcpy_to_eeprom_with_checksum и memcpy_from_eeprom_with_checksum
 * 
 * Этот файл содержит модульные тесты для проверки корректности работы функций
 * работы с EEPROM с контрольными суммами.
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cassert>

// Заглушки для HAL функций - объявляем структуры и функции самостоятельно
extern "C" {
    typedef struct {
        void* Instance;
        uint32_t Mode;
        uint32_t ErrorCorrection;
        uint32_t EnableInterrupt;
    } HAL_EEPROM_HandleTypeDef;
    
    typedef enum {
        HAL_OK       = 0x00U,
        HAL_ERROR    = 0x01U,
        HAL_BUSY     = 0x02U,
        HAL_TIMEOUT  = 0x03U
    } HAL_StatusTypeDef;
    
    typedef enum {
        HAL_EEPROM_MODE_TWO_STAGE = 0,
        HAL_EEPROM_MODE_SINGLE_STAGE
    } HAL_EEPROM_ModeTypeDef;
    
    typedef enum {
        HAL_EEPROM_ECC_ENABLE = 0,
        HAL_EEPROM_ECC_DISABLE
    } HAL_EEPROM_ECCTypeDef;
    
    typedef enum {
        HAL_EEPROM_SERR_ENABLE = 0,
        HAL_EEPROM_SERR_DISABLE
    } HAL_EEPROM_SERRTypeDef;
    
    typedef enum {
        HAL_EEPROM_WRITE_SINGLE = 0,
        HAL_EEPROM_WRITE_BURST
    } HAL_EEPROM_WriteModeTypeDef;
    
    // Объявления функций
    HAL_StatusTypeDef HAL_EEPROM_Init(HAL_EEPROM_HandleTypeDef *heeprom);
    HAL_StatusTypeDef HAL_EEPROM_CalculateTimings(HAL_EEPROM_HandleTypeDef *heeprom, uint32_t clock_freq);
    HAL_StatusTypeDef HAL_EEPROM_Erase(HAL_EEPROM_HandleTypeDef *heeprom, uint8_t start_page, uint8_t word_count, uint32_t write_mode, uint32_t timeout);
    HAL_StatusTypeDef HAL_EEPROM_Write(HAL_EEPROM_HandleTypeDef *heeprom, uint16_t address, uint32_t *data, uint8_t word_count, uint32_t write_mode, uint32_t timeout);
    HAL_StatusTypeDef HAL_EEPROM_Read(HAL_EEPROM_HandleTypeDef *heeprom, uint16_t address, uint32_t *data, uint8_t word_count, uint32_t timeout);
}

// Константы из eeprom.cpp
#define EEPROM_PAGE_WORDS 32
#define EEPROM_PAGE_COUNT 64
#define EEPROM_OP_TIMEOUT 100000

// Глобальные переменные для симуляции EEPROM
static uint32_t simulated_eeprom[EEPROM_PAGE_COUNT][EEPROM_PAGE_WORDS] = {0};
static bool eeprom_initialized = false;

// Заглушки HAL функций
HAL_StatusTypeDef HAL_EEPROM_Init(HAL_EEPROM_HandleTypeDef *heeprom) {
    eeprom_initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EEPROM_CalculateTimings(HAL_EEPROM_HandleTypeDef *heeprom, uint32_t clock_freq) {
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EEPROM_Erase(HAL_EEPROM_HandleTypeDef *heeprom, uint8_t start_page, uint8_t word_count, uint32_t write_mode, uint32_t timeout) {
    if (!eeprom_initialized) return HAL_ERROR;
    if (start_page >= EEPROM_PAGE_COUNT) return HAL_ERROR;
    
    for (uint8_t page = 0; page < word_count && (start_page + page) < EEPROM_PAGE_COUNT; page++) {
        for (int i = 0; i < EEPROM_PAGE_WORDS; i++) {
            simulated_eeprom[start_page + page][i] = 0xFFFFFFFF;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EEPROM_Write(HAL_EEPROM_HandleTypeDef *heeprom, uint16_t address, uint32_t *data, uint8_t word_count, uint32_t write_mode, uint32_t timeout) {
    if (!eeprom_initialized) return HAL_ERROR;
    
    uint8_t start_page = address / (EEPROM_PAGE_WORDS * 4);
    uint16_t offset_words = (address % (EEPROM_PAGE_WORDS * 4)) / 4;
    
    for (uint8_t i = 0; i < word_count; i++) {
        uint8_t page = start_page + ((offset_words + i) / EEPROM_PAGE_WORDS);
        uint8_t word_in_page = (offset_words + i) % EEPROM_PAGE_WORDS;
        
        if (page >= EEPROM_PAGE_COUNT) return HAL_ERROR;
        
        simulated_eeprom[page][word_in_page] = data[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EEPROM_Read(HAL_EEPROM_HandleTypeDef *heeprom, uint16_t address, uint32_t *data, uint8_t word_count, uint32_t timeout) {
    if (!eeprom_initialized) return HAL_ERROR;
    
    uint8_t start_page = address / (EEPROM_PAGE_WORDS * 4);
    uint16_t offset_words = (address % (EEPROM_PAGE_WORDS * 4)) / 4;
    
    for (uint8_t i = 0; i < word_count; i++) {
        uint8_t page = start_page + ((offset_words + i) / EEPROM_PAGE_WORDS);
        uint8_t word_in_page = (offset_words + i) % EEPROM_PAGE_WORDS;
        
        if (page >= EEPROM_PAGE_COUNT) return HAL_ERROR;
        
        data[i] = simulated_eeprom[page][word_in_page];
    }
    return HAL_OK;
}

// Заглушка для delay_ms
void delay_ms(uint32_t ms) {
    // Ничего не делаем в тестах
    (void)ms;
}

// Включаем реальные функции из eeprom.cpp
#include "../lib/grbl/src/eeprom.cpp"

// Вспомогательные функции для тестов
void reset_simulated_eeprom() {
    memset(simulated_eeprom, 0, sizeof(simulated_eeprom));
    eeprom_initialized = false;
}

void init_eeprom_for_test() {
    reset_simulated_eeprom();
    eeprom_init();
}

// Тест 1: Базовая запись и чтение с контрольной суммой
void test_basic_write_read() {
    printf("Тест 1: Базовая запись и чтение с контрольной суммой\n");
    
    init_eeprom_for_test();
    
    char source_data[] = "Hello, EEPROM!";
    char dest_buffer[50] = {0};
    unsigned int data_size = strlen(source_data);
    
    // Записываем данные в EEPROM
    memcpy_to_eeprom_with_checksum(0, 2, source_data, 0, data_size);
    
    // Читаем данные обратно
    int result = memcpy_from_eeprom_with_checksum(dest_buffer, 0, 2, 0, data_size);
    
    // Проверяем результат
    assert(result == 1); // Контрольная сумма должна совпадать
    assert(strcmp(source_data, dest_buffer) == 0); // Данные должны быть идентичны
    
    printf("  ✓ Данные успешно записаны и прочитаны\n");
}

// Тест 2: Проверка контрольной суммы при повреждении данных
void test_corrupted_data() {
    printf("Тест 2: Проверка контрольной суммы при повреждении данных\n");
    
    init_eeprom_for_test();
    
    char source_data[] = "Test data for checksum";
    char dest_buffer[50] = {0};
    unsigned int data_size = strlen(source_data);
    
    // Записываем данные
    memcpy_to_eeprom_with_checksum(0, 2, source_data, 0, data_size);
    
    // Повреждаем один байт в симулированной EEPROM
    uint8_t* eeprom_bytes = (uint8_t*)simulated_eeprom;
    eeprom_bytes[5] ^= 0xFF; // Инвертируем байт
    
    // Пытаемся прочитать
    int result = memcpy_from_eeprom_with_checksum(dest_buffer, 0, 2, 0, data_size);
    
    // Контрольная сумма не должна совпадать
    assert(result == 0);
    
    printf("  ✓ Контрольная сумма обнаружила повреждение данных\n");
}

// Тест 3: Запись с offset
void test_write_with_offset() {
    printf("Тест 3: Запись с offset\n");
    
    init_eeprom_for_test();
    
    char source_data[] = "OffsetTest";
    char dest_buffer[50] = {0};
    unsigned int data_size = strlen(source_data);
    unsigned int offset = 10; // Смещение 10 байт
    
    // Записываем данные со смещением
    memcpy_to_eeprom_with_checksum(0, 2, source_data, offset, data_size);
    
    // Читаем данные с тем же смещением
    int result = memcpy_from_eeprom_with_checksum(dest_buffer, 0, 2, offset, data_size);
    
    assert(result == 1);
    assert(strcmp(source_data, dest_buffer) == 0);
    
    printf("  ✓ Запись и чтение со смещением работают корректно\n");
}

// Тест 4: Запись данных, занимающих несколько страниц
void test_multiple_pages() {
    printf("Тест 4: Запись данных, занимающих несколько страниц\n");
    
    init_eeprom_for_test();
    
    // Создаем данные размером больше одной страницы
    unsigned int page_size_bytes = EEPROM_PAGE_WORDS * 4;
    unsigned int data_size = page_size_bytes * 2 - 5; // Немного меньше двух полных страниц
    
    char* source_data = new char[data_size];
    char* dest_buffer = new char[data_size];
    
    // Заполняем тестовыми данными
    for (unsigned int i = 0; i < data_size; i++) {
        source_data[i] = 'A' + (i % 26);
    }
    
    // Записываем данные, начиная со страницы 1
    memcpy_to_eeprom_with_checksum(1, 3, source_data, 0, data_size);
    
    // Читаем данные
    int result = memcpy_from_eeprom_with_checksum(dest_buffer, 1, 3, 0, data_size);
    
    assert(result == 1);
    assert(memcmp(source_data, dest_buffer, data_size) == 0);
    
    delete[] source_data;
    delete[] dest_buffer;
    
    printf("  ✓ Данные, занимающие несколько страниц, обрабатываются корректно\n");
}

// Тест 5: Граничные случаи (нулевой размер, неинициализированная EEPROM)
void test_edge_cases() {
    printf("Тест 5: Граничные случаи\n");
    
    // Тест с нулевым размером
    reset_simulated_eeprom();
    eeprom_init();
    
    char dummy_data[] = "Dummy";
    char dummy_buffer[10] = {0};
    
    // Нулевой размер - функция должна корректно обработать
    memcpy_to_eeprom_with_checksum(0, 1, dummy_data, 0, 0);
    int result = memcpy_from_eeprom_with_checksum(dummy_buffer, 0, 1, 0, 0);
    assert(result == 0); // При нулевом размере возвращает 0
    
    // Неинициализированная EEPROM
    reset_simulated_eeprom(); // eeprom_initialized = false
    memcpy_to_eeprom_with_checksum(0, 1, dummy_data, 0, 5);
    result = memcpy_from_eeprom_with_checksum(dummy_buffer, 0, 1, 0, 5);
    assert(result == 0); // Должен вернуть 0 при неинициализированной EEPROM
    
    // Некорректный номер страницы
    init_eeprom_for_test();
    memcpy_to_eeprom_with_checksum(EEPROM_PAGE_COUNT, 1, dummy_data, 0, 5); // Страница за пределами
    result = memcpy_from_eeprom_with_checksum(dummy_buffer, EEPROM_PAGE_COUNT, 1, 0, 5);
    assert(result == 0);
    
    printf("  ✓ Граничные случаи обрабатываются корректно\n");
}

// Тест 6: Проверка алгоритма контрольной суммы
void test_checksum_algorithm() {
    printf("Тест 6: Проверка алгоритма контрольной суммы\n");
    
    // Вручную вычисляем контрольную сумму для проверки
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t expected_checksum = 0;
    
    for (unsigned int i = 0; i < sizeof(test_data); ++i) {
        expected_checksum = (expected_checksum << 1) | (expected_checksum >> 7);
        expected_checksum += test_data[i];
    }
    
    // Проверяем через реальные функции
    init_eeprom_for_test();
    
    memcpy_to_eeprom_with_checksum(0, 1, (char*)test_data, 0, sizeof(test_data));
    
    // Читаем контрольную сумму из симулированной EEPROM
    uint8_t* eeprom_bytes = (uint8_t*)simulated_eeprom;
    uint8_t stored_checksum = eeprom_bytes[sizeof(test_data)]; // Контрольная сумма после данных
    
    assert(stored_checksum == expected_checksum);
    
    printf("  ✓ Алгоритм контрольной суммы работает корректно\n");
}

int main() {
    printf("Запуск тестов для функций memcpy_to/from_eeprom_with_checksum\n");
    printf("=============================================================\n");
    
    test_basic_write_read();
    test_corrupted_data();
    test_write_with_offset();
    test_multiple_pages();
    test_edge_cases();
    test_checksum_algorithm();
    
    printf("\n=============================================================\n");
    printf("Все тесты пройдены успешно!\n");
    
    return 0;
}