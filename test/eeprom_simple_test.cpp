/*
 * eeprom_simple_test.cpp - Упрощённые тесты для функций memcpy_to_eeprom_with_checksum и memcpy_from_eeprom_with_checksum
 * 
 * Этот файл содержит модульные тесты, которые проверяют логику работы функций
 * без зависимости от реального HAL. Используется симуляция EEPROM.
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <iostream>

// Константы из eeprom.cpp
#define EEPROM_PAGE_WORDS 32
#define EEPROM_PAGE_COUNT 64
#define EEPROM_OP_TIMEOUT 100000

// Симуляция EEPROM
static uint32_t simulated_eeprom[EEPROM_PAGE_COUNT][EEPROM_PAGE_WORDS] = {0};
static bool eeprom_initialized = false;

// Вспомогательные функции для симуляции
void delay_ms(uint32_t ms) {
    (void)ms;
}

// Алгоритм контрольной суммы (такой же как в eeprom.cpp)
uint8_t calculate_checksum(const char* data, unsigned int size) {
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < size; ++i) {
        uint8_t byte = static_cast<uint8_t>(data[i]);
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += byte;
    }
    return checksum;
}

// Упрощённые версии тестируемых функций
void memcpy_to_eeprom_with_checksum(unsigned int destination, unsigned int count_page, char *source, unsigned int offset, unsigned int size) {
    // Проверка на нулевой размер
    if (size == 0) return;
    if (!eeprom_initialized) return;
    if (destination >= EEPROM_PAGE_COUNT) return;
    if (count_page == 0) return;
    
    unsigned int page_size_bytes = EEPROM_PAGE_WORDS * 4;
    if (destination + count_page > EEPROM_PAGE_COUNT) return;
    if (offset >= page_size_bytes * count_page) return;
    
    // Вычисляем контрольную сумму
    uint8_t checksum = calculate_checksum(source, size);
    
    // Записываем данные в симулированную EEPROM
    uint8_t* eeprom_bytes = (uint8_t*)simulated_eeprom;
    unsigned int start_byte = destination * page_size_bytes + offset;
    
    // Копируем данные
    for (unsigned int i = 0; i < size; i++) {
        if (start_byte + i < EEPROM_PAGE_COUNT * page_size_bytes) {
            eeprom_bytes[start_byte + i] = static_cast<uint8_t>(source[i]);
        }
    }
    
    // Записываем контрольную сумму
    if (start_byte + size < EEPROM_PAGE_COUNT * page_size_bytes) {
        eeprom_bytes[start_byte + size] = checksum;
    }
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source_page, unsigned int count_page, unsigned int offset, unsigned int size) {
    if (size == 0) return 0;
    if (!eeprom_initialized) return 0;
    if (source_page >= EEPROM_PAGE_COUNT) return 0;
    if (count_page == 0) return 0;
    
    unsigned int page_size_bytes = EEPROM_PAGE_WORDS * 4;
    if (source_page + count_page > EEPROM_PAGE_COUNT) return 0;
    if (offset >= count_page * page_size_bytes) return 0;
    
    // Читаем данные из симулированной EEPROM
    uint8_t* eeprom_bytes = (uint8_t*)simulated_eeprom;
    unsigned int start_byte = source_page * page_size_bytes + offset;
    
    // Проверяем границы
    if (start_byte + size >= EEPROM_PAGE_COUNT * page_size_bytes) return 0;
    
    // Копируем данные и вычисляем контрольную сумму
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < size; i++) {
        uint8_t byte = eeprom_bytes[start_byte + i];
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += byte;
        destination[i] = static_cast<char>(byte);
    }
    
    // Читаем сохранённую контрольную сумму
    uint8_t stored_checksum = eeprom_bytes[start_byte + size];
    
    return (checksum == stored_checksum) ? 1 : 0;
}

// Вспомогательные функции для тестов
void reset_simulated_eeprom() {
    memset(simulated_eeprom, 0, sizeof(simulated_eeprom));
    eeprom_initialized = false;
}

void init_eeprom_for_test() {
    reset_simulated_eeprom();
    eeprom_initialized = true;
}

// Тест 1: Базовая запись и чтение с контрольной суммой
void test_basic_write_read() {
    std::cout << "Тест 1: Базовая запись и чтение с контрольной суммой" << std::endl;
    
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
    
    std::cout << "  ✓ Данные успешно записаны и прочитаны" << std::endl;
}

// Тест 2: Проверка контрольной суммы при повреждении данных
void test_corrupted_data() {
    std::cout << "Тест 2: Проверка контрольной суммы при повреждении данных" << std::endl;
    
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
    
    std::cout << "  ✓ Контрольная сумма обнаружила повреждение данных" << std::endl;
}

// Тест 3: Запись с offset
void test_write_with_offset() {
    std::cout << "Тест 3: Запись с offset" << std::endl;
    
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
    
    std::cout << "  ✓ Запись и чтение со смещением работают корректно" << std::endl;
}

// Тест 4: Проверка алгоритма контрольной суммы
void test_checksum_algorithm() {
    std::cout << "Тест 4: Проверка алгоритма контрольной суммы" << std::endl;
    
    // Вручную вычисляем контрольную сумму для проверки
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t expected_checksum = 0;
    
    for (unsigned int i = 0; i < sizeof(test_data); ++i) {
        expected_checksum = (expected_checksum << 1) | (expected_checksum >> 7);
        expected_checksum += test_data[i];
    }
    
    // Проверяем через нашу функцию
    uint8_t calculated = calculate_checksum((const char*)test_data, sizeof(test_data));
    assert(calculated == expected_checksum);
    
    std::cout << "  ✓ Алгоритм контрольной суммы работает корректно" << std::endl;
}

// Тест 5: Граничные случаи
void test_edge_cases() {
    std::cout << "Тест 5: Граничные случаи" << std::endl;
    
    // Тест с нулевым размером
    init_eeprom_for_test();
    
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
    
    std::cout << "  ✓ Граничные случаи обрабатываются корректно" << std::endl;
}

// Тест 6: Запись данных, занимающих несколько страниц
void test_multiple_pages() {
    std::cout << "Тест 6: Запись данных, занимающих несколько страниц" << std::endl;
    
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
    
    std::cout << "  ✓ Данные, занимающие несколько страниц, обрабатываются корректно" << std::endl;
}

int main() {
    std::cout << "Запуск упрощённых тестов для функций memcpy_to/from_eeprom_with_checksum" << std::endl;
    std::cout << "========================================================================" << std::endl;
    
    test_basic_write_read();
    test_corrupted_data();
    test_write_with_offset();
    test_checksum_algorithm();
    test_edge_cases();
    test_multiple_pages();
    
    std::cout << "\n========================================================================" << std::endl;
    std::cout << "Все тесты пройдены успешно!" << std::endl;
    
    return 0;
}