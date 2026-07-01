/*
  main.c - Встраиваемый CNC контроллер с поддержкой rs274/ngc (G-код)
  Part of Grbl


  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// #include <Arduino.h>
#include <grbl.hpp>

static char line[LINE_BUFFER_SIZE]; // Строка для выполнения. Завершается нулём.
uint8_t buf_pointer;                // Указатель на текущий элемент буфера
uint8_t line_flags = 0;

// Объявление глобальной структуры системных переменных
system_t sys;
int32_t sys_position[N_AXIS];                    // Вектор позиции машины (также домашней позиции) в шагах в реальном времени.
int32_t sys_probe_position[N_AXIS];              // Последняя позиция датчика в координатах машины и шагах.
volatile uint8_t sys_probe_state;                // Состояние зондирования. Используется для координации цикла зондирования с прерыванием шагового двигателя.
volatile uint8_t sys_rt_exec_state;              // Глобальная переменная-битовый флаг исполнителя реального времени для управления состоянием. См. битовые маски EXEC.
volatile uint8_t sys_rt_exec_alarm;              // Глобальная переменная-битовый флаг исполнителя реального времени для установки различных аварийных сигналов.
volatile uint8_t sys_rt_exec_motion_override;    // Глобальная переменная-битовый флаг исполнителя реального времени для переопределений, связанных с движением.
volatile uint8_t sys_rt_exec_accessory_override; // Глобальная переменная-битовый флаг исполнителя реального времени для переопределений шпинделя/охлаждения.
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif

void setup()
{

  // Инициализация системы при включении питания.
  serial_init();   // Настройка последовательного соединения
  eeprom_init();   // Инициализация EEPROM
  settings_init(); // Загрузка настроек Grbl из EEPROM
  stepper_init();  // Конфигурация выводов шагового двигателя и таймеров прерываний
  system_init();   // Конфигурация выводов и прерываний по изменению состояния выводов

  memset(sys_position, 0, sizeof(sys_position)); // Очистка позиции машины.
// sei(); // Включение прерываний

// Инициализация состояния системы.
#ifdef FORCE_INITIALIZATION_ALARM
                                                 // Принудительный перевод Grbl в состояние ALARM при перезагрузке питания или аппаратном сбросе.
  sys.state = STATE_ALARM;
#else
  sys.state = STATE_IDLE;
#endif

// Проверка включения питания и установка системной тревоги, если включена функция поиска дома,
// чтобы принудительно выполнить цикл поиска дома путём установки состояния тревоги Grbl.
// Тревога блокирует все команды G-кода, включая стартовые скрипты, но разрешает доступ
// к настройкам и внутренним командам. Только цикл поиска дома '$H' или снятие блокировки тревоги '$X'
// отключат тревогу.
// ПРИМЕЧАНИЕ: Стартовый скрипт выполнится после успешного завершения цикла поиска дома,
// но не после снятия блокировки тревоги. Это предотвращает неконтролируемые столкновения
// стартовых блоков движения с объектами. Очень важно.
#ifdef HOMING_INIT_LOCK
  if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE))
  {
    sys.state = STATE_ALARM;
  }
#endif

#ifdef ENABLE_WIFI
  wifi_init();
#endif
}

// Обработчик прерываний
extern "C"
{
  void trap_handler()
  {
    // if (EPIC_CHECK_TIMER16_1())
    // {
    //      if (__HAL_TIMER16_GET_FLAG_IT(&timer_step, TIMER16_FLAG_CMPM))
    //      {
    //        StepTimer();
    //         __HAL_TIMER16_CLEAR_FLAG(&timer_step, TIMER16_FLAG_CMPM);
    //      }
    // }

    if (EPIC_CHECK_GPIO_IRQ())
    {
      pin_limit_vect();
    }

    if (EPIC_CHECK_UART_0())
    {
      /* Прием данных: запись в буфер */
      if (HAL_USART_RXNE_ReadFlag(UART_0))
      {
        line[buf_pointer] = serial_read(CLIENT_SERIAL);
        if (!(line_flags & LINE_FLAG_LINE_READ))
        {
          // Отбросить все символы комментариев (кроме EOL) и символы переполнения.
          /* Если принят символ '\n' или '\r', заменить его на '\0' (символ конца строки) */
          if (line[buf_pointer] == '\n' || line[buf_pointer] == '\r')
          {
            line[buf_pointer] = '\0';
            buf_pointer = 0;
            line_flags &= ~(LINE_FLAG_LINE_READ);
          }
          else
          {
            if (line[buf_pointer] == ')')
            {
              // Конец комментария '()'. Обработка строки разрешена.
              if (line_flags & LINE_FLAG_COMMENT_PARENTHESES)
              {
                line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES);
              }
            }
            else
            {
              if (line[buf_pointer] <= ' ')
              {
                // Отбросить пробельные и управляющие символы
              }
              else if (line[buf_pointer] == '/')
              {
                // Пропуск блока НЕ ПОДДЕРЖИВАЕТСЯ. Игнорировать символ.
                // ПРИМЕЧАНИЕ: Если бы поддерживалось, нужно было бы проверить системный флаг block delete.
              }
              else if (line[buf_pointer] == '(')
              {
                // Включить флаг комментария и игнорировать все символы до ')' или конца строки.
                // ПРИМЕЧАНИЕ: Это не совсем соответствует определению NIST, но пока достаточно.
                // В будущем можно просто удалять содержимое комментариев, но сохранять
                // управляющие символы комментариев, чтобы парсер g-code мог проверять их на ошибки.
                line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
              }
              else if (line[buf_pointer] == ';')
              {
                // ПРИМЕЧАНИЕ: Комментарий ';' до конца строки — это определение LinuxCNC, не NIST.
                line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
                // TODO: Реализовать функционал '%'
                // } else if (c == '%') {
                // Символ процента начала/конца программы НЕ ПОДДЕРЖИВАЕТСЯ.
                // ПРИМЕЧАНИЕ: Может быть реализован, чтобы сообщать Grbl, когда запущена программа,
                // а когда — ручной ввод. Во время программы автозапуск цикла будет выполняться
                // до следующего символа '%'. Это поможет исправить проблемы возобновления
                // с некоторыми функциями, которые очищают буфер планировщика.
              }
              else if (buf_pointer >= (LINE_BUFFER_SIZE - 1))
              {
                // Обнаружено переполнение буфера строки — установить флаг.
                line_flags |= LINE_FLAG_OVERFLOW;
              }
              else if (line[buf_pointer] >= 'a' && line[buf_pointer] <= 'z')
              { // Привести к верхнему регистру
                line[buf_pointer] = line[buf_pointer] - 'a' + 'A';
              }
              else
              {
                line[buf_pointer] = line[buf_pointer];
              }
            }
            buf_pointer += 1;
            if (buf_pointer >= LINE_BUFFER_SIZE)
              buf_pointer = 0;
          }
          HAL_USART_RXNE_ClearFlag(UART_0);
        }
      }

      // if (EPIC_CHECK_TIMER16_1())
      // {

      //      if (__HAL_TIMER16_GET_FLAG_IT(&timer_step, TIMER16_FLAG_CMPM))
      //      {
      //         // HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7); /* Смена сигнала PORT1_3 на противоположный */
      //         __HAL_TIMER16_CLEAR_FLAG(&timer_step, TIMER16_FLAG_CMPM);
      //         // HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_HIGH);
      //         // HAL_DelayMs(10);
      //         // HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_LOW);
      //         // HAL_DelayMs(1000);
      //         // HAL_Timer16_StartSetOnes_IT(timer_step, 0xFFFF, 0xFFFF / 2);
      //      }

      // }

      //   /* Сброс прерываний */
      //   // Денис рекомендовал следующую последовательность, сбросить флаг прерывания, затем его обрабатывать.
      //   // Чтобы было меньше багов
      HAL_EPIC_Clear(0xFFFFFFFF);
      // }
    }
  }
}

  int main()
  {

    // Тестирование EEPROM
    // char line[LINE_BUFFER_SIZE] = "3388HELLO";
    // char line_result[LINE_BUFFER_SIZE] = "";
    // eeprom_init();
    // memcpy_to_eeprom_with_checksum(0, 1, (char*)line, 0, 10);
    // memcpy_from_eeprom_with_checksum((char*)line_result, 0, 1, 0, 10);

    // Цикл инициализации Grbl при включении питания или аварийной остановке системы. В последнем случае все процессы
    // вернутся в этот цикл для чистой повторной инициализации.
    setup();
    // Сброс системных переменных.
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Очистка структурной переменной системы.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;                    // Установить на 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE;                   // Установить на 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;    // Установить на 100%
    memset(sys_probe_position, 0, sizeof(sys_probe_position)); // Очистка позиции датчика.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Сброс основных систем Grbl.
    serial_reset_read_buffer(CLIENT_ALL); // Очистка буфера чтения последовательного порта
    gc_init();                            // Установка парсера G-кода в состояние по умолчанию
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset(); // Очистка буфера блоков и переменных планировщика
    st_reset();   // Очистка переменных подсистемы шагового двигателя.

    // Синхронизация очищенных позиций G-кода и планировщика с текущей системной позицией.
    plan_sync_position();
    gc_sync_position();

    // Вывод приветственного сообщения. Указывает, что произошла инициализация при включении питания или сбросе.
    report_init_message(CLIENT_ALL);

    // Запуск основного цикла Grbl. Обрабатывает входные данные программы и выполняет их.
    protocol_main_loop(line, &line_flags);
    return 0;
  }
