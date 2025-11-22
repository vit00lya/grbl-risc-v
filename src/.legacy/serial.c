/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
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

#include "grbl.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable // Переменная состояния управления потоком
#endif
  
// Возвращает количество байт, используемых в последовательном буфере RX.
// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile // Копировать, чтобы ограничить количество обращений к volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
// Возвращает количество байт, используемых в буфере последовательной передачи данных.
// ПРИМЕЧАНИЕ: Используется только для отладки и обеспечения отсутствия узких мест в системе передачи данных.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // Set baud rate // Установить скорость передачи данных в бодах
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200 // включен удвоитель скорости передачи данных в бодах для обеспечения высокой скорости передачи данных в бодах, т.е. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx // включить приемопередачу и tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte // включить прерывание при завершении приема байта
  UCSR0B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit // по умолчанию 8-разрядный, без четности, 1 стоп-бит
}

// Записывает один байт в последовательный буфер TX. Вызывается основной программой.
// ЗАДАЧА: Проверьте, можем ли мы ускорить это для записи строк, а не отдельных байтов.
// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data) {
  // Вычислить следующий напор
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  // Подождите, пока в буфере не освободится место
  while (next_head == serial_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.  
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop. // Проверяйте только прерывание, чтобы избежать бесконечного цикла.
  }

  // Store data and advance head
  // Хранение данных и предварительная загрузка
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  // Включить прерывание регистрации пустых данных, чтобы убедиться, что tx-streaming запущен
  UCSR0B |=  (1 << UDRIE0); 
}


// Data Register Empty Interrupt handler
// Обработчик пустых прерываний регистра данных
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile) // Временный serial_tx_buffer_tail (для оптимизации с учетом изменчивости)
  
  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) { 
      UDR0 = XOFF_CHAR; 
      flow_ctrl = XOFF_SENT; 
    } else if (flow_ctrl == SEND_XON) { 
      UDR0 = XON_CHAR; 
      flow_ctrl = XON_SENT; 
    } else
  #endif
  { 
    // Send a byte from the buffer // Отправить байт из буфера
    UDR0 = serial_tx_buffer[tail];
  
    // Update tail position // Обновить положение хвоста
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }
  
    serial_tx_buffer_tail = tail;
  }
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer 
  // Отключите прерывание пустого регистра данных, чтобы остановить потоковую передачу данных, если это завершит передачу
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}


// Fetches the first byte in the serial read buffer. Called by main program.
// Извлекает первый байт из буфера последовательного чтения. Вызывается основной программой.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile) // Временный serial_rx_buffer_tail (для оптимизации с учетом изменчивости)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX
      }
    #endif
    
    return data;
  }
}


ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
  // Извлекает символы команд реального времени непосредственно из последовательного потока. Эти символы
  // не передаются в буфер, но они устанавливают биты флага состояния системы для выполнения в реальном времени.
  switch (data) {
    case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Set as true // Установить значение true
    case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
    case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine. // Вызовите процедуру сброса управления движением.
    default: // Write character to buffer // Записать символ в буфер     
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full. // Записывайте данные в буфер, если он не заполнен.
      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;    
        
        #ifdef ENABLE_XONXOFF
          if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR0B |=  (1 << UDRIE0); // Force TX
          } 
        #endif
        
      } 
      //TODO: else alarm on overflow? //TODO: еще один сигнал тревоги при переполнении?
  } 
}


void SerialResetReadBuffer() 
{
  serial_rx_buffer_tail = serial_rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
