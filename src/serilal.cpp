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

void serial_init(){
      UART_Init(UART_0, F_CPU/BAUD_RATE, UART_CONTROL1_TE_M | UART_CONTROL1_RE_M  | UART_CONTROL1_M_8BIT_M, 0, 0);
}

// Writes one byte to the TX serial buffer. Called by main program. // Записывает один байт в последовательный буфер TX. Вызывается основной программой.
void serial_write(u8 data){
	UART_WriteByte(UART_0, data);
	UART_WaitTransmission(UART_0);
}

// Fetches the first byte in the serial read buffer. Called by main program. // Извлекает первый байт из буфера последовательного чтения. Вызывается основной программой.
u16 serial_read(){
  return UART_ReadByte(UART_0);
}

// Reset and empty data in read buffer. Used by e-stop and reset. // Сброс и очистка данных в буфере чтения. Используется e-stop и reset.
void serial_reset_read_buffer(){
  UART_ClearRxFifo(UART_0);
}

// Returns the number of bytes used in the RX serial buffer. 
// Возвращает количество байт, используемых в последовательном буфере RX.
u8 serial_get_rx_buffer_count(){
 return 0; // Заглушка
}

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
// Возвращает количество байт, используемых в последовательном буфере TX.
// ПРИМЕЧАНИЕ: Используется только для отладки и обеспечения отсутствия узких мест в TX.
u8 serial_get_tx_buffer_count(){
  return 0; // Заглушка
}