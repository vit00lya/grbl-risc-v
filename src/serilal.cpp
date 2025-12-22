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
#include "serial.h"

Serial::Serial()
{
  UART_Init(UART_0, F_CPU/BAUD_RATE, UART_CONTROL1_TE_M | UART_CONTROL1_RE_M  | UART_CONTROL1_M_8BIT_M, 0, 0);
}

  void Serial::Write(uint8_t data){
    	UART_WriteByte(UART_0, data);
	    UART_WaitTransmission(UART_0);
  }
  uint16_t Serial::Read(){
    return UART_ReadByte(UART_0);
  }
  void Serial::SerialResetReadBuffer(){
      UART_ClearRxFifo(UART_0);
  }
  uint8_t Serial::SerialGetRxBufferCount(){
     return 0; // Заглушка
  }
  uint8_t Serial::SerialGetTxBufferCount(){
     return 0; // Заглушка
  }
