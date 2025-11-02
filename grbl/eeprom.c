// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

/* These EEPROM bits have different names on different devices. */
/* Эти биты EEPROM имеют разные названия на разных устройствах. */
#ifndef EEPE
		#define EEPE  EEWE  //!< Программа EEPROM/разрешает запись.
		#define EEMPE EEMWE //!< Основная программа EEPROM/разрешает запись.
#endif

/* К сожалению, эти два параметра не определены во включаемых файлах устройства. */
#define EEPM1 5 //!< Бит 1 режима программирования EEPROM.
#define EEPM0 4 //!< Бит 0 режима программирования EEPROM.

/* Определите, чтобы уменьшить размер кода. */
#define EEPROM_IGNORE_SELFPROG //!< Удалить опрос с флагом SPM.

/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
/*! \краткое чтение байта из EEPROM.
 *
 * Эта функция считывает один байт с заданного адреса EEPROM.
 *
 * обратите внимание, что во время чтения EEPROM центральный процессор останавливается на 4 такта.
 *
 * \параметр add - адрес EEPROM, с которого выполняется чтение.
 * \возвращает байт, считанный из адреса EEPROM.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write. // Дождитесь завершения предыдущей записи.
	EEAR = addr; // Set EEPROM address register. // Установите адресный регистр EEPROM.
	EECR = (1<<EERE); // Start EEPROM read operation. // Запустите операцию чтения EEPROM.
	return EEDR; // Return the byte read from EEPROM. // Возвращает байт, считанный из EEPROM.
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */

/*! \короткая запись байта в EEPROM.
 *
 * Эта функция записывает один байт на заданный адрес EEPROM.
 * Разница между существующим байтом и новым значением используется
 * для выбора наиболее эффективного режима программирования EEPROM.
 *
* \примечание Во время программирования EEPROM центральный процессор останавливается на 2 такта.
 *
 * \примечание. Когда эта функция возвращается, новое значение EEPROM недоступно
 * до тех пор, пока не истечет время программирования EEPROM. Бит EEPE в EECR
 * следует провести опрос, чтобы проверить, завершено ли программирование.
 *
 * \обратите внимание, что функция EEPROM_GetChar() автоматически проверяет бит EEPE.
 *
* \param add - адрес электронной памяти для записи.
 * \param new_value - Новое значение электронной памяти.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	char old_value; // Old EEPROM value. // Старое значение EEPROM.
	char diff_mask; // Difference mask, i.e. old value XOR new value. // Маска различия, т.е. старое значение XOR ИЛИ новое значение.

	cli(); // Ensure atomic operation for the write operation.// Обеспечьте атомарную операцию для операции записи. 
	
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write. // Дождитесь завершения предыдущей записи.
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM. // Дождитесь завершения SPM.
	#endif
	
	EEAR = addr; // Set EEPROM address register. // Установите адресный регистр EEPROM.
	EECR = (1<<EERE); // Start EEPROM read operation. // Запустите операцию чтения EEPROM.
	old_value = EEDR; // Get old EEPROM value. // Получить старое значение EEPROM.
	diff_mask = old_value ^ new_value; // Get bit differences. // Получите различия в битах.
	
	// Check if any bits are changed to '1' in the new value. // Проверьте, не изменены ли какие-либо биты в новом значении на "1".
	if( diff_mask & new_value ) {
		// Now we know that _some_ bits need to be erased to '1'. // Теперь мы знаем, что некоторые биты нужно стереть до '1'.
		
		// Check if any bits in the new value are '0'. // Проверьте, равны ли какие-либо биты в новом значении "0". 
		if( new_value != 0xff ) {
			// Now we know that some bits need to be programmed to '0' also. // Теперь мы знаем, что некоторые биты также должны быть запрограммированы на "0".
			
			EEDR = new_value; // Set EEPROM data register. // Установите регистр данных EEPROM.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit... // Установите бит разрешения основной записи...
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode. // ...и режим стирания+запись.
			EECR |= (1<<EEPE);  // Start Erase+Write operation. // Запустите операцию стирания+записи.
		} else {
			// Now we know that all bits should be erased. // Теперь мы знаем, что все биты должны быть стерты.

			EECR = (1<<EEMPE) | // Set Master Write Enable bit... // Установите бит разрешения основной записи...
			       (1<<EEPM0);  // ...and Erase-only mode. // ...и режим только для стирания.
			EECR |= (1<<EEPE);  // Start Erase-only operation. // Запустите операцию только для стирания.
		}
	} else {
		// Now we know that _no_ bits need to be erased to '1'.
		
		// Check if any bits are changed from '1' in the old value.

	  // Теперь мы знаем, что никакие биты не должны быть заменены на "1".
		
		// Проверьте, не изменены ли какие-либо биты с "1" в старом значении.
		if( diff_mask ) {
			// Now we know that _some_ bits need to the programmed to '0'.
			
			EEDR = new_value;   // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM1);  // ...and Write-only mode.
			EECR |= (1<<EEPE);  // Start Write-only operation.
		}
	}
	
	sei(); // Restore interrupt flag state. // Восстановить состояние флага прерывания.
}

// Extensions added as part of Grbl  // Расширения, добавленные как часть Grbl


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
