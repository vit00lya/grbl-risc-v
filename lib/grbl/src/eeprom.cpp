// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
* EEPROM implementation for MIK32 platform (ELRON_ACE_UNO)
******************************************************************************/

#include <mik32_hal_eeprom.h>
#include <mik32_memory_map.h>
#include "grbl.hpp"

// EEPROM handle
static HAL_EEPROM_HandleTypeDef heeprom;
static bool eeprom_initialized = false;

/*! \brief  Initialize EEPROM.
 *
 *  This function Initializes the EEPROM buffer.
 *
 */
void eeprom_init()
{
	// Initialize EEPROM handle for MIK32
	heeprom.Instance = EEPROM_REGS;
	heeprom.Mode = HAL_EEPROM_MODE_TWO_STAGE;
	heeprom.ErrorCorrection = HAL_EEPROM_ECC_ENABLE;
	heeprom.EnableInterrupt = HAL_EEPROM_SERR_DISABLE;
	
	HAL_EEPROM_Init(&heeprom);
	HAL_EEPROM_CalculateTimings(&heeprom, 32000000); // 32MHz system clock
	eeprom_initialized = true;
}

/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	if (!eeprom_initialized) {
		return 0;
	}
	
	uint32_t data;
	if (HAL_EEPROM_Read(&heeprom, addr, &data, 1, 100000) == HAL_OK) {
		return (unsigned char)data;
	}
	return 0; // Return 0 on error
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	if (!eeprom_initialized) {
		return;
	}
	
	uint32_t data = (uint32_t)new_value;
	HAL_EEPROM_Write(&heeprom, addr, &data, 1, HAL_EEPROM_WRITE_ALL, 100000);
}

// Extensions added as part of Grbl


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
