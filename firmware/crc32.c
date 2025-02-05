/*
 * Copyright (C) 2015 Johannes Taelman
 * Edited 2023 - 2024 by Ksoloti
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */

#include "crc32.h"

#include "ch.h"
#include "hal.h"
#include "exceptions.h"

static uint32_t revbit(uint32_t data) {
  uint32_t result;
  __ASM
  volatile ("rbit %0, %1" : "=r" (result) : "r" (data));
  return result;
}



// uint32_t Crc32(uint32_t Crc, const uint32_t Data)
// {
//   int i;
 
//   Crc = Crc ^ Data;
 
//   for(i=0; i<32; i++)
//     if (Crc & 0x80000000)
//       Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
//     else
//       Crc = (Crc << 1);
 
//   return(Crc);
// }

// uint32_t Crc32Block(uint32_t Crc, uint32_t WordCount, const uint32_t *Data)
// {
//   while(WordCount--)
//     Crc = Crc32(Crc, *Data++);
 
//   return(Crc);
// }

// uint32_t CalcCRC32(uint8_t *buffer, uint32_t size) {
//   return Crc32Block(0xFFFFFFFF, size >> 2, buffer);
// }

// typedef struct
// {
//   __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
//   __IO uint32_t IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
//   __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
//   uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
//   __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
//   __IO uint32_t POL;         /*!< CRC polynomial register,                     Address offset: 0x14 */
// } CRC_TypeDef;

uint32_t CalcCRC32(uint8_t *buffer, uint32_t size) {
  return 1234;
  
  uint32_t i, j;
  uint32_t ui32x;

  RCC->AHB4ENR |= RCC_AHB4ENR_CRCEN;
  
  CRC->CR = 1;
  asm("NOP");
  asm("NOP");
  asm("NOP");
  //delay for hardware ready

  i = size >> 2;

  while (i--) {
    ui32x = *((uint32_t *)buffer);
    buffer += 4;
    ui32x = revbit(ui32x); //reverse the bit order of input data
    CRC->DR = ui32x;
    if ((i && 0xFFF) == 0)
      watchdog_feed();
  }
  ui32x = CRC->DR;

  ui32x = revbit(ui32x); //reverse the bit order of output data
  i = size & 3;
  while (i--) {
    ui32x ^= (uint32_t) * buffer++;

    for (j = 0; j < 8; j++)
      if (ui32x & 1)
        ui32x = (ui32x >> 1) ^ 0xEDB88320;
      else
        ui32x >>= 1;
  }
  ui32x ^= 0xffffffff; //xor with 0xffffffff
  return ui32x; //now the output is compatible with windows/winzip/winrar
}
