/*
 * Firmware for the Portenta X8 STM32H747AIIX/Cortex-M7 core.
 * Copyright (C) 2022 Arduino (http://www.arduino.cc/)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "m4_util.h"
#include "error_handler.h"
#include "debug.h"
#include "rpc.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_rcc.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static FLASH_OBProgramInitTypeDef OBInit;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void disableCM4Autoboot() {
  OBInit.Banks     = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  dbg_printf("OBInit.USERConfig: %lX\n", OBInit.USERConfig);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    dbg_printf("Changing option bytes\n");
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    if (HAL_FLASH_OB_Unlock() == HAL_OK)
      if (HAL_FLASH_Unlock() == HAL_OK)
        if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK)
          if (HAL_FLASH_OB_Launch() == HAL_OK)
            if (HAL_FLASH_OB_Lock() == HAL_OK)
              if (HAL_FLASH_Lock() == HAL_OK)
              {
                dbg_printf("Option bytes changed\n");
                dbg_printf("Requires rebooting\n");
                NVIC_SystemReset();
                return;
              }
    dbg_printf("Failed changing option bytes");
  }
}

static int m4_booted_correctly = -1;

void try_execute_m4_app() {
  int m4_app_valid = (((*(__IO uint32_t *) FLASH_BANK2_BASE) & 0xFF000000) == 0x10000000);

  if (m4_app_valid) {
    dbg_printf("Boot CM4\n");
    serial_rpc_begin();
    LL_RCC_ForceCM4Boot();
    m4_booted_correctly = serial_rpc_ready();
    dbg_printf("CM4 booted: %d\n", m4_booted_correctly);
  }
}

int is_m4_booted_correctly()
{
  return m4_booted_correctly;
}