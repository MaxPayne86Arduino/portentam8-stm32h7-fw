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

#ifndef GPIO_H
#define GPIO_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <inttypes.h>

#include "stm32h7xx_hal.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

struct GPIO_numbers {
  GPIO_TypeDef * port;
  uint16_t pin;
};

struct IRQ_numbers {
  uint16_t pin;
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void gpio_init();

void gpio_init_nirq();
void gpio_init_ncs();

int  gpio_handle_data();

uint8_t GPIO_PIN_to_index(uint32_t pin);

void gpio_enable_irq(uint16_t pin);
void gpio_disable_irq(uint16_t pin);
void gpio_set_handler(uint16_t pin);

void gpio_handle_irq();

#endif /* GPIO_H */
