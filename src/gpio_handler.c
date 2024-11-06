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

#include "gpio_handler.h"

#include "stm32h7xx_hal.h"

#include "gpio.h"
#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "peripherals.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define GPIO_MODE_IN_RE         0x01   /*!< Input interrupt rising edge */
#define GPIO_MODE_IN_FE         0x02   /*!< Input interrupt falling edge */

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

extern struct GPIO_numbers GPIO_pinmap[];
extern struct IRQ_numbers IRQ_pinmap[];

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void configure_pins_shorted_together(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef* GPIO_InitStruct) {
  if (GPIO_InitStruct->Pin == GPIO_PIN_10 && GPIOx == GPIOB) {
    GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct->Pull = GPIO_NOPULL;
    GPIO_InitStruct->Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOG, GPIO_InitStruct);
  }
  if (GPIO_InitStruct->Pin == GPIO_PIN_15 && GPIOx == GPIOD) {
    GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct->Pull = GPIO_NOPULL;
    GPIO_InitStruct->Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOG, GPIO_InitStruct);
  }
}

static void write_pins_shorted_together(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
  if (GPIO_Pin == GPIO_PIN_10 && GPIOx == GPIOB) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, PinState);
  }
  if (GPIO_Pin == GPIO_PIN_15 && GPIOx == GPIOD) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, PinState);
  }
}

int gpio_handler(uint8_t const opcode, uint8_t const * data, uint16_t const size)
{
  uint16_t const gpio_data = *((uint16_t*)data);
  enum Opcodes_GPIO const action = opcode;

  uint8_t const value = (gpio_data & 0xFF00) >> 8;
  uint8_t const index = gpio_data & 0xFF;

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t response[2];

  switch (action) {
    case CONFIGURE:
      GPIO_InitStruct.Pin = GPIO_pinmap[index].pin;
      GPIO_InitStruct.Mode = value;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_pinmap[index].port, &GPIO_InitStruct);
      configure_pins_shorted_together(GPIO_pinmap[index].port, &GPIO_InitStruct);
      dbg_printf("GPIO%d: CONFIGURE %d\n", index, value);
      break;
    case IRQ_TYPE:
      GPIO_InitStruct.Pin = GPIO_pinmap[index].pin;

      if (GPIO_InitStruct.Pin == GPIO_PIN_15) {
        // IRQ15 is used for SPI
        return 0;
      }

      if      (value == GPIO_MODE_IN_RE) GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      else if (value == GPIO_MODE_IN_FE) GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      else if (value == (GPIO_MODE_IN_FE | GPIO_MODE_IN_RE)) GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
      else                               return 0;

      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(GPIO_pinmap[index].port, &GPIO_InitStruct);
      IRQ_pinmap[GPIO_PIN_to_index(GPIO_InitStruct.Pin)].pin = index;
      dbg_printf("GPIO%d: IRQ_TYPE %d\n", index, value);
      break;
    case IRQ_ENABLE:
      dbg_printf("GPIO%d: IRQ_ENABLE %d\n", index, value);
      if (value == 1) {
        gpio_set_handler(GPIO_pinmap[index].pin);
        gpio_enable_irq(GPIO_pinmap[index].pin);
      } else {
        gpio_disable_irq(GPIO_pinmap[index].pin);
      }
      break;
    case WRITE:
      HAL_GPIO_WritePin(GPIO_pinmap[index].port, GPIO_pinmap[index].pin, value);
      write_pins_shorted_together(GPIO_pinmap[index].port, GPIO_pinmap[index].pin, value);
      dbg_printf("GPIO%d: WRITE %d\n", index, value);
      break;
    case READ:
      response[0] = index;
      response[1] = HAL_GPIO_ReadPin(GPIO_pinmap[index].port, GPIO_pinmap[index].pin);
      dbg_printf("GPIO%d: READ %d\n", index, response[1]);
      return enqueue_packet(PERIPH_GPIO, opcode, sizeof(response), &response);
      break;
    case IRQ_SIGNAL:
      // do nothing;
      dbg_printf("GPIO%d: IRQ_SIGNAL %d\n", index, value);
      break;
    case IRQ_ACK:
      dbg_printf("GPIO%d: IRQ_ACK %d\n", index, value);
      /* Re-enable the interrupt that was disabled within
       * gpio_handle_irq to prevent firing of another interrupt
       * until this one has been signaled to the application.
       */
      gpio_enable_irq(GPIO_pinmap[index].pin);
      break;
  }
  return 0;
}
