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

#include "gpio.h"

#include "debug.h"
#include "system.h"
#include "opcodes.h"
#include "peripherals.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

struct GPIO_numbers GPIO_pinmap[] = {
  // GPIOs
  { GPIOF, GPIO_PIN_8 },
  { GPIOF, GPIO_PIN_6 },
  { GPIOF, GPIO_PIN_3 },
  { GPIOF, GPIO_PIN_4 },
  { GPIOF, GPIO_PIN_12 },
  { GPIOE, GPIO_PIN_10 },
  { GPIOE, GPIO_PIN_11 },
  // ADCs
  { GPIOF, GPIO_PIN_11 },
  { GPIOA, GPIO_PIN_6 },
  { GPIOF, GPIO_PIN_13 },
  { GPIOB, GPIO_PIN_1 },
  { GPIOC, GPIO_PIN_4 },
  { GPIOF, GPIO_PIN_7 },
  { GPIOF, GPIO_PIN_9 },
  { GPIOF, GPIO_PIN_5 },
  // FDCAN1
  { GPIOD, GPIO_PIN_1 },
  { GPIOD, GPIO_PIN_0 },
  // FDCAN1
  { GPIOB, GPIO_PIN_6 },
  { GPIOB, GPIO_PIN_5 },
  // USART2
  { GPIOD, GPIO_PIN_5 },
  { GPIOD, GPIO_PIN_6 },
  { GPIOD, GPIO_PIN_4 },
  { GPIOD, GPIO_PIN_3 },
  // PWM
  { GPIOC, GPIO_PIN_7 },
  { GPIOA, GPIO_PIN_9 },
  { GPIOA, GPIO_PIN_10 },
  { GPIOB, GPIO_PIN_10 },
  { GPIOA, GPIO_PIN_11 },
  { GPIOD, GPIO_PIN_15 },
  { GPIOA, GPIO_PIN_8 },
  { GPIOC, GPIO_PIN_6 },
  { GPIOA, GPIO_PIN_12 },
  { GPIOC, GPIO_PIN_8 },
};

struct IRQ_numbers IRQ_pinmap[16];

static volatile uint16_t int_event_flags = 0;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void MX_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
}

void gpio_handle_irq() {
  uint32_t pr = EXTI->PR1;
  uint8_t index = 0;
  while (pr != 0) {
    if (pr & 0x1) {
      dbg_printf("gpio_handle_irq: index = %d (%x)\n", index, 1<<index);
      /* Set the flag variable which leads to a transmission
       * of a interrupt event within gpio_handle_data.
       */
      int_event_flags |= (1 << index);
      /* Clear interrupt flag for this specific GPIO interrupt. */
      HAL_GPIO_EXTI_IRQHandler(1 << index);
    }
    pr >>= 1;
    index++;
  }
}

void gpio_disable_irq(uint16_t pin) {
  dbg_printf("gpio_disable_irq: pin = %x\n", pin);
  if (pin == GPIO_PIN_0) {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  }
  else if (pin == GPIO_PIN_1) {
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  }
  else if (pin == GPIO_PIN_2) {
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  }
  else if (pin == GPIO_PIN_3) {
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  }
  else if (pin == GPIO_PIN_4) {
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  }
  else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  }
}

void gpio_enable_irq(uint16_t pin) {
  dbg_printf("gpio_enable_irq: pin = %x\n", pin);
  if (pin == GPIO_PIN_0) {
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  } else if (pin == GPIO_PIN_1) {
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  } else if (pin == GPIO_PIN_2) {
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  } else if (pin == GPIO_PIN_3) {
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  } else if (pin == GPIO_PIN_4) {
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  } else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
}

void gpio_set_handler(uint16_t pin)
{
  if (pin == GPIO_PIN_0) {
    NVIC_SetVector(EXTI0_IRQn, (uint32_t)&gpio_handle_irq);
  } else if (pin == GPIO_PIN_1) {
    NVIC_SetVector(EXTI1_IRQn, (uint32_t)&gpio_handle_irq);
  } else if (pin == GPIO_PIN_2) {
    NVIC_SetVector(EXTI2_IRQn, (uint32_t)&gpio_handle_irq);
  } else if (pin == GPIO_PIN_3) {
    NVIC_SetVector(EXTI3_IRQn, (uint32_t)&gpio_handle_irq);
  } else if (pin == GPIO_PIN_4) {
    NVIC_SetVector(EXTI4_IRQn, (uint32_t)&gpio_handle_irq);
  } else if (pin >= GPIO_PIN_5 && pin <= GPIO_PIN_9) {
    NVIC_SetVector(EXTI9_5_IRQn, (uint32_t)&gpio_handle_irq);
  }
}

uint8_t GPIO_PIN_to_index(uint32_t pin)
{
  uint8_t index = 0;
  while (pin >>= 1) {
    index++;
  }
  return index;
}

void gpio_init()
{
  MX_GPIO_Init();
}

void gpio_init_nirq()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

void gpio_init_ncs()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

int gpio_handle_data()
{
  int bytes_enqueued = 0;

  /* Take a threadsafe copy of the interrupt flags. */
  __disable_irq();
  uint16_t const copy_int_event_flags = int_event_flags;
  __enable_irq();

  /* We have a total of 15 external interrupts - IRQ15 is dedicated to SPI */
  for (uint8_t index = 0; index < 15; index++)
  {
    /* Check whether or not an external interrupt has occurred. */
    if (copy_int_event_flags & (1 << index))
    {
      uint8_t irq_pin = IRQ_pinmap[index].pin;
      bytes_enqueued += enqueue_packet(PERIPH_GPIO, IRQ_SIGNAL, sizeof(irq_pin), &irq_pin);
      __disable_irq();
      int_event_flags &= ~(1 << index); /*Clear this flag */
      __enable_irq();
    }
  }

  return bytes_enqueued;
}
