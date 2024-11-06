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

#include "system.h"
#include "error_handler.h"
#include "debug.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "rpc.h"
#include "spi.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer_1[SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer_2[SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer          [SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer_userspace[SPI_DMA_BUFFER_SIZE];

typedef enum
{
  Idle, Header, Data, Complete, Error
} eTransferState;
volatile eTransferState transaction_state = Idle;

volatile bool is_rx_buf_userspace_processed = false;

volatile uint8_t * p_tx_buf_active   = TX_Buffer_1;
volatile uint8_t * p_tx_buf_transfer = TX_Buffer_2;
volatile struct subpacket * rx_pkt_userspace = (struct subpacket *)RX_Buffer_userspace;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void SystemClock_Config(void) {

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler("HAL_RCC_OscConfig failed.");
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler("HAL_RCC_ClockConfig failed.");
  }

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

static void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 128;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler("HAL_RCCEx_PeriphCLKConfig failed.");
  }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Do MPU */
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer_1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer_2;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer_userspace;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void clean_dma_buffer()
{
  memset((uint8_t*)TX_Buffer_1, 0, sizeof(TX_Buffer_1));
  memset((uint8_t*)TX_Buffer_2, 0, sizeof(TX_Buffer_2));
  memset((uint8_t*)RX_Buffer, 0, sizeof(RX_Buffer));
  memset((uint8_t*)RX_Buffer_userspace, 0, sizeof(RX_Buffer_userspace));

  struct complete_packet * pkt = (struct complete_packet *)TX_Buffer_1;
  pkt->header.size = 0;
  pkt->header.checksum = pkt->header.size ^ 0x5555;

  pkt = (struct complete_packet *)TX_Buffer_2;
  pkt->header.size = 0;
  pkt->header.checksum = pkt->header.size ^ 0x5555;
}

int get_available_enqueue() {
  struct complete_packet * pkt = (struct complete_packet *)p_tx_buf_active;
  return (SPI_DMA_BUFFER_SIZE - 4) - pkt->header.size;
}

int enqueue_packet(uint8_t const peripheral, uint8_t const opcode, uint16_t const size, void * data)
{
  /* Enter critical section: Since this function is called both from inside
   * interrupt context (handle_irq/gpio.c) as well as from normal execution
   * context it is necessary not only to blindly re-enable interrupts, but
   * to store the current interrupt situation and restore it at the end of
   * the critical section.
   */
  volatile uint32_t primask_bit = __get_PRIMASK();
  /*
   * PRIMASK - Typically configured in code using the CMSIS __disable_irq() and
   * __enable_irq() routines or the cpsid i and cpsie i assembly instructions
   * directly. Setting the PRIMASK to 1 disables all exceptions of configurable
   * priority. This means, only NMI, Hardfault, & Reset exceptions can still occur.
   *
   * Source: https://interrupt.memfault.com/blog/arm-cortex-m-exceptions-and-nvic
   */
  __set_PRIMASK(1) ;

  int bytes_enqueued = 0;

  /* complete_packet:
   * - uint16_t size;      |
   * - uint16_t checksum;  | sizeof(complete_packet.header) = 4 Bytes
   */
  struct complete_packet * pkt = (struct complete_packet *)p_tx_buf_active;
  if ((pkt->header.size + size) > (SPI_DMA_BUFFER_SIZE - 4))
    goto cleanup;

  /* subpacket:
   * - uint8_t peripheral; |
   * - uint8_t opcode;     |
   * - uint16_t size;      | sizeof(subpacket.header) = 4 Bytes
   * - uint8_t raw_data;
   */
  struct subpacket subpkt;
  subpkt.header.peripheral = peripheral;
  subpkt.header.opcode = opcode;
  subpkt.header.size = size;
  /* Copy subpacket.header at the end of the current complete_packet superframe. */
  memcpy((uint8_t*)&(pkt->data) + pkt->header.size, &subpkt, sizeof(subpkt.header));
  pkt->header.size += sizeof(subpkt.header);
  /* Copy subpacket.raw_data at after subpacket.header. */
  memcpy((uint8_t*)&(pkt->data) + pkt->header.size, data, size);
  pkt->header.size += size;
  /* Calculate a simple checksum to ensure bit flips in the length field can be recognized. */
  pkt->header.checksum = pkt->header.size ^ 0x5555;
  /* Update internal status variable of how many bytes have been enqueued. */
  bytes_enqueued += sizeof(subpkt.header) + size;

#ifdef DEBUG
  dbg_printf("Enqueued packet for peripheral: %s Opcode: %X Size: %X\n  data: ",
      peripheral_to_string(peripheral), opcode, size);

  for (int i = 0; i < size; i++)
    dbg_printf("0x%02X ", *(((uint8_t*)data) + i));

  dbg_printf("\n");
#endif

cleanup:

  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);

  return bytes_enqueued;
}

void set_nirq_low()
{
  spi_end();
  dma_load();

  /* Trigger transfer. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
}

void set_nirq_high()
{
  /* Signal completion of transfer. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
}

bool is_nirq_low()
{
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
}

uint16_t get_tx_packet_size()
{
  /* Enter critical section. */
  uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1) ;

  struct complete_packet * tx_pkt = (struct complete_packet *)p_tx_buf_active;
  uint16_t const tx_packet_size = tx_pkt->header.size;

  /* Exit critical section: restore previous priority mask */
  __set_PRIMASK(primask_bit);

  return tx_packet_size;
}

void system_init() {

  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  PeriphCommonClock_Config();
}

void dma_init()
{
  MX_DMA_Init();
  clean_dma_buffer();
}

void dma_load() {
  struct complete_packet *tx_pkt = (struct complete_packet *)p_tx_buf_active;
  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;
  spi_transmit_receive((uint8_t*)&(tx_pkt->header), (uint8_t*)&(rx_pkt->header), SPI_DMA_BUFFER_SIZE);
}

void EXTI15_10_IRQHandler(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    spi_end();
    dma_load();
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (is_nirq_low())
  {
    /* Only swap buffers and transmit data if the device has initiated
      * communication by setting NIRQ low.
      */
    p_tx_buf_transfer = p_tx_buf_active;
    p_tx_buf_active = (p_tx_buf_active == TX_Buffer_1) ? TX_Buffer_2 : TX_Buffer_1;
  }

  struct complete_packet *tx_pkt = (struct complete_packet *)p_tx_buf_active;
  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;

  if (transaction_state == Idle)
  {
    /* Step #3:
     * The SPI transfer is now complete, copy to userspace memory.
     */
    memcpy((void *)rx_pkt_userspace, &(rx_pkt->data), rx_pkt->header.size);

    /* Mark the next packet as invalid. */
    *((uint32_t*)((uint8_t *)rx_pkt_userspace + rx_pkt->header.size)) = 0xFFFFFFFF;

    /* Clean the transfer buffer size to restart. */
    tx_pkt->header.size = 0;
    tx_pkt->header.checksum = tx_pkt->header.size ^ 0x5555;

    transaction_state = Complete;
    is_rx_buf_userspace_processed = false;

    set_nirq_high();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  dbg_printf("HAL_SPI_ErrorCallback\n");
  transaction_state = Idle;
  spi_end();
  dma_load();
}

void dma_handle_data()
{
  if (transaction_state == Error)
  {
    dbg_printf("dma_handle_data: got transfer error, recovering\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
    transaction_state = Idle;
    return;
  }

  /* Enter critical section. */
  volatile uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1) ;

  if (transaction_state == Complete) {

    while (!is_rx_buf_userspace_processed) {

      if (rx_pkt_userspace->header.peripheral != 0xFF &&
          rx_pkt_userspace->header.peripheral != 0x00)
      {
  #ifdef DEBUG
        dbg_printf("Peripheral: %s Opcode: %X Size: %X\n  data: ",
                  peripheral_to_string(rx_pkt_userspace->header.peripheral),
                  rx_pkt_userspace->header.opcode,
                  rx_pkt_userspace->header.size);

        for (int i = 0; i < rx_pkt_userspace->header.size; i++)
          dbg_printf("0x%02X ", *((&rx_pkt_userspace->raw_data) + i));

        dbg_printf("\n");
  #endif

        /* Invoke the registered callback for the selected peripheral. */
        int const rc = peripheral_invoke_callback(rx_pkt_userspace->header.peripheral,
                                                  rx_pkt_userspace->header.opcode,
                                                  (uint8_t *)(&(rx_pkt_userspace->raw_data)),
                                                  rx_pkt_userspace->header.size);

        if (rc < 0) {
          dbg_printf("dma_handle_data: %s callback error: %d",
                    peripheral_to_string(rx_pkt_userspace->header.peripheral) , rc);
        }

        /* Advance to the next package. */
        rx_pkt_userspace = (struct subpacket *)((uint8_t *)rx_pkt_userspace + 4 /* sizeof(subpacket.header) */ + rx_pkt_userspace->header.size);
      }
      else
      {
        /* Mark the receive buffer as having been processed. */
        is_rx_buf_userspace_processed = true;
        /* Make sure that the RX packet processing pointer is pointing to the start of the receive buffer. */
        rx_pkt_userspace = (struct subpacket *)RX_Buffer_userspace;
        transaction_state = Idle;
        spi_end();
        dma_load();
      }
    }
  }

  /* Leave critical section. */
  __set_PRIMASK(primask_bit);
}

bool is_dma_transfer_complete()
{
  bool is_dma_transfer_complete_flag_temp = false;
  /* Enter critical section. */
  volatile uint32_t primask_bit = __get_PRIMASK();
  __set_PRIMASK(1) ;

  is_dma_transfer_complete_flag_temp = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET);

  /* Leave critical section. */
  __set_PRIMASK(primask_bit);

  return is_dma_transfer_complete_flag_temp;
}
