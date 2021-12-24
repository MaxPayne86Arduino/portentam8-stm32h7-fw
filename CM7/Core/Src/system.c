#include "system.h"
#include "main.h"
#include "peripherals.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "adc.h"
#include "pwm.h"
#include "gpio.h"
#include "can_api.h"
#include "rtc.h"
#include "uart.h"
#include "rpc.h"
#include "spi.h"
/*
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
*/

void (*PeriphCallbacks[20]) (uint8_t opcode, uint8_t *data, uint8_t size); //[100];

enum { TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_ERROR };

__IO uint32_t transferState = TRANSFER_WAIT;

__attribute__((section("dma"), aligned(2048))) volatile uint8_t TX_Buffer[SPI_DMA_BUFFER_SIZE];
__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer[SPI_DMA_BUFFER_SIZE];

__attribute__((section("dma"), aligned(2048))) volatile uint8_t RX_Buffer_userspace[SPI_DMA_BUFFER_SIZE];

volatile bool get_data_amount = true;
volatile uint16_t data_amount = 0;

static void SystemClock_Config(void) {

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // ATTENTION: make sure this matches the actual hardware configuration
#ifndef PORTENTA_DEBUG_WIRED
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
#else
  HAL_PWREx_ConfigSupply(PWR_SMPS_1V8_SUPPLIES_LDO);
#endif

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
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
    Error_Handler();
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
    Error_Handler();
  }
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Do MPU */
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = (uint32_t)TX_Buffer;
  MPU_InitStruct.Size = MPU_REGION_SIZE_2KB;    
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = (uint32_t)RX_Buffer_userspace;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void MX_DMA_Init(void) {

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

void clean_dma_buffer() {
  memset((uint8_t*)TX_Buffer, 0, sizeof(TX_Buffer));
  memset((uint8_t*)RX_Buffer, 0, sizeof(RX_Buffer));
}

void enqueue_packet(uint8_t peripheral, uint8_t opcode, uint16_t size, void* data) {

/*
  int timeout = 100000;
	// don't feed data in the middle of a transmission
	while (get_data_amount == false && timeout > 0) {
		// wait for the DMA interrupt to be over
    timeout--;
	}
*/

  while (get_data_amount == false) {
    // wait for the DMA interrupt to be over
  }

	__disable_irq();
	struct complete_packet *tx_pkt = get_dma_packet();
	uint16_t offset = tx_pkt->size;
	if (offset + size > get_dma_packet_size()) {
		goto cleanup;
	}
	struct subpacket pkt;
	pkt.peripheral = peripheral;
	pkt.opcode = opcode;
	pkt.size = size;
	memcpy((uint8_t*)&(tx_pkt->data) + offset, &pkt, 4);
	memcpy((uint8_t*)&(tx_pkt->data) + offset + 4, data, size);
	tx_pkt->size += 4 + size;
	tx_pkt->checksum = tx_pkt->size ^ 0x5555;

  dbg_printf("Enqueued packet for peripheral: %s Opcode: %X Size: %X\n  data: ",
      to_peripheral_string(peripheral), opcode, size);

  for (int i = 0; i < size; i++) {
    dbg_printf("0x%02X ", *(((uint8_t*)data) + i));
  }
  dbg_printf("\n");

cleanup:
	__enable_irq();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

  //trigger_irq = true;
}

void writeVersion() {
  const char* version = "v0.1";
  enqueue_packet(PERIPH_H7, FW_VERSION, strlen(version), (void*)version);
}

void h7_handler(uint8_t opcode, uint8_t *data, uint8_t size) {
  if (opcode == FW_VERSION) {
    writeVersion();
  }
}

void system_init() {

  MPU_Config();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  PeriphCommonClock_Config();

  register_peripheral_callback(PERIPH_H7, &h7_handler);
}

void dma_init() {
  MX_DMA_Init();

  clean_dma_buffer();
}

void dispatchPacket(uint8_t peripheral, uint8_t opcode, uint16_t size, uint8_t* data) {
  //Get function callback from LUT (peripherals vs opcodes)
  (*PeriphCallbacks[peripheral])(opcode, data, size);
}

struct complete_packet* get_dma_packet() {
  return (struct complete_packet *)TX_Buffer;
}

int get_dma_packet_size() {
  return sizeof(TX_Buffer);
}

#ifdef PORTENTA_DEBUG_WIRED
void EXTI0_IRQHandler(void)
{
	if (get_data_amount) {
    spi_transmit_receive(PERIPH_SPI2, (uint8_t *)TX_Buffer,
		                     (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
#endif

void EXTI15_10_IRQHandler(void)
{
  if (get_data_amount) {
    spi_transmit_receive(PERIPH_SPI3, (uint8_t *)TX_Buffer,
		                     (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  }
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

  struct complete_packet *rx_pkt = (struct complete_packet *)RX_Buffer;
  struct complete_packet *tx_pkt = (struct complete_packet *)TX_Buffer;

  if (get_data_amount) {

#ifdef PORTENTA_DEBUG_WIRED
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5, 0);
#endif

    data_amount = max(tx_pkt->size, rx_pkt->size);

    if (data_amount == 0) {
      return;
    }

    // reconfigure the DMA to actually receive the data
#ifndef PORTENTA_DEBUG_WIRED
    spi_transmit_receive(PERIPH_SPI3, (uint8_t*)&(tx_pkt->data), (uint8_t*)&(rx_pkt->data), data_amount);
#else
    spi_transmit_receive(PERIPH_SPI2, (uint8_t*)&(tx_pkt->data), (uint8_t*)&(rx_pkt->data), data_amount);
#endif
    get_data_amount = false;

  } else {
    // real end of operation, pause DMA, memcpy stuff around and reenable DMA
    // HAL_SPI_DMAPause(&hspi1);

    transferState = TRANSFER_COMPLETE;

    memcpy((void *)RX_Buffer_userspace, &(rx_pkt->data), rx_pkt->size);

    // mark the next packet as invalid
    *((uint32_t*)((uint8_t *)RX_Buffer_userspace + rx_pkt->size)) = 0xFFFFFFFF; // INVALID;

    // clean the transfer buffer size to restart
    tx_pkt->size = 0;

#ifdef PORTENTA_DEBUG_WIRED
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_6, 0);
#endif

    get_data_amount = true;

    // HAL_SPI_DMAResume(&hspi1);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  transferState = TRANSFER_ERROR;
/*
#ifndef PORTENTA_DEBUG_WIRED
  HAL_SPI_Abort(&hspi3);
#else
  HAL_SPI_Abort(&hspi2);
#endif
*/
  spi_end();

/*
  // Restart DMA
  #ifndef PORTENTA_DEBUG_WIRED
  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)TX_Buffer,
                                  (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  #else
  HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)TX_Buffer,
                                  (uint8_t *)RX_Buffer, sizeof(uint16_t) * 2);
  #endif
*/
}

void dma_handle_data() {
  if (transferState == TRANSFER_COMPLETE) {

#ifdef PORTENTA_DEBUG_WIRED
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 1);
#endif

    struct subpacket *rx_pkt_userspace =
        (struct subpacket *)RX_Buffer_userspace;

    while (rx_pkt_userspace->peripheral != 0xFF &&
            rx_pkt_userspace->peripheral != 0x00) {
      dbg_printf("Peripheral: %s Opcode: %X Size: %X\n  data: ",
          to_peripheral_string(rx_pkt_userspace->peripheral), rx_pkt_userspace->opcode,
              rx_pkt_userspace->size);
      for (int i = 0; i < rx_pkt_userspace->size; i++) {
        dbg_printf("0x%02X ", *((&rx_pkt_userspace->raw_data) + i));
      }
      dbg_printf("\n");

      // do something useful with this packet
      dispatchPacket(rx_pkt_userspace->peripheral, rx_pkt_userspace->opcode,
          rx_pkt_userspace->size, &(rx_pkt_userspace->raw_data));

      rx_pkt_userspace = (struct subpacket *)((uint8_t *)rx_pkt_userspace + 4 + rx_pkt_userspace->size);
    }
    transferState = TRANSFER_WAIT;
  }

  if (transferState == TRANSFER_ERROR) {
      dbg_printf("got transfer error, recovering\n");
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
      transferState = TRANSFER_WAIT;
  }
}

void register_peripheral_callback(uint8_t peripheral,/* uint8_t opcode,*/ void* func) {
  PeriphCallbacks[peripheral]/*[opcode]*/ = func;
}

void Error_Handler_Name(const char* name) {
  dbg_printf("Error_Handler called by %s\n", name);
  __disable_irq();
  while (1) {
  }
}
