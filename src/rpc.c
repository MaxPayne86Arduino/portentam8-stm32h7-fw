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

#include "openamp.h"
#include "arduino_openamp.h"
#include "stm32h7xx_hal.h"
#include "ringbuffer.h"
#include "stm32h7xx_ll_rcc.h"
#include "debug.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum endpoints_t {
  ENDPOINT_RAW = 0,
  ENDPOINT_RPC
};

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static struct rpmsg_endpoint rp_endpoints[2];
extern ring_buffer_t virtual_uart_ring_buffer;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

int rpmsg_recv_raw_callback(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
  ring_buffer_queue_arr(&virtual_uart_ring_buffer, (const char *)data, len);

  return 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
  uint8_t buffer[1] = {0};
  struct rpmsg_endpoint *ept = NULL;

  if (strcmp(name, "rpc") == 0) {
      ept = &rp_endpoints[ENDPOINT_RPC];
  } else if (strcmp(name, "raw") == 0) {
      ept = &rp_endpoints[ENDPOINT_RAW];
  }

  if (ept) {
      OPENAMP_create_endpoint(ept, name, dest, rpmsg_recv_raw_callback, NULL);
      OPENAMP_send(ept, buffer, sizeof(buffer));
  }
}

int serial_rpc_begin() {

  /* Initialize OpenAmp and libmetal libraries */
  if (MX_OPENAMP_Init(RPMSG_HOST, new_service_cb) !=  HAL_OK) {
    return 0;
  }

  /* Initialize the rpmsg endpoint to set default addresses to RPMSG_ADDR_ANY */
  memset(rp_endpoints, 0, sizeof(rp_endpoints));

  return 1;
}

int serial_rpc_ready() {
  /*
  * The rpmsg service is initiate by the remote processor, on H7 new_service_cb
  * callback is received on service creation. Wait for the callback
  */
  uint32_t millis_start = HAL_GetTick();
  while (rp_endpoints[0].rdev == NULL || rp_endpoints[1].rdev == NULL) {
    if ((HAL_GetTick() - millis_start) >= 1000) {
      dbg_printf("M4 RPC timeout\n");
      return 0;
    }
  }

  return 1;
}

void serial_rpc_available() {
  OPENAMP_check_for_message();
}

void serial_rpc_write(uint8_t const * buf, size_t len) {
  // we'll only get rpc requests from "upstairs"
  OPENAMP_send(&rp_endpoints[ENDPOINT_RPC], buf, len);
}

void HSEM1_IRQHandler(void)
{
  OPENAMP_check_for_message();
  HAL_HSEM_IRQHandler();
}
