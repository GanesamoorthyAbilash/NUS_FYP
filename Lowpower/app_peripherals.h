/***************************************************************************//**
 * @file
 * @brief iostream usart examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_PERIPHERALS_H
#define APP_PERIPHERALS_H

#include <stdint.h>

/***************************************************************************//**
 * Initialize iostream usart
 ******************************************************************************/
void sleeptimer_app_init(void);
void IADCRescale(uint32_t newScale);
void initGPIO(void);
void initIADC(void);
void initLDMA(uint16_t *buffer, uint32_t size);
void em23start(void);
/***************************************************************************//**
 * iostream usart ticking function
 ******************************************************************************/
void calc_action(uint16_t *buffer);

#endif
