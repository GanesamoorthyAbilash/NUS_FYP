/***************************************************************************//**
 * @file
 * @brief
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

//For printing
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


//Driver files
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "em_ldma.h"
#include "app.h"
#include "sl_sleeptimer.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
// Declare time to sleep in ms
#define TIMEOUT_MS 10000


//Setting CLK Frequencies to be used by the ADC
#define CLK_SRC_ADC_FREQ    20000000  // CLK_SRC_ADC
#define CLK_ADC_FREQ        10000000  // CLK_ADC ==> 10 MHz max in normal mode


//Setting the AIN1 as the IADC input
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPadAna0 | 1;

//Setting the LDMA CH0 as the IADC LDMA CH
#define IADC_LDMA_CH              0

// LED0 output for checking sleeptimer IRQ
#define GPIO_OUTPUT_0_PORT        gpioPortC
#define GPIO_OUTPUT_0_PIN         10

// LED1 output for indicating key signal detected
#define GPIO_OUTPUT_1_PORT        gpioPortC
#define GPIO_OUTPUT_1_PIN         11

//For timer
static sl_sleeptimer_timer_handle_t periodic_timer;


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

void initGPIO(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  //Setting LED Pins as output
  GPIO_PinModeSet(GPIO_OUTPUT_0_PORT, GPIO_OUTPUT_0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(GPIO_OUTPUT_1_PORT, GPIO_OUTPUT_1_PIN, gpioModePushPull, 0);
}


// Periodic timer IRQ callback
static void on_periodic_timeout(sl_sleeptimer_timer_handle_t *handle,
                                void *data)
{
  (void)&handle;
  (void)&data;
  flagset();
}

//Initialising function for the periodic timer
//Flag set to ensure timer does not use the HF clocks which might be turned off when going into EM2
void sleeptimer_app_init(void)
{
  // Create timer for waking up the system periodically.
  sl_sleeptimer_start_periodic_timer_ms(&periodic_timer,
                                        TIMEOUT_MS,
                                        on_periodic_timeout, NULL,
                                        0,
                                        SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}


//IADC Recalibration
void IADCRescale(uint32_t newScale)
{
    // Disable the IADC
    IADC0->EN_CLR = IADC_EN_EN;

    // wait for IADC to disable
    while((IADC0->EN & _IADC_EN_DISABLING_MASK) == IADC_EN_DISABLING);

    // configure new scale settings
    IADC0->CFG[0].SCALE = newScale;

    // Re-enable IADC
    IADC0->EN_SET = IADC_EN_EN;
    IADC_command(IADC0, iadcCmdEnableTimer);
    IADC_command(IADC0, iadcCmdStartSingle);
}


//IADC initialisation function
void initIADC(void)
{
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Use the FSRC0 as the IADC clock so it can run in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK,cmuSelect_FSRCO);

  // Declare initialization structures
  IADC_Init_t init = IADC_INIT_DEFAULT;

  //ADC does not turn off in between scans
  init.warmup = iadcWarmupKeepWarm;

  // Setting the SRC_CLK to the above defined CLK_SRC_ADC_FREQ  value
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  //Number of SRC_CLK cycles to trigger IADC Read
  init.timerCycles = 200; // 20MHz/200 ==> 100kHz sampling



  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;

  //Setting ADC into Normal mode
  initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;

  //Setting Oversampling rate of 2 times
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;

  //Setting Gain value of 1
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;

  //Setting IADC Ref as 0 to 3.3v
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].vRef = 3300;

  // Setting to averaging over 1 sample
  initAllConfigs.configs[0].digAvg = iadcDigitalAverage1;

  // Setting the ADC_CLK to the above defined CLK_ADC_FREQ  value
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);


  IADC_InitSingle_t initSingle = { iadcAlignRight12,/*Setting the IADC 12-bit value to be right aligned in the 32 bit register */ \
      false,                           /* Disabling ID to be stored along with the 12-bit Result */              \
      iadcFifoCfgDvl8,                 /* Setting the FIFO to trigger only when 8 IADC values have been stored in it */   \
      true,                            /*Wakes up the DMA on FIFO trigger, in this case after storing 8 values */ \
      iadcTriggerSelTimer,             /* Start ADC conversion on Timer trigger */   \
      iadcTriggerActionOnce,           /* Convert once on single trigger */        \
      false,                           /* No tailgating */                         \
      false                           /* Do not start single queue */             \
    };

  //Setting the IADC input pin to AIN1
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;
  initSingleInput.posInput   =  IADC_INPUT_0_PORT_PIN;
  initSingleInput.negInput = iadcNegInputGnd;



  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize single input
  IADC_initSingle (IADC0, &initSingle, &initSingleInput);
}

//LDMA initialisation function, receives storage destination address, and number of samples to store
void initLDMA(int16_t *buffer, uint32_t size)
{
  // Declare LDMA init structs
  LDMA_Init_t initldma = LDMA_INIT_DEFAULT;
  LDMA_Init(&initldma);
  LDMA_Descriptor_t descriptor;

  // Trigger LDMA transfer on IADC single completion
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SINGLE);


  descriptor =
    (LDMA_Descriptor_t)
  {                                                                    \
    .xfer =                                                            \
    {                                                                  \
      .structType   = ldmaCtrlStructTypeXfer,       /* Transfer mode */                       \
      .structReq    = 0,                                               \
      .xferCnt      = (size) - 1,                   /* number of samples to store */                  \
      .byteSwap     = 0,                                               \
      .blockSize    = ldmaCtrlBlockSizeUnit8,        /* Transferring in blocks of 8 */                      \
      .doneIfs      = 1,                             /* Generate Interrupts when LDMA is done transferring */                  \
      .reqMode      = ldmaCtrlReqModeBlock,          /* Transfer in Blocks */                   \
      .decLoopCnt   = 0,                                               \
      .ignoreSrec   = 1,                                               \
      .srcInc       = ldmaCtrlSrcIncNone,             /* IADC Source address does not change*/                  \
      .size         = ldmaCtrlSizeHalf,               /*Transferring half word size (16bits)*/                  \
      .dstInc       = ldmaCtrlDstIncOne,              /*Increases Destination address by half word size for next sample*/                   \
      .srcAddrMode  = ldmaCtrlSrcAddrModeAbs,         /*Absolute Addressing mode*/                 \
      .dstAddrMode  = ldmaCtrlDstAddrModeAbs,         /*Absolute Addressing mode*/                    \
      .srcAddr      = (uint32_t)(&(IADC0->SINGLEFIFODATA)),  /*IADC Source Addr*/                                 \
      .dstAddr      = (int32_t)(buffer),                     /* Destination Buffer address*/              \
      .linkMode     = 0,                               /*Linking disabled*/     \
      .link         = 0,                                               \
      .linkAddr     = 0                           \
    } \
  };
  LDMA_StartTransfer(IADC_LDMA_CH, (void*)&transferCfg, (void*)&descriptor);
}

//For EM2 power mode Initialisation
void em23start(void)
{
  // Turn on DCDC regulator
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);

  // Enable voltage downscaling in EM2/3 (VSCALE0)
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;

  // Initialize EM mode 2/3
  EMU_EM23Init(&em23Init);

}

//Printing function for debugging, prints on the serial port
void calc_action(uint16_t *buffer)
{
  for(int i=0;i<10240;i++)
    {
      //printf("%u    ",(buffer[i]));
      //printf("%u    ",(((buffer[i] >> 4) & 0x0FF)));
      printf("%.2f    ",(((buffer[i] >> 4) & 0x0FF))* 3.3 / 0x0FF);
      //printf("%.2f    ",(buffer[i])* 3.3 / 0xFFF);
    }
  printf("\n\n");
}
