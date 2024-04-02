/***************************************************************************//**
 * @file
 * @brief Top level application functions
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

#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"
#include <stdio.h>
#include "sl_iostream.h"
#include <math.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_iadc.h"
#include "em_gpio.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// How many samples to capture and average
#define NUM_SAMPLES                 1024

//Setting CLK Frequencies to be used by the ADC
#define CLK_SRC_ADC_FREQ    20000000  // CLK_SRC_ADC
#define CLK_ADC_FREQ        10000000  // CLK_ADC ==> 10 MHz max in normal mode


//Setting the AIN1 as the IADC input
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPadAna0 | 1;


#define IADC_SCALE_OFFSET_MAX_NEG   0x00020000UL // 18-bit 2's compliment
#define IADC_SCALE_OFFSET_ZERO      0x00000000UL

// Push-buttons are active-low
#define PB_PRESSED (0)


// LED Toggle output
#define BSP_GPIO_LED0_PORT       gpioPortC
#define BSP_GPIO_LED0_PIN        11

// PB input
#define BSP_GPIO_PB0_PORT       gpioPortB
#define BSP_GPIO_PB0_PIN         1


/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

static volatile IADC_Result_t sample;
static volatile double singleResult; // Volts

/**************************************************************************//**
 * @brief  GPIO Initializer
 *****************************************************************************/
void initGPIO (void)
{
  // Enable GPIO clock branch
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure push button PB0 as a user input; will use as a toggle to indicate when inputs are ready
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN , gpioModeInputPullFilter, 1);

  // Configure LED0 as output, will indicate when conversions are being performed
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  Initialize IADC function
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC0 clock branch
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Select clock for IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].vRef = 3300;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                             CLK_ADC_FREQ,
                                             0,
                                             iadcCfgModeNormal,
                                             init.srcClkPrescale);

  initAllConfigs.configs[0].twosComplement = iadcCfgTwosCompBipolar; // Force IADC to use bipolar inputs for conversion

  // Assign pins to positive and negative inputs in differential mode
  initSingleInput.posInput   = IADC_INPUT_0_PORT_PIN;
  initSingleInput.negInput   = iadcNegInputGnd;

  // Initialize the IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize the Single conversion inputs
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);
}

/**************************************************************************//**
 * @brief  Take several sequential samples and average the measurement
 *****************************************************************************/
double IADCAverageConversion(uint32_t numSamples)
{
    int i;
    double average;
    IADC_Result_t sample;

    // indicate to user IADC is busy
    GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);

    // Averaging loop
    average = 0; // reset accumulator
    for(i = 0; i < numSamples; i++)
    {
      // Start IADC conversion
      IADC_command(IADC0, iadcCmdStartSingle);

      // Wait for conversion to be complete
      while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
                  | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

      // Get ADC result
      sample = IADC_pullSingleFifoResult(IADC0);
      average += (int32_t) sample.data;
    }
    average /= NUM_SAMPLES;
    printf("\n%.2f",(average* 6.6) / 0xFFF);

    // turn off indicator LED
    GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);

    return average;
}


//Initialising USART
void app_iostream_usart_init(void)
{
  /* Prevent buffering of output/input.*/
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
  setvbuf(stdout, NULL, _IONBF, 0);   /*Set unbuffered mode for stdout (newlib)*/
  setvbuf(stdin, NULL, _IONBF, 0);   /*Set unbuffered mode for stdin (newlib)*/
#endif
  sl_iostream_set_default(sl_iostream_vcom_handle);

}


/**************************************************************************//**
 * @brief  IADC must be disabled to change scale
 *****************************************************************************/
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
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/

void app_init(void)
{
  initGPIO();
  app_iostream_usart_init();
  initIADC();
}


void app_process_action(void)
{
  int count = 0;
  uint32_t scale;
  double calibration_gain13lsb;
  double result_fullscale, result_zero, result_offset;
  double gain_correction_factor;
  double IADC_GAIN13LSB_LSB = 0.25 / 8192; // LSB Gain multiplication factor
  uint32_t IADC_CALIBRATED_GAIN13LSB;
  int32_t IADC_CALIBRATED_OFFSET;



  // Set initial offset to maximum negative and initial gain to 1.0
  scale = IADC_SCALE_GAIN3MSB_GAIN100 | IADC_SCALE_GAIN13LSB_DEFAULT | IADC_SCALE_OFFSET_MAX_NEG;

  IADCRescale(scale);

  printf("waiting for PB Press after applying max V");
  // Apply a full-scale positive input to the IADC
  // Wait until differential voltage is applied
  while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) != PB_PRESSED); //user feedback
  while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == PB_PRESSED); //make it a toggle

  printf("\nPB Pressed");
  // Take multiple conversions and average to reduce system-level noise
  result_fullscale = IADCAverageConversion(NUM_SAMPLES);


  printf("\nwaiting for PB Press after applying 0v");
  // Apply a zero differential input to the IADC (short P25-P24)
  // Wait until differential voltage is applied
  while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) != PB_PRESSED); // user feedback
  while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == PB_PRESSED);
  printf("\nPB Pressed");
  result_zero = IADCAverageConversion(NUM_SAMPLES);

  // Calculate gain correction factor
  //  In bipolar mode, expected positive full-scale for IADC is (2^11) - 1 = 2047
  gain_correction_factor = 2047 / (result_fullscale - result_zero);

  // Set IADC correction gain and clear offset in order to calibrate offset
  //  3 MSB of gain represented by 1 bit; 1 => 100 representing 1.00x to 1.2499x, 0 => 011 representing 0.75x to .9999x
  if(gain_correction_factor >= 1.0)
  {
      calibration_gain13lsb = (gain_correction_factor - 1.0) / IADC_GAIN13LSB_LSB;
      IADC_CALIBRATED_GAIN13LSB = (uint32_t) (calibration_gain13lsb + 0.5); // round to the nearest integer
      scale = IADC_SCALE_GAIN3MSB_GAIN100 | (IADC_CALIBRATED_GAIN13LSB << _IADC_SCALE_GAIN13LSB_SHIFT) | IADC_SCALE_OFFSET_ZERO;
  }
  else
  {
      calibration_gain13lsb = (gain_correction_factor - 0.75) / IADC_GAIN13LSB_LSB;
      IADC_CALIBRATED_GAIN13LSB = (uint32_t) (calibration_gain13lsb + 0.5); // round to the nearest integer
      scale = IADC_SCALE_GAIN3MSB_GAIN011 | (IADC_CALIBRATED_GAIN13LSB << _IADC_SCALE_GAIN13LSB_SHIFT) | IADC_SCALE_OFFSET_ZERO;
  }

  IADCRescale(scale);

  result_offset = IADCAverageConversion(NUM_SAMPLES);

  // scale and negate offset
  //  OFFSET is encoded as a 2's complement, 18-bit number with the LSB representing 1 / (2^20) of full scale.
  IADC_CALIBRATED_OFFSET = (int32_t) (result_offset * -256);

  // 18-bit boundary check [-2^17 <-> (2^17-1)]
  //  Check if offset is too large to be corrected and set to maximum correction allowed
  if (IADC_CALIBRATED_OFFSET > 131071)
      IADC_CALIBRATED_OFFSET = 131071;
  if (IADC_CALIBRATED_OFFSET < -131072)
      IADC_CALIBRATED_OFFSET = -131072;

  // Maintain previous gain correction; apply offset correction
  if(gain_correction_factor >= 1.0)
  {
      scale = IADC_SCALE_GAIN3MSB_GAIN100 | (IADC_CALIBRATED_GAIN13LSB << _IADC_SCALE_GAIN13LSB_SHIFT)
              | (IADC_CALIBRATED_OFFSET & _IADC_SCALE_OFFSET_MASK);
      count=1;
  }
  else
  {
      scale = IADC_SCALE_GAIN3MSB_GAIN011 | (IADC_CALIBRATED_GAIN13LSB << _IADC_SCALE_GAIN13LSB_SHIFT)
              | (IADC_CALIBRATED_OFFSET & _IADC_SCALE_OFFSET_MASK);
      count=1;
  }

  IADCRescale(scale);

//Printing of the Necessary scale value, which is to be used in the main program
  if (count==1)
    {
    printf("\n The scale to be used is %u\n",scale);
    count = 0;
    }

  // Infinite loop where we can test the recalibrated values
  while(1)
  {
      printf("\n waiting for PB Press \n");
      while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) != PB_PRESSED); //user feedback
      while(GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == PB_PRESSED); //make it a toggle
    // Start IADC conversion
    IADC_command(IADC0, iadcCmdStartSingle);

    // Wait for conversion to be complete
    while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
                | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

    // Get ADC result
    sample = IADC_pullSingleFifoResult(IADC0);

    // Calculate input voltage:
    //  For differential inputs, the resultant range is from -Vref to +Vref, i.e.,
    //  for Vref = AVDD = 3.30V, 12 bits represents 6.60V full scale IADC range.
    singleResult = ((int32_t) sample.data * 6.6) / 0xFFF;
    printf("%.2f",singleResult);
  }
}
