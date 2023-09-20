/*!****************************************************************************
 * @file
 * system_stm32f1xx.c
 *
 * @brief
 * Early system initialisation
 *
 * @date  19.08.2023
 ******************************************************************************/

/*- Header files -------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "system_stm32f1xx.h"


/*- Compiler options ---------------------------------------------------------*/
#pragma GCC push_options
#pragma GCC optimize("O1")


/*- Macros -------------------------------------------------------------------*/
/*! Processor supports USB feature                                            */
#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) 
  #define PROC_FEATURE_USB      1u  /* Supports USB feature                   */
#else
  #define PROC_FEATURE_USB      0u  /* Does not support USB feature           */
#endif

/*! Processor supports USB OTG FS feature                                     */
#if defined(STM32F105xC) || defined(STM32F107xC)
  #define PROC_FEATURE_OTGFS    1u  /* Supports USB OTG FS feature            */
#else
  #define PROC_FEATURE_OTGFS    0u  /* Does not support USB OTG FS feature    */
#endif

/*! Processor supports PLL2, PLL3 feature                                     */
#if defined(STM32F105xC) || defined(STM32F107xC)
  #define PROC_FEATURE_PLL23    1u  /* Supports PLL2 and PLL3 feature         */
#else
  #define PROC_FEATURE_PLL23    0u  /* Does not support PLL2 and PLL3 feature */
#endif

/*! Processor supports PREDIV1 feature                                        */
#if PROC_FEATURE_PLL23 || defined(STM32F100xB) || defined(STM32F100xE)
  #define PROC_FEATURE_PREDIV1  1u  /* Supports PREDIV1 feature               */
#else
  #define PROC_FEATURE_PREDIV1  0u  /* Does not support PREDIV1 feature       */
#endif


/*- Global variables ---------------------------------------------------------*/
/*! AHB Prescaler shift Look-Up Table                                         */
const uint8_t AHBPrescTable[16] = { [0 ... 7] = 0, 1, 2, 3, 4, 6, 7, 8, 9};

/*! APB Prescaler shift Look-Up Table                                         */
const uint8_t APBPrescTable[8] = { [0 ... 3] = 0, 1, 2, 3, 4};

/*! Calculated SYSCLK frequency in Hz                                         */
volatile uint32_t SystemCoreClock;


/*- Private functions --------------------------------------------------------*/
#if PROC_FEATURE_PLL23
/*!*****************************************************************************
 * @brief
 * Calc PLL2 frequency for PLL configuration
 * 
 * @param[in] ulCfgR2     Cached RCC_CFGR2 value
 * @return  (uint32_t)  PLL frequency in Hz
 * @date  19.08.2023
*******************************************************************************/
static uint32_t ulCalcPll2Freq(uint32_t ulCfgR2)
{
  uint32_t ulDiv = ((ulCfgR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
  uint32_t ulMul = ((ulCfgR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
  
  return (HSE_VALUE / ulDiv) * ulMul;
}
#endif /* PROC_FEATURE_PLL23 */

/*!*****************************************************************************
 * @brief
 * Calc PREDIV1 frequency from configuration
 * 
 * @param[in] ulCfgR      Cached RCC_CFGR value
 * @return  (uint32_t)  PREDIV1 output frequency in Hz
 * @date  19.08.2023
 ******************************************************************************/
static uint32_t ulCalcPrediv1Freq(uint32_t ulCfgR)
{
  uint32_t ulPredivFreq;

#if PROC_FEATURE_PLL23
  uint32_t ulCfgR2 = RCC->CFGR2;

  /* Divider range in CFGR2                             */
  uint32_t ulDiv = ((ulCfgR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos) + 1;

  if (!(ulCfgR2 & RCC_CFGR2_PREDIV1SRC))
  {
    /* HSE as PREDIV1 input                             */
    ulPredivFreq = HSE_VALUE / ulDiv;
  }
  else
  {
    /* PLL2 as PREDIV1 input                            */
    ulPredivFreq = ulCalcPll2Freq(ulCfgR2) / ulDiv;
  }
#else
  /* HSE or HSE/2 as PREDIV1 input                      */
  ulPredivFreq = HSE_VALUE >> !!(ulCfgR & RCC_CFGR_PLLXTPRE);
#endif /* PROC_FEATURE_PLL23 */

  return ulPredivFreq;
}

/*!*****************************************************************************
 * @brief
 * Determine SYSCLK base clock frequency for PLL configuration
 * 
 * @param[in] ulCfgR      Cached RCC_CFGR value
 * @return  (uint32_t)  Base clock frequency in Hz
 * @date  19.08.2023
*******************************************************************************/
static uint32_t ulCalcPllBaseClk(uint32_t ulCfgR)
{
  uint32_t ulInputFreq;
  uint32_t ulMul = ((ulCfgR & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos) + 2;
  
  if (!(ulCfgR & RCC_CFGR_PLLSRC))
  {
    /* HSI/2 as PLL input                                 */
    ulInputFreq = HSI_VALUE / 2;
  }
  else
  {
    /* PREDIV1 as PLL input                               */
    ulInputFreq = ulCalcPrediv1Freq(ulCfgR);
  }

  return ulInputFreq * ulMul;
}


/*!*****************************************************************************
 * @brief
 * Early system init
 * 
 * Configures relocateable interrupt vector table and initialises minimum-
 * config for clock tree.
 * 
 * @date  19.08.2023
 ******************************************************************************/
void SystemInit(void)
{
  /* Enable HSI and switch SYSCLK to it                   */
  RCC->CR |= RCC_CR_HSION;
  RCC->CFGR &= ~(RCC_CFGR_MCO | RCC_CFGR_ADCPRE | RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE | RCC_CFGR_SW);

  /* Power down and de-configure PLL and HSE              */
  RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEON);
  RCC->CR &= ~RCC_CR_HSEBYP;
  RCC->CFGR &= ~(RCC_CFGR_PLLMULL | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC |
#if PROC_FEATURE_USB
  RCC_CFGR_USBPRE |
#elif PROC_FEATURE_OTGFS
  RCC_CFGR_OTGFSPRE |
#endif /* PROC_FEATURE_USB, PROC_FEATURE_OTGFS */
  0);

  /* Disable RCC interrupts and clear interrupt flags     */
  RCC->CIR &= ~(RCC_CIR_PLLRDYIE | RCC_CIR_HSERDYIE | RCC_CIR_HSIRDYIE | RCC_CIR_LSERDYIE | RCC_CIR_LSIRDYIE |
#if PROC_FEATURE_PLL23
  RCC_CIR_PLL3RDYIE | RCC_CIR_PLL2RDYIE |
#endif /* PROC_FEATURE_PLL23 */
  0);
  RCC->CIR |= RCC_CIR_PLLRDYC | RCC_CIR_HSERDYC | RCC_CIR_HSIRDYC | RCC_CIR_LSERDYC | RCC_CIR_LSIRDYC |
#if PROC_FEATURE_PLL23
  RCC_CIR_PLL3RDYC | RCC_CIR_PLL2RDYC |
#endif
  0;

  /* Update System Core clock frequency                   */
  SystemCoreClock = HSI_VALUE;
}

/*!*****************************************************************************
 * @brief
 * Calculate HCLK from current configuration
 * 
 * @date  19.08.2023
 ******************************************************************************/
void SystemCoreClockUpdate(void)
{
  uint32_t ulCfgR = RCC->CFGR;
  uint32_t ulBaseClk;

  /* Determine SYSCLK base clock frequency                */
  switch (ulCfgR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_HSI: ulBaseClk = HSI_VALUE; break;
    case RCC_CFGR_SWS_HSE: ulBaseClk = HSE_VALUE; break;
    case RCC_CFGR_SWS_PLL: ulBaseClk = ulCalcPllBaseClk(ulCfgR); break;
    default:
      /* Invalid configuration                            */
      ulBaseClk = HSI_VALUE;
  }

  /* Apply HCLK prescaler to determine HCLK               */
  uint8_t ucPrescInd = (ulCfgR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
  SystemCoreClock = ulBaseClk >> AHBPrescTable[ucPrescInd];
}


/*- Restore compiler options -------------------------------------------------*/
#pragma GCC pop_options
