#if defined(STM32F100xB)
#include "startup_stm32f100xb.s"
#elif defined(STM32F100xE)
#include "startup_stm32f100xe.s"
#elif defined(STM32F101x6)
#include "startup_stm32f101x6.s"
#elif defined(STM32F101xB)
#include "startup_stm32f101xb.s"
#elif defined(STM32F101xE)
#include "startup_stm32f101xe.s"
#elif defined(STM32F101xG)
#include "startup_stm32f101xg.s"
#elif defined(STM32F102x6)
#include "startup_stm32f102x6.s"
#elif defined(STM32F102xB)
#include "startup_stm32f102xb.s"
#elif defined(STM32F103x6)
#include "startup_stm32f103x6.s"
#elif defined(STM32F103xB)
#include "startup_stm32f103xb.s"
#elif defined(STM32F103xE)
#include "startup_stm32f103xe.s"
#elif defined(STM32F103xG)
#include "startup_stm32f103xg.s"
#elif defined(STM32F105xC)
#include "startup_stm32f105xc.s"
#elif defined(STM32F107xC)
#include "startup_stm32f107xc.s"
#else
#error "Please select a valid STM32F1xx device using preprocessor macros"
#endif