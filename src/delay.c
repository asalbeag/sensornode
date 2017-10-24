
//#include "stdint.h"
//#include "stm32f0xx_conf.h"
#include "stm32f0xx_misc.h"

volatile uint32_t uptime_ms = 0; ///< Uptime in ms

extern void SysTick_Handler(void)
{
  uptime_ms++;
}

/** Wait for X milliseconds.
 *
 * @param ms Milliseconds
 */
void delay_ms(unsigned ms)
{
  uint32_t start = uptime_ms;
  while (uptime_ms - start < ms);
}

/** Initialize the millisecond timer. */
void mstimer_init(void)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_Config(SystemCoreClock / 1000);
}

/** Return the number of milliseconds since start.
 *
 * @return Milliseconds
 */
uint32_t mstimer_get(void)
{
  return uptime_ms;
}
