#include "stm32f4xx.h"

/* ============================================================================
 *  APP_B  ——  烧在 0x08040000
 *  现象：PF10（LED1）快闪 5Hz，用来和 A 分区区分
 * ========================================================================== */

#define APP_B_ADDR      0x08040000UL
#define LED1_PIN        10U

static void delay_ms(volatile uint32_t n)
{
    while (n--) { for (volatile uint32_t i = 0; i < 4000; i++); }
}

int main(void)
{
    /* ★ B 分区的向量表地址 */
    SCB->VTOR = APP_B_ADDR;
    __enable_irq();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    GPIOF->MODER   &= ~(3U << (LED1_PIN * 2U));
    GPIOF->MODER   |=  (1U << (LED1_PIN * 2U));
    GPIOF->OTYPER  &= ~(1U << LED1_PIN);
    GPIOF->OSPEEDR &= ~(3U << (LED1_PIN * 2U));
    GPIOF->PUPDR   &= ~(3U << (LED1_PIN * 2U));
    GPIOF->ODR     |=  (1U << LED1_PIN);

    /* PF10 快闪 5Hz = B 分区 */
    for (;;)
    {
        GPIOF->ODR ^= (1U << LED1_PIN);
        delay_ms(100);
    }
}