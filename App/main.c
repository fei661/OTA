#include "stm32f4xx.h"

/* ============================================================================
 *  APP_A  ——  烧在 0x08010000
 *  工程配置：IROM1 Start=0x08010000  Size=0x30000
 *            system_stm32f4xx.c 里 VECT_TAB_OFFSET=0x10000, PLL_M=8
 *  现象：PF9（LED0）快闪
 * ========================================================================== */

#define APP_A_ADDR      0x08010000UL
#define LED0_PIN        9U

static void delay_ms(volatile uint32_t n)
{
    while (n--) { for (volatile uint32_t i = 0; i < 4000; i++); }
}

int main(void)
{
    /* ★ 从 Bootloader 跳过来必须做的两行 */
    SCB->VTOR = APP_A_ADDR;
    __enable_irq();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    GPIOF->MODER   &= ~(3U << (LED0_PIN * 2U));
    GPIOF->MODER   |=  (1U << (LED0_PIN * 2U));
    GPIOF->OTYPER  &= ~(1U << LED0_PIN);
    GPIOF->OSPEEDR &= ~(3U << (LED0_PIN * 2U));
    GPIOF->PUPDR   &= ~(3U << (LED0_PIN * 2U));
    GPIOF->ODR     |=  (1U << LED0_PIN);

    /* PF9 快闪 = A 分区固件（升级测试时可改这里观察变化） */
    for (;;)
    {
        GPIOF->ODR ^= (1U << LED0_PIN);
        delay_ms(150);
    }
}