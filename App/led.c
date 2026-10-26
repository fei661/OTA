#include "led.h"

/* ---------------- 引脚定义（改板子只改这里）---------------- */
#define LED_PORT        GPIOF
#define LED0_PIN        9U
#define LED1_PIN        10U

/* 引脚号表：用 LED 编号当索引，方便扩展 */
static const uint16_t led_pin[] = { LED0_PIN, LED1_PIN };

/**
  * @brief  LED 初始化（PF9 / PF10 推挽输出，初始全灭）
  * @param  无
  * @retval 无
  */
void LED_Init(void)
{
    /* ① 使能 GPIOF 时钟（F4 的 GPIO 挂 AHB1 总线） */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

    /* ② 配置为【通用推挽输出】 */
    /* MODER: 每个引脚占 2 位，01 = 输出 */
    LED_PORT->MODER   &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));
    LED_PORT->MODER   |=  ((1U << (LED0_PIN * 2U)) | (1U << (LED1_PIN * 2U)));

    /* OTYPER: 每个引脚占 1 位，0 = 推挽，1 = 开漏 */
    LED_PORT->OTYPER  &= ~((1U << LED0_PIN) | (1U << LED1_PIN));

    /* OSPEEDR: 每个引脚占 2 位，00 = 低速 */
    LED_PORT->OSPEEDR &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));

    /* PUPDR: 每个引脚占 2 位，00 = 无上下拉 */
    LED_PORT->PUPDR   &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));

    /* ③ 初始状态全灭（低电平点亮，所以置 1 = 灭） */
    LED_Off(LED0);
    LED_Off(LED1);
}

/**
  * @brief  点亮指定 LED
  * @param  n  LED 编号（LED0 / LED1）
  * @retval 无
  */
void LED_On(uint8_t n)
{
    if (n > LED1) return;

    /* 低电平点亮：把对应位置 0 */
    LED_PORT->ODR &= ~(1U << led_pin[n]);
}

/**
  * @brief  熄灭指定 LED
  * @param  n  LED 编号（LED0 / LED1）
  * @retval 无
  */
void LED_Off(uint8_t n)
{
    if (n > LED1) return;

    /* 置 1 = 灭 */
    LED_PORT->ODR |= (1U << led_pin[n]);
}

/**
  * @brief  翻转指定 LED 状态
  * @param  n  LED 编号（LED0 / LED1）
  * @retval 无
  */
void LED_Toggle(uint8_t n)
{
    if (n > LED1) return;

    LED_PORT->ODR ^= (1U << led_pin[n]);
}

/**
  * @brief  设置指定 LED 亮灭
  * @param  n   LED 编号（LED0 / LED1）
  * @param  On  0=熄灭  非0=点亮
  * @retval 无
  */
void LED_Set(uint8_t n, uint8_t On)
{
    if (On)
    {
        LED_On(n);
    }
    else
    {
        LED_Off(n);
    }
}