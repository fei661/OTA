#include "stm32f4xx.h"

/* ============================================================================
 *  STM32F407 OTA Bootloader v1.1
 *
 *  工程配置：IROM1 Start=0x08000000  Size=0x8000(32KB)
 *            system_stm32f4xx.c: VECT_TAB_OFFSET=0x00, PLL_M=8
 *            startup_stm32f40_41xxx.s: Stack_Size=0x2000
 *
 *  硬件： GEC-M4 / STM32F407ZET6 / 512KB Flash / 192KB RAM
 *        W25Q128(16MB)：SPI1 = PB3(SCK)/PB4(MISO)/PB5(MOSI)，AF5；CS = PB14
 *        LED0 = PF9  LED1 = PF10（低电平点亮）
 *        USART1 = PA9(TX)/PA10(RX)，115200-8-N-1
 *
 *  片内分区：Bootloader 32K | 标志 16K | 参数 16K | APP_A 192K | APP_B 192K
 * ========================================================================== */

/* ---- 片内分区 ---- */
#define APP_A_ADDR       0x08010000UL
#define APP_B_ADDR       0x08040000UL
#define APP_A_SECTOR     4U
#define APP_B_SECTOR     6U
#define APP_SECTORS      2U
#define APP_SIZE         (192UL * 1024UL)

#define OTA_FLAG_ADDR    0x08008000UL
#define FLAG_SECTOR      2U

/* ---- 外部 W25Q 分区 ---- */
#define W25Q_STAGE_ADDR  0x000000UL          /* 固件暂存区 */
#define W25Q_CKPT_ADDR   0x400000UL          /* 断点记录  */

/* ---- 帧协议 ---- 不是标准协议如Ymodem/xmodem*/
#define FRAME_HEAD0      0xAAU
#define FRAME_HEAD1      0x55U
#define CMD_BEGIN        0x01U
#define CMD_DATA         0x02U
#define CMD_END          0x03U
#define CMD_ACTIVATE     0x04U
#define CMD_QUERY        0x05U
#define RSP_ACK          0x81U
#define RSP_NAK          0x82U
#define MAX_PAYLOAD      512U

/* ---- LED ---- */
#define LED0_PIN         9U
#define LED1_PIN         10U

/* ---- 魔数（"OTA1" / "CKP1"） ---- */
#define OTA_MAGIC        0x4F544131UL
#define CKPT_MAGIC       0x4B505431UL
#define PART_A           0U
#define PART_B           1U
#define PART_NONE        0xFFU

/* ---- 片内 FLASH 寄存器位 ---- */
#define FLASH_KEY1       0x45670123UL
#define FLASH_KEY2       0xCDEF89ABUL
#define CR_PG_           (1UL << 0)
#define CR_SER_          (1UL << 1)
#define CR_SNB_POS       3U
#define CR_PSIZE_POS     8U
#define CR_STRT_         (1UL << 16)
#define CR_LOCK_         (1UL << 31)
#define SR_EOP_          (1UL << 0)
#define SR_ERR_MASK_     (0x3FUL)
#define SR_BSY_          (1UL << 16)

/* ---- SPI 寄存器位 ---- */
#define SPI_CR1_MSTR_    (1UL << 2)
#define SPI_CR1_SSM_     (1UL << 9)
#define SPI_CR1_SSI_     (1UL << 8)
#define SPI_CR1_SPE_     (1UL << 6)
#define SPI_CR1_BR_POS   3U
#define SPI_SR_RXNE_     (1UL << 0)
#define SPI_SR_TXE_      (1UL << 1)

/* ---- W25Q 命令 ---- */
#define W25Q_CMD_JEDEC      0x9FU
#define W25Q_CMD_READ       0x03U
#define W25Q_CMD_PAGEPROG   0x02U
#define W25Q_CMD_SECTOR_ER  0x20U
#define W25Q_CMD_WREN       0x06U
#define W25Q_CMD_RDSR       0x05U
#define W25Q_PAGE_SIZE      256UL
#define W25Q_SECTOR_SIZE    4096UL

/* ---- 数据结构bootloader部分 ---- */
typedef struct {
    uint32_t magic;
    uint32_t active_part;
    uint32_t pending_part;
    uint32_t version_a;
    uint32_t version_b;
    uint32_t self_crc;
} ota_flag_t;
//W25Q
typedef struct {
    uint32_t magic;
    uint32_t total_size;
    uint32_t downloaded;
    uint32_t src_type;      /1=UART 2=SD，预留 /
    uint32_t version;
    uint32_t self_crc;
} ota_ckpt_t;

typedef void (*pFunc)(void);//拿到APP分区的入口地址，跳转APP运行

static ota_flag_t g_flag;
static ota_ckpt_t g_ck;
static uint32_t   g_last_sector = 0xFFFFFFFFUL;   /* 已擦除的 W25Q 扇区 */

/* ============================================================================
 *  延时 / LED
 * ========================================================================== */

/* 精确毫秒延时：SysTick 轮询，不产生中断 */
static void delay_ms(uint32_t n)
{
    uint32_t reload = SystemCoreClock / 1000U;
    if (reload == 0U || reload > 0xFFFFFFUL) reload = 168000U;

    while (n--)
    {
        SysTick->LOAD = reload - 1U;
        SysTick->VAL  = 0;
        SysTick->CTRL = 5U;                              /* 处理器时钟 + 使能，无中断 */
        while ((SysTick->CTRL & (1UL << 16)) == 0U);     /* 等 COUNTFLAG */
        SysTick->CTRL = 0;
    }
}

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

    GPIOF->MODER   &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));
    GPIOF->MODER   |=  ((1U << (LED0_PIN * 2U)) | (1U << (LED1_PIN * 2U)));
    GPIOF->OTYPER  &= ~((1U << LED0_PIN) | (1U << LED1_PIN));
    GPIOF->OSPEEDR &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));
    GPIOF->PUPDR   &= ~((3U << (LED0_PIN * 2U)) | (3U << (LED1_PIN * 2U)));
    GPIOF->ODR     |=  ((1U << LED0_PIN) | (1U << LED1_PIN));   /* 默认熄灭 */
}

static void led_blink(uint8_t pin, uint32_t n, uint32_t on_ms, uint32_t off_ms)
{
    while (n--)
    {
        GPIOF->ODR &= ~(1U << pin);
        delay_ms(on_ms);
        GPIOF->ODR |=  (1U << pin);
        delay_ms(off_ms);
    }
}

/* ============================================================================
 *  串口 USART1（PA9/PA10，AF7）
 * ========================================================================== */

//算出APB2总线时钟频率，计算pclk2用来计算波特率
//读取 RCC 配置里 APB2 的分频系数，算出 **APB2 外设时钟频率**，给 USART1 计算波特率用
static uint32_t get_pclk2(void)
{
    uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7U;
    uint32_t div   = (ppre2 >= 4U) ? (1UL << (ppre2 - 3U)) : 1UL;
    return SystemCoreClock / div;
}

static void uart_init(void)
{
    uint32_t pclk2;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 = bit19:18，PA10 = bit21:20 */
    GPIOA->MODER   &= ~((3U << 18) | (3U << 20));
    GPIOA->MODER   |=  ((2U << 18) | (2U << 20));                 /* 复用 */
    GPIOA->OSPEEDR |=  ((2U << 18) | (2U << 20));
    GPIOA->PUPDR   &= ~((3U << 18) | (3U << 20));
    GPIOA->PUPDR   |=  ((1U << 18) | (1U << 20));                 /* 上拉 */
    GPIOA->AFR[1]  &= ~((0xFUL << 4) | (0xFUL << 8));
    GPIOA->AFR[1]  |=  ((7UL << 4) | (7UL << 8));                 /* AF7 = USART1 */

    /* 波特率：BRR = fPCLK2 / 115200（四舍五入） */
    pclk2 = get_pclk2();
    if (pclk2 < 1000000UL) pclk2 = 16000000UL;                    /* 兜底 */
    USART1->BRR = (pclk2 + (115200UL / 2UL)) / 115200UL;

    USART1->CR1 = (1U << 3) | (1U << 2) | (1U << 13); //防止崩溃            /* TE | RE | UE */
}

static void uart_putc(char c)
{
    while (!(USART1->SR & (1U << 7)));                            /* 等 TXE */
    USART1->DR = (uint16_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static void uart_putb(uint8_t b) { uart_putc((char)b); }

static void uart_puthex(uint32_t v)
{
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4)
    {
        uint32_t d = (v >> i) & 0xFU;
        uart_putc((d < 10U) ? (char)('0' + d) : (char)('A' + d - 10U));
    }
}

static void uart_putu(uint32_t v)
{
    char b[12];
    int  i = 0;
    if (v == 0U) { uart_putc('0'); return; }
    while (v) { b[i++] = (char)('0' + v % 10U); v /= 10U; }
    while (i--) uart_putc(b[i]);
}

/* 高速轮询接收：不做任何延时，115200 下不会丢字节 */
//专门给 OTA 解析帧用，**防止上位机断开后程序卡死在死等**。
static int uart_getc_timeout(uint32_t ms)
{
    uint32_t reload = SystemCoreClock / 1000U;
    if (reload == 0U || reload > 0xFFFFFFUL) reload = 168000U;

    int result = -1;

    SysTick->LOAD = reload - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5U;

    for (;;)
    {
        if (USART1->SR & (1U << 5))                              /* RXNE */
        {
            result = (int)(USART1->DR & 0xFFU);
            break;
        }
        if (SysTick->CTRL & (1UL << 16))                         /* 1ms 到，清 BY COUNTFLAG */
        {
            if (ms <= 1U) break;
            ms--;
        }
    }

    SysTick->CTRL = 0;
    return result;
}

/* ============================================================================
 *  CRC
 * ========================================================================== */

/* CRC16-Modbus：poly 0xA001，init 0xFFFF，不取反 */
//1. `crc16_calc`：CRC16-Modbus
//2. `crc32_calc`：标准 CRC32（和 zlib 的 crc32 一致）
static uint16_t crc16_calc(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFU;
    while (n--)
    {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
    }
    return crc;
}

/* CRC32：poly 0xEDB88320，init 0xFFFFFFFF，结果取反（与 zlib.crc32 一致） */
static uint32_t crc32_calc(const void *d, uint32_t n)
{
    const uint8_t *p = (const uint8_t *)d;
    uint32_t crc = 0xFFFFFFFFUL;
    while (n--)
    {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return ~crc;
}

/* ============================================================================
 *  片内 Flash
 * ========================================================================== */
//内置flash底层读写驱动,Bootloader 专门用来读写那个**A/B 分区启动标记 ota_flag_t**。
static void flash_unlock(void)
{
    if (FLASH->CR & CR_LOCK_) { FLASH->KEYR = FLASH_KEY1; FLASH->KEYR = FLASH_KEY2; }
}

static void flash_lock(void) { FLASH->CR |= CR_LOCK_; }

static int flash_erase_sector(uint8_t snb)
{
    int err = 0;

    flash_unlock();
    while (FLASH->SR & SR_BSY_);//等待 Flash 空闲。BSY=1 代表 Flash 正在操作，必须等它做完才能下发新命令。
    FLASH->SR = SR_ERR_MASK_;//清零之前残留的错误标志，避免旧错误干扰本次判断。

    FLASH->CR &= ~((0xFUL << CR_SNB_POS) | (0x3UL << CR_PSIZE_POS) |
                   CR_SER_ | CR_PG_ | CR_STRT_);//把 CR 寄存器里相关位全部清空：扇区号、编程宽度、扇区擦除位、编程位、启动位，清除上一次的配置。
    FLASH->CR |= CR_SER_ | (2UL << CR_PSIZE_POS) |
                 ((uint32_t)(snb & 0x0FU) << CR_SNB_POS);//编程宽度设置为 **32bit**,扇区号（0~15，F4 只用 0~11），放到 SNB 域，指定要擦哪一个扇区
    FLASH->CR |= CR_STRT_;
    while (FLASH->SR & SR_BSY_);

    if (FLASH->SR & (SR_ERR_MASK_ & ~SR_EOP_)) err = -1;

    FLASH->CR &= ~CR_SER_;
    FLASH->SR  = SR_ERR_MASK_;
    flash_lock();
    return err;
}

static int flash_write_words(uint32_t addr, const uint32_t *data, uint32_t words)
{
    int err = 0;

    flash_unlock();
    while (FLASH->SR & SR_BSY_);
    FLASH->SR = SR_ERR_MASK_;

    FLASH->CR &= ~((0x3UL << CR_PSIZE_POS) | CR_SER_ | CR_PG_);
    FLASH->CR |= CR_PG_ | (2UL << CR_PSIZE_POS);          /* x32 编程 */

    while (words--)
    {
        *(volatile uint32_t *)addr = *data++;
        while (FLASH->SR & SR_BSY_);
        if (FLASH->SR & (SR_ERR_MASK_ & ~SR_EOP_)) { err = -1; break; }
        addr += 4U;
    }

    FLASH->CR &= ~CR_PG_;
    FLASH->SR  = SR_ERR_MASK_;
    flash_lock();
    return err;
}
//读取片内 Flash，按 32 位字读取，把 Flash 的数据拷贝到内存缓冲区。
static void flash_read(uint32_t addr, uint32_t *buf, uint32_t words)
{//把 Flash 地址转成指针；加`volatile`，**阻止编译器优化读取**，保证每次都真的去读 Flash，而不是用寄存器缓存的值。
    const volatile uint32_t *p = (const volatile uint32_t *)addr;
    while (words--) *buf++ = *p++;
}

/* ============================================================================
 *  SPI1 + W25Q128
 * ========================================================================== */

static void spi1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* PB3/PB4/PB5 -> AF5 */
    const uint8_t pins[3] = { 3U, 4U, 5U };
    for (int i = 0; i < 3; i++)
    {
        uint32_t s = (uint32_t)pins[i] * 2U;
        GPIOB->MODER   &= ~(3UL << s);
        GPIOB->MODER   |=  (2UL << s);
        GPIOB->OSPEEDR |=  (3UL << s);
        GPIOB->PUPDR   &= ~(3UL << s);
        GPIOB->AFR[0]  &= ~(0xFUL << (pins[i] * 4U));
        GPIOB->AFR[0]  |=  (5UL << (pins[i] * 4U));
    }

    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR_ | SPI_CR1_SSM_ | SPI_CR1_SSI_ |
                (3UL << SPI_CR1_BR_POS) |                     /* fPCLK/16 ≈ 5.25MHz */
                SPI_CR1_SPE_;

    /* CS = PB14，普通推挽输出，默认拉高 */
    GPIOB->MODER   &= ~(3UL << (14U * 2U));
    GPIOB->MODER   |=  (1UL << (14U * 2U));
    GPIOB->OTYPER  &= ~(1UL << 14);
    GPIOB->OSPEEDR |=  (3UL << (14U * 2U));
    GPIOB->ODR     |=  (1UL << 14);
}

static void cs_low(void)  { GPIOB->ODR &= ~(1UL << 14); }
static void cs_high(void) { GPIOB->ODR |=  (1UL << 14); }

static uint8_t spi_xfer(uint8_t d)
{
    while (!(SPI1->SR & SPI_SR_TXE_));
    *(volatile uint8_t *)&SPI1->DR = d;
    while (!(SPI1->SR & SPI_SR_RXNE_));
    return *(volatile uint8_t *)&SPI1->DR;
}

static void w25q_wait_busy(void)
{
    uint8_t sr;
    cs_low();
    spi_xfer(W25Q_CMD_RDSR);
    do { sr = spi_xfer(0xFF); } while (sr & 0x01);       /* 等 BUSY 位清零 */
    cs_high();
}

static void w25q_write_enable(void)
{
    cs_low();
    spi_xfer(W25Q_CMD_WREN);
    cs_high();
}

static void w25q_read(uint32_t a, uint8_t *b, uint32_t n)
{
    cs_low();
    spi_xfer(W25Q_CMD_READ);
    spi_xfer((uint8_t)(a >> 16));
    spi_xfer((uint8_t)(a >> 8));
    spi_xfer((uint8_t)a);
    while (n--) *b++ = spi_xfer(0xFF);
    cs_high();
}

static void w25q_write(uint32_t a, const uint8_t *b, uint32_t n)
{
    while (n)
    {
        uint32_t off = a % W25Q_PAGE_SIZE;
        uint32_t k   = W25Q_PAGE_SIZE - off;
        if (k > n) k = n;

        w25q_write_enable();
        cs_low();
        spi_xfer(W25Q_CMD_PAGEPROG);
        spi_xfer((uint8_t)(a >> 16));
        spi_xfer((uint8_t)(a >> 8));
        spi_xfer((uint8_t)a);
        for (uint32_t i = 0; i < k; i++) spi_xfer(b[i]);
        cs_high();
        w25q_wait_busy();

        a += k; b += k; n -= k;
    }
}

static void w25q_erase_sector(uint32_t a)
{
    w25q_write_enable();
    cs_low();
    spi_xfer(W25Q_CMD_SECTOR_ER);
    spi_xfer((uint8_t)(a >> 16));
    spi_xfer((uint8_t)(a >> 8));
    spi_xfer((uint8_t)a);
    cs_high();
    w25q_wait_busy();
}

static int w25q_check(void)
{
    uint8_t id[3];

    cs_low();
    spi_xfer(W25Q_CMD_JEDEC);
    id[0] = spi_xfer(0xFF);
    id[1] = spi_xfer(0xFF);
    id[2] = spi_xfer(0xFF);
    cs_high();

    uart_puts("[W25Q] ID = ");
    uart_puthex(id[0]); uart_puts(" ");
    uart_puthex(id[1]); uart_puts(" ");
    uart_puthex(id[2]); uart_puts("\r\n");

    return (id[0] == 0xEFU) ? 0 : -1;                     /* Winbond 厂商码 */
}

/* ============================================================================
 *  标志区 / 断点区
 * ========================================================================== */

static int flag_load(void)
{
    flash_read(OTA_FLAG_ADDR, (uint32_t *)&g_flag, sizeof(g_flag) / 4U);

    if (g_flag.magic != OTA_MAGIC) return -1;
    if (crc32_calc(&g_flag, sizeof(g_flag) - 4U) != g_flag.self_crc) return -2;
    return 0;
}

static int flag_save(void)
{
    g_flag.magic    = OTA_MAGIC;
    g_flag.self_crc = crc32_calc(&g_flag, sizeof(g_flag) - 4U);

    if (flash_erase_sector(FLAG_SECTOR) != 0) return -1;
    return flash_write_words(OTA_FLAG_ADDR, (uint32_t *)&g_flag, sizeof(g_flag) / 4U);
}

static void ckpt_save(void)
{
    g_ck.magic    = CKPT_MAGIC;
    g_ck.self_crc = crc32_calc(&g_ck, sizeof(g_ck) - 4U);

    w25q_erase_sector(W25Q_CKPT_ADDR);
    w25q_write(W25Q_CKPT_ADDR, (const uint8_t *)&g_ck, sizeof(g_ck));
}

static int ckpt_load(void)
{
    w25q_read(W25Q_CKPT_ADDR, (uint8_t *)&g_ck, sizeof(g_ck));

    if (g_ck.magic != CKPT_MAGIC) return -1;
    if (crc32_calc(&g_ck, sizeof(g_ck) - 4U) != g_ck.self_crc) return -2;
    return 0;
}

/* ============================================================================
 *  分区有效性 + 跳转
 * ========================================================================== */

static int partition_is_valid(uint32_t a)
{
    uint32_t sp = *(volatile uint32_t *)a;          /* 向量表第 0 项：初始 SP */
    uint32_t pc = *(volatile uint32_t *)(a + 4U);   /* 向量表第 1 项：复位向量 */

    if ((sp == 0xFFFFFFFFUL) || (pc == 0xFFFFFFFFUL)) return 0;   /* 空白 Flash */
    if ((sp < 0x20000000UL) || (sp > 0x20040000UL))   return 0;   /* SP 不在 RAM */
    if ((pc < a) || (pc >= (a + APP_SIZE)))           return 0;   /* 向量不在本分区 */
    return 1;
}

static int jump_to_app(uint32_t a)
{
    uint32_t sp = *(volatile uint32_t *)a;
    uint32_t pc = *(volatile uint32_t *)(a + 4U);

    if (!partition_is_valid(a)) return -1;

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (int i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    __set_MSP(sp);
    SCB->VTOR = a;
    ((pFunc)pc)();

    return -1;                                      /* 不会执行到这里 */
}

/* ============================================================================
 *  OTA 应用：擦目标分区 -> 从 W25Q 搬运 -> 校验
 * ========================================================================== */

static int ota_apply_to_partition(uint32_t part)
{
    uint32_t dst    = (part == PART_B) ? APP_B_ADDR : APP_A_ADDR;
    uint8_t  sector = (part == PART_B) ? APP_B_SECTOR : APP_A_SECTOR;
    static uint8_t   buf[256];
    uint32_t total  = g_ck.total_size;
    uint32_t done   = 0;

    if (total == 0U || total > APP_SIZE) return -1;
    if (part == g_flag.active_part)      return -3;   /* 安全锁：绝不擦运行分区 */

    uart_puts("[OTA] apply part=");
    uart_putu(part);
    uart_puts(" dst=");
    uart_puthex(dst);
    uart_puts(" sector=");
    uart_putu((uint32_t)sector);
    uart_puts(" size=");
    uart_putu(total);
    uart_puts("\r\n");

    for (uint8_t i = 0; i < APP_SECTORS; i++)
        flash_erase_sector((uint8_t)(sector + i));

    uart_puts("[OTA] copying...\r\n");

    while (done < total)
    {
        uint32_t k = total - done;
        if (k > sizeof(buf)) k = sizeof(buf);

        w25q_read(W25Q_STAGE_ADDR + done, buf, k);

        if (flash_write_words(dst + done, (const uint32_t *)buf, (k + 3U) / 4U) != 0)
        {
            uart_puts("[OTA] flash write FAILED\r\n");
            return -2;
        }
        done += k;
    }

    return 0;
}

/* 搬运 + 切标志 + 复位（成功时不返回） */
static int boot_apply_and_switch(void)
{
    int r = ota_apply_to_partition(g_flag.pending_part);

    if (r != 0)
    {
        uart_puts("[OTA] apply FAILED r=");
        uart_putu((uint32_t)(-r));
        uart_puts("\r\n");
        g_flag.pending_part = PART_NONE;
        flag_save();
        return -1;
    }

    if (g_flag.pending_part == PART_A) g_flag.version_a = g_ck.version;
    else                               g_flag.version_b = g_ck.version;

    g_flag.active_part  = g_flag.pending_part;
    g_flag.pending_part = PART_NONE;
    flag_save();

    uart_puts("[OTA] activated, rebooting...\r\n");
    delay_ms(300);
    NVIC_SystemReset();

    return -1;                                      /* 不会执行到这里 */
}

/* ============================================================================
 *  OTA 协议
 * ========================================================================== */

static void send_ack(uint8_t cmd, uint16_t seq)
{
    uint8_t f[8];
    uint16_t c;

    f[0] = FRAME_HEAD0; f[1] = FRAME_HEAD1;
    f[2] = RSP_ACK;     f[3] = cmd;
    f[4] = (uint8_t)(seq >> 8); f[5] = (uint8_t)seq;

    c = crc16_calc(&f[2], 4);
    f[6] = (uint8_t)(c >> 8);
    f[7] = (uint8_t)(c & 0xFF);

    for (int i = 0; i < 8; i++) uart_putb(f[i]);
}

static void send_nak(uint8_t cmd, uint16_t seq, uint8_t err)
{
    uint8_t f[7];

    f[0] = FRAME_HEAD0; f[1] = FRAME_HEAD1;
    f[2] = RSP_NAK;     f[3] = cmd;
    f[4] = (uint8_t)(seq >> 8); f[5] = (uint8_t)seq;
    f[6] = err;

    for (int i = 0; i < 7; i++) uart_putb(f[i]);
}

static int ota_loop(void)
{
    uint8_t        cmd, seq_h, seq_l, len_h, len_l, bad;
    static uint8_t data[MAX_PAYLOAD];
    uint16_t       seq, len, crc_rx, crc_calc;
    int            c;

    uart_puts("[OTA] waiting for BEGIN...\r\n");

    for (;;)
    {
        /* ---- 找帧头 ---- */
        c = uart_getc_timeout(8000);
        if (c < 0) return -1;
        if (c != FRAME_HEAD0) continue;

        c = uart_getc_timeout(50);
        if (c != FRAME_HEAD1) continue;

        /* ---- 定长头：CMD + SEQ(2) + LEN(2) ---- */
        c = uart_getc_timeout(50); if (c < 0) continue; cmd = (uint8_t)c;
        c = uart_getc_timeout(50); if (c < 0) continue; seq_h = (uint8_t)c;
        c = uart_getc_timeout(50); if (c < 0) continue; seq_l = (uint8_t)c;
        c = uart_getc_timeout(50); if (c < 0) continue; len_h = (uint8_t)c;
        c = uart_getc_timeout(50); if (c < 0) continue; len_l = (uint8_t)c;

        seq = (uint16_t)(((uint16_t)seq_h << 8) | (uint16_t)seq_l);
        len = (uint16_t)(((uint16_t)len_h << 8) | (uint16_t)len_l);

        if (len > MAX_PAYLOAD) { send_nak(cmd, seq, 1); continue; }

        /* ---- 数据 ---- */
        bad = 0;
        for (uint16_t i = 0; i < len; i++)
        {
            c = uart_getc_timeout(200);
            if (c < 0) { bad = 1; break; }
            data[i] = (uint8_t)c;
        }
        if (bad) continue;

        /* ---- 帧 CRC ---- */
        c = uart_getc_timeout(50); if (c < 0) continue; crc_rx  = (uint16_t)((uint8_t)c << 8);
        c = uart_getc_timeout(50); if (c < 0) continue; crc_rx |= (uint16_t)(uint8_t)c;

        {
            static uint8_t tmp[5 + MAX_PAYLOAD];
            tmp[0] = cmd;
            tmp[1] = (uint8_t)(seq >> 8);
            tmp[2] = (uint8_t)seq;
            tmp[3] = len_h;
            tmp[4] = len_l;
            for (uint16_t i = 0; i < len; i++) tmp[5 + i] = data[i];
            crc_calc = crc16_calc(tmp, 5U + len);
        }
        if (crc_calc != crc_rx) { send_nak(cmd, seq, 2); continue; }

        /* ---- 命令分派 ---- */
        switch (cmd)
        {
        case CMD_BEGIN:
            if (len < 8U) { send_nak(cmd, seq, 3); break; }

            g_ck.total_size = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                              ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            g_ck.version    = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                              ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
            g_ck.downloaded = 0;
            g_ck.src_type   = 1;                     /* UART */
            g_ck.magic      = CKPT_MAGIC;
            g_last_sector   = 0xFFFFFFFFUL;

            if (g_ck.total_size == 0U || g_ck.total_size > APP_SIZE)
            { send_nak(cmd, seq, 4); break; }

            g_flag.pending_part = (g_flag.active_part == PART_B) ? PART_A : PART_B;
            flag_save();
            ckpt_save();

            uart_puts("[OTA] BEGIN size=");
            uart_putu(g_ck.total_size);
            uart_puts(" target=");
            uart_putu(g_flag.pending_part);
            uart_puts(" ver=");
            uart_puthex(g_ck.version);
            uart_puts("\r\n");

            send_ack(cmd, seq);
            break;

        case CMD_DATA:
        {
            uint32_t start, end, s0, s1;

            if (g_ck.magic != CKPT_MAGIC) { send_nak(cmd, seq, 5); break; }

            start = W25Q_STAGE_ADDR + g_ck.downloaded;
            end   = start + (uint32_t)len - 1U;
            s0    = start & ~(W25Q_SECTOR_SIZE - 1U);
            s1    = end   & ~(W25Q_SECTOR_SIZE - 1U);

            /* 进入新扇区先擦除（Flash 只能 1->0） */
            if (s0 != g_last_sector) { w25q_erase_sector(s0); g_last_sector = s0; }
            if ((s1 != s0) && (s1 != g_last_sector))
            { w25q_erase_sector(s1); g_last_sector = s1; }

            w25q_write(start, data, len);
            g_ck.downloaded += len;

            /* 每跨 4KB 存一次断点 */
            if ((g_ck.downloaded % W25Q_SECTOR_SIZE) < len) ckpt_save();

            if ((seq & 0x1FU) == 0U)
            {
                uart_puts("[OTA] ");
                uart_putu(g_ck.downloaded);
                uart_puts(" / ");
                uart_putu(g_ck.total_size);
                uart_puts("\r\n");
            }

            send_ack(cmd, seq);
            break;
        }

        case CMD_END:
        {
            uint32_t done = 0, crc = 0xFFFFFFFFUL;
            uint32_t expect;
            static uint8_t cbuf[512];

            if (len < 4U) { send_nak(cmd, seq, 6); break; }

            expect = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);

            uart_puts("[OTA] verify crc32...\r\n");

            while (done < g_ck.total_size)
            {
                uint32_t k = g_ck.total_size - done;
                if (k > sizeof(cbuf)) k = sizeof(cbuf);
                w25q_read(W25Q_STAGE_ADDR + done, cbuf, k);

                for (uint32_t i = 0; i < k; i++)
                {
                    crc ^= cbuf[i];
                    for (int b = 0; b < 8; b++)
                        crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
                }
                done += k;
            }
            crc = ~crc;

            if (crc != expect)
            {
                uart_puts("[OTA] CRC FAIL exp=");
                uart_puthex(expect);
                uart_puts(" got=");
                uart_puthex(crc);
                uart_puts("\r\n");
                send_nak(cmd, seq, 7);
                g_ck.magic = 0;
                break;
            }

            uart_puts("[OTA] CRC OK\r\n");
            ckpt_save();
            send_ack(cmd, seq);
            break;
        }

        case CMD_ACTIVATE:
            if (g_ck.magic != CKPT_MAGIC) { send_nak(cmd, seq, 8); break; }
            send_ack(cmd, seq);
            delay_ms(100);
            boot_apply_and_switch();
            break;

        case CMD_QUERY:
            uart_puts("[OTA] active=");
            uart_putu(g_flag.active_part);
            uart_puts(" pending=");
            uart_putu(g_flag.pending_part);
            uart_puts(" downloaded=");
            uart_putu(g_ck.downloaded);
            uart_puts("/");
            uart_putu(g_ck.total_size);
            uart_puts("\r\n");
            send_ack(cmd, seq);
            break;

        default:
            send_nak(cmd, seq, 0xFF);
            break;
        }
    }
}

/* ============================================================================
 *  主函数
 * ========================================================================== */

int main(void)
{
    uint32_t target = 0;
    int a_ok, b_ok, c;

    SystemCoreClockUpdate();

    led_init();
    uart_init();
    spi1_init();

    uart_puts("\r\n\r\n");
    uart_puts("=== STM32F407 OTA Bootloader v1.1 ===\r\n");
    uart_puts("[BL] APB2=");
    uart_putu(get_pclk2() / 1000000UL);
    uart_puts("MHz  BRR=");
    uart_puthex(USART1->BRR);
    uart_puts("\r\n");

    if (w25q_check() != 0)
        uart_puts("[BL] W25Q NOT FOUND!\r\n");

    /* ---- 标志区：坏了就写默认值 ---- */
    if (flag_load() != 0)
    {
        uart_puts("[BL] flag invalid, init default\r\n");
        g_flag.magic        = OTA_MAGIC;
        g_flag.active_part  = PART_A;
        g_flag.pending_part = PART_NONE;
        g_flag.version_a    = 0;
        g_flag.version_b    = 0;
        flag_save();
        flag_load();
    }

    uart_puts("[BL] active=");
    uart_putu(g_flag.active_part);
    uart_puts(" pending=");
    uart_putu(g_flag.pending_part);
    uart_puts(" verA=");
    uart_puthex(g_flag.version_a);
    uart_puts(" verB=");
    uart_puthex(g_flag.version_b);
    uart_puts("\r\n");

    /* ---- 上次没搬完的升级任务 ---- */
    if (g_flag.pending_part != PART_NONE)
    {
        uart_puts("[BL] pending upgrade found\r\n");
        if (ckpt_load() == 0)
        {
            boot_apply_and_switch();                /* 失败会自动清 pending */
        }
        else
        {
            uart_puts("[BL] checkpoint invalid, clear pending\r\n");
            g_flag.pending_part = PART_NONE;
            flag_save();
        }
    }

    /* ---- 10 秒 OTA 窗口 ---- */
    uart_puts("[BL] listen for OTA (10s)...\r\n");
    c = uart_getc_timeout(10000);
    if (c == FRAME_HEAD0)
    {
        c = uart_getc_timeout(100);
        if (c == FRAME_HEAD1)
        {
            uart_puts("[BL] OTA mode!\r\n");
            ota_loop();
            NVIC_SystemReset();
        }
    }

    /* ---- 正常启动 ---- */
    a_ok = partition_is_valid(APP_A_ADDR);
    b_ok = partition_is_valid(APP_B_ADDR);

    uart_puts("[BL] A=");
    uart_putu((uint32_t)a_ok);
    uart_puts(" B=");
    uart_putu((uint32_t)b_ok);
    uart_puts("\r\n");

    if (a_ok && b_ok)
        target = (g_flag.active_part == PART_B) ? APP_B_ADDR : APP_A_ADDR;
    else if (a_ok)
        target = APP_A_ADDR;
    else if (b_ok)
        target = APP_B_ADDR;
    else
    {
        uart_puts("[BL] NO valid app, wait OTA\r\n");
        for (;;) { led_blink(LED1_PIN, 8, 150, 150); delay_ms(1000); }
    }

    uart_puts("[BL] jump ");
    uart_puthex(target);
    uart_puts("\r\n");

    led_blink(LED0_PIN, 2, 200, 200);
    jump_to_app(target);

    /* 跳转失败才会到这里 */
    for (;;) { led_blink(LED1_PIN, 9, 100, 100); delay_ms(1000); }
}
