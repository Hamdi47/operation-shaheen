/*
 * stm32f411xx.h — Bare Metal Register Definitions
 * Target: STM32F411CEU6
 *
 * This replaces ST's CMSIS headers. It defines only what we need:
 *   - FLASH  (wait state configuration)
 *   - RCC    (clock control and enable)
 *   - GPIOC  (PC13 = onboard LED)
 *   - TIM2   (hardware timer for 1 Hz interrupt)
 *   - NVIC   (interrupt controller)
 *
 * Every register is declared as `volatile` because:
 *   - The hardware can change them at any time (interrupt flags, etc.)
 *   - Without volatile, the compiler may cache the value in a CPU register
 *     and miss hardware updates, causing silent bugs.
 *
 * Reference: RM0383 — STM32F411xC/E Reference Manual
 */

#ifndef STM32F411XX_H
#define STM32F411XX_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════════════════════
 * Bus and peripheral base addresses
 * Source: RM0383, Section 2.3 "Memory map"
 * ══════════════════════════════════════════════════════════════════════ */
#define PERIPH_BASE     0x40000000UL
#define APB1_BASE       (PERIPH_BASE + 0x00000000UL)
#define APB2_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1_BASE       (PERIPH_BASE + 0x00020000UL)

#define TIM2_BASE       (APB1_BASE + 0x00000000UL)   /* APB1 offset 0x0000 */
#define GPIOA_BASE      (AHB1_BASE + 0x00000000UL)   /* AHB1 offset 0x0000 */
#define GPIOB_BASE      (AHB1_BASE + 0x00000400UL)   /* AHB1 offset 0x0400 */
#define GPIOC_BASE      (AHB1_BASE + 0x00000800UL)   /* AHB1 offset 0x0800 */
#define RCC_BASE        (AHB1_BASE + 0x00003800UL)   /* AHB1 offset 0x3800 */
#define FLASH_R_BASE    (AHB1_BASE + 0x00003C00UL)   /* AHB1 offset 0x3C00 */

#define NVIC_BASE       0xE000E100UL   /* ARM Cortex-M4 core — fixed address */


/* ══════════════════════════════════════════════════════════════════════
 * FLASH — Access Control Register
 * Must be configured BEFORE raising the CPU clock.
 * At 100 MHz / 3.3V → 3 wait states required (RM0383, Table 6).
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct
{
    volatile uint32_t ACR;       /* 0x00 - Access Control Register */
    volatile uint32_t KEYR;      /* 0x04 - Key Register            */
    volatile uint32_t OPTKEYR;   /* 0x08 - Option Key Register     */
    volatile uint32_t SR;        /* 0x0C - Status Register         */
    volatile uint32_t CR;        /* 0x10 - Control Register        */
    volatile uint32_t OPTCR;     /* 0x14 - Option Control Register */
} FLASH_TypeDef;

#define FLASH  ((FLASH_TypeDef *)FLASH_R_BASE)

/* FLASH_ACR bits */
#define FLASH_ACR_LATENCY_Pos   0U
#define FLASH_ACR_LATENCY_Msk   (0xFUL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_PRFTEN        (1UL << 8)    /* Prefetch enable          */
#define FLASH_ACR_ICEN          (1UL << 9)    /* Instruction cache enable */
#define FLASH_ACR_DCEN          (1UL << 10)   /* Data cache enable        */


/* ══════════════════════════════════════════════════════════════════════
 * RCC — Reset and Clock Control
 * Source: RM0383, Section 6
 *
 * PLL Clock Tree for 100 MHz:
 *
 *   HSE (25 MHz)
 *      │
 *      ├─ /PLLM (25) ──► 1 MHz   (VCO input — must be 1–2 MHz)
 *      │       │
 *      │    x PLLN (400) ──► 400 MHz  (VCO output — must be 192–432 MHz)
 *      │       │
 *      │    /PLLP (4) ──► 100 MHz  ──► SYSCLK
 *      │    /PLLQ (8) ──►  50 MHz  ──► USB/SDIO (not used now)
 *      │
 *   SYSCLK = 100 MHz
 *      │
 *      ├─ /AHB  (1) ──► HCLK  = 100 MHz  (CPU, DMA, Flash)
 *      ├─ /APB1 (2) ──► PCLK1 =  50 MHz  (TIM2, I2C, SPI2/3, UART2)
 *      └─ /APB2 (1) ──► PCLK2 = 100 MHz  (SPI1, UART1, TIM1)
 *
 * NOTE: When APB1 prescaler != 1, TIM2 clock = PCLK1 * 2 = 100 MHz.
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct
{
    volatile uint32_t CR;           /* 0x00 - Clock control register          */
    volatile uint32_t PLLCFGR;      /* 0x04 - PLL configuration register      */
    volatile uint32_t CFGR;         /* 0x08 - Clock configuration register    */
    volatile uint32_t CIR;          /* 0x0C - Clock interrupt register        */
    volatile uint32_t AHB1RSTR;     /* 0x10 - AHB1 peripheral reset           */
    volatile uint32_t AHB2RSTR;     /* 0x14 - AHB2 peripheral reset           */
    uint32_t          RESERVED0[2]; /* 0x18–0x1C                              */
    volatile uint32_t APB1RSTR;     /* 0x20 - APB1 peripheral reset           */
    volatile uint32_t APB2RSTR;     /* 0x24 - APB2 peripheral reset           */
    uint32_t          RESERVED1[2]; /* 0x28–0x2C                              */
    volatile uint32_t AHB1ENR;      /* 0x30 - AHB1 peripheral clock enable    */
    volatile uint32_t AHB2ENR;      /* 0x34 - AHB2 peripheral clock enable    */
    uint32_t          RESERVED2[2]; /* 0x38–0x3C                              */
    volatile uint32_t APB1ENR;      /* 0x40 - APB1 peripheral clock enable    */
    volatile uint32_t APB2ENR;      /* 0x44 - APB2 peripheral clock enable    */
    uint32_t          RESERVED3[2]; /* 0x48–0x4C                              */
    volatile uint32_t AHB1LPENR;    /* 0x50 - AHB1 low-power clock enable     */
    volatile uint32_t AHB2LPENR;    /* 0x54 - AHB2 low-power clock enable     */
    uint32_t          RESERVED4[2]; /* 0x58–0x5C                              */
    volatile uint32_t APB1LPENR;    /* 0x60 - APB1 low-power clock enable     */
    volatile uint32_t APB2LPENR;    /* 0x64 - APB2 low-power clock enable     */
    uint32_t          RESERVED5[2]; /* 0x68–0x6C                              */
    volatile uint32_t BDCR;         /* 0x70 - Backup domain control           */
    volatile uint32_t CSR;          /* 0x74 - Clock control & status          */
    uint32_t          RESERVED6[2]; /* 0x78–0x7C                              */
    volatile uint32_t SSCGR;        /* 0x80 - Spread spectrum clock gen       */
    volatile uint32_t PLLI2SCFGR;   /* 0x84 - PLLI2S configuration            */
    uint32_t          RESERVED7;    /* 0x88                                   */
    volatile uint32_t DCKCFGR;      /* 0x8C - Dedicated clocks configuration  */
} RCC_TypeDef;

#define RCC  ((RCC_TypeDef *)RCC_BASE)

/* RCC_CR bits */
#define RCC_CR_HSION    (1UL << 0)    /* HSI oscillator enable            */
#define RCC_CR_HSIRDY   (1UL << 1)    /* HSI oscillator ready flag        */
#define RCC_CR_HSEON    (1UL << 16)   /* HSE oscillator enable            */
#define RCC_CR_HSERDY   (1UL << 17)   /* HSE oscillator ready flag        */
#define RCC_CR_PLLON    (1UL << 24)   /* Main PLL enable                  */
#define RCC_CR_PLLRDY   (1UL << 25)   /* Main PLL ready flag              */

/* RCC_PLLCFGR bits
 *   PLLM  [5:0]   divide HSE input (25 MHz / 25 = 1 MHz)
 *   PLLN  [14:6]  VCO multiplier   (1 MHz  x 400 = 400 MHz)
 *   PLLP  [17:16] SYSCLK divider   (00=/2, 01=/4, 10=/6, 11=/8)
 *   PLLSRC [22]   clock source     (0=HSI, 1=HSE)
 *   PLLQ  [27:24] USB/SDIO divider (400 MHz / 8 = 50 MHz)
 */
#define RCC_PLLCFGR_PLLSRC_HSE  (1UL << 22)

/* RCC_CFGR bits */
#define RCC_CFGR_SW_PLL      (0x2UL << 0)    /* Select PLL as system clock  */
#define RCC_CFGR_SWS_PLL     (0x2UL << 2)    /* PLL is current system clock */
#define RCC_CFGR_SWS_Msk     (0x3UL << 2)    /* SWS field mask              */
#define RCC_CFGR_HPRE_DIV1   (0x0UL << 4)    /* AHB  prescaler = /1         */
#define RCC_CFGR_PPRE1_DIV2  (0x4UL << 10)   /* APB1 prescaler = /2         */
#define RCC_CFGR_PPRE2_DIV1  (0x0UL << 13)   /* APB2 prescaler = /1         */

/* RCC_AHB1ENR — enable GPIO port clocks */
#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN  (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN  (1UL << 2)   /* GPIOC — onboard LED PC13 */

/* RCC_APB1ENR — enable timer clocks */
#define RCC_APB1ENR_TIM2EN   (1UL << 0)   /* TIM2 clock enable */


/* ══════════════════════════════════════════════════════════════════════
 * GPIO — General Purpose I/O
 * Source: RM0383, Section 7
 *
 * MODER: 2 bits per pin →  00=Input  01=Output  10=AF  11=Analog
 * OTYPER: 1 bit per pin →  0=Push-pull  1=Open-drain
 * OSPEEDR: 2 bits per pin → 00=Low  01=Medium  10=Fast  11=High
 * PUPDR: 2 bits per pin →  00=None  01=Pull-up  10=Pull-down
 * BSRR: bits[15:0]=Set, bits[31:16]=Reset (atomic, no read needed)
 * ODR: current output state — XOR to toggle
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct
{
    volatile uint32_t MODER;     /* 0x00 - Mode register           */
    volatile uint32_t OTYPER;    /* 0x04 - Output type register    */
    volatile uint32_t OSPEEDR;   /* 0x08 - Output speed register   */
    volatile uint32_t PUPDR;     /* 0x0C - Pull-up/down register   */
    volatile uint32_t IDR;       /* 0x10 - Input data register     */
    volatile uint32_t ODR;       /* 0x14 - Output data register    */
    volatile uint32_t BSRR;      /* 0x18 - Bit set/reset register  */
    volatile uint32_t LCKR;      /* 0x1C - Configuration lock      */
    volatile uint32_t AFR[2];    /* 0x20 - Alt function [0]=low [1]=high */
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *)GPIOC_BASE)

/* GPIO MODER field values (2 bits per pin) */
#define GPIO_MODE_INPUT   0x0U
#define GPIO_MODE_OUTPUT  0x1U
#define GPIO_MODE_AF      0x2U
#define GPIO_MODE_ANALOG  0x3U

/* PC13 — onboard LED (active LOW: 0=on, 1=off)
 * PC13 occupies bits [27:26] in MODER, OSPEEDR, PUPDR */
#define GPIO_MODER_PC13_Pos    26U
#define GPIO_MODER_PC13_Msk    (0x3UL << GPIO_MODER_PC13_Pos)
#define GPIO_OSPEEDR_PC13_Pos  26U
#define GPIO_OSPEEDR_PC13_Msk  (0x3UL << GPIO_OSPEEDR_PC13_Pos)
#define GPIO_PUPDR_PC13_Pos    26U
#define GPIO_PUPDR_PC13_Msk    (0x3UL << GPIO_PUPDR_PC13_Pos)

/* BSRR helpers for PC13 */
#define GPIO_BSRR_BS13   (1UL << 13)   /* Set   PC13 = 1 (LED off) */
#define GPIO_BSRR_BR13   (1UL << 29)   /* Reset PC13 = 0 (LED on)  */

/* ODR bit for PC13 — use with XOR to toggle */
#define GPIO_ODR_OD13    (1UL << 13)


/* ══════════════════════════════════════════════════════════════════════
 * TIM2 — 32-bit General Purpose Timer on APB1
 * Source: RM0383, Section 15
 *
 * Update frequency formula:
 *   f_update = TIM_clock / ((PSC + 1) * (ARR + 1))
 *
 * For 1 Hz at TIM_clock = 100 MHz:
 *   PSC = 9999  → 100 MHz / 10 000 = 10 kHz
 *   ARR = 9999  → 10 kHz  / 10 000 = 1 Hz
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct
{
    volatile uint32_t CR1;        /* 0x00 - Control register 1              */
    volatile uint32_t CR2;        /* 0x04 - Control register 2              */
    volatile uint32_t SMCR;       /* 0x08 - Slave mode control              */
    volatile uint32_t DIER;       /* 0x0C - DMA/Interrupt enable            */
    volatile uint32_t SR;         /* 0x10 - Status register                 */
    volatile uint32_t EGR;        /* 0x14 - Event generation                */
    volatile uint32_t CCMR1;      /* 0x18 - Capture/compare mode 1          */
    volatile uint32_t CCMR2;      /* 0x1C - Capture/compare mode 2          */
    volatile uint32_t CCER;       /* 0x20 - Capture/compare enable          */
    volatile uint32_t CNT;        /* 0x24 - Counter value                   */
    volatile uint32_t PSC;        /* 0x28 - Prescaler                       */
    volatile uint32_t ARR;        /* 0x2C - Auto-reload register            */
    uint32_t          RESERVED0;  /* 0x30 - (no RCR on TIM2–TIM5)          */
    volatile uint32_t CCR1;       /* 0x34 - Capture/compare register 1      */
    volatile uint32_t CCR2;       /* 0x38 - Capture/compare register 2      */
    volatile uint32_t CCR3;       /* 0x3C - Capture/compare register 3      */
    volatile uint32_t CCR4;       /* 0x40 - Capture/compare register 4      */
    uint32_t          RESERVED1;  /* 0x44 - (no BDTR on TIM2–TIM5)         */
    volatile uint32_t DCR;        /* 0x48 - DMA control                     */
    volatile uint32_t DMAR;       /* 0x4C - DMA address                     */
    volatile uint32_t OR;         /* 0x50 - Option register                 */
} TIM_TypeDef;

#define TIM2  ((TIM_TypeDef *)TIM2_BASE)

/* TIM_CR1 bits */
#define TIM_CR1_CEN    (1UL << 0)   /* Counter enable (starts counting)   */
#define TIM_CR1_ARPE   (1UL << 7)   /* Auto-reload preload enable         */

/* TIM_DIER bits */
#define TIM_DIER_UIE   (1UL << 0)   /* Update interrupt enable            */

/* TIM_SR bits */
#define TIM_SR_UIF     (1UL << 0)   /* Update interrupt flag (clear by writing 0) */

/* TIM_EGR bits */
#define TIM_EGR_UG     (1UL << 0)   /* Force update event — loads PSC/ARR into shadow registers */


/* ══════════════════════════════════════════════════════════════════════
 * NVIC — Nested Vectored Interrupt Controller
 * Source: ARMv7-M Architecture Reference Manual, Section B3.4
 *
 * The NVIC is part of the Cortex-M4 core, not STM32-specific.
 * It lives at a fixed address in the Private Peripheral Bus (PPB).
 *
 * ISER[n]: each bit enables one IRQ. IRQn → ISER[n/32] bit (n%32).
 * IP[n]:   priority byte for IRQn. STM32F411 uses 4 priority bits
 *          in the upper nibble → shift priority value left by 4.
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct
{
    volatile uint32_t ISER[8];    /* 0x000 - Interrupt Set Enable Registers    */
    uint32_t          RSVD0[24];
    volatile uint32_t ICER[8];    /* 0x080 - Interrupt Clear Enable Registers  */
    uint32_t          RSVD1[24];
    volatile uint32_t ISPR[8];    /* 0x100 - Interrupt Set Pending Registers   */
    uint32_t          RSVD2[24];
    volatile uint32_t ICPR[8];    /* 0x180 - Interrupt Clear Pending Registers */
    uint32_t          RSVD3[24];
    volatile uint32_t IABR[8];    /* 0x200 - Interrupt Active Bit Registers    */
    uint32_t          RSVD4[56];
    volatile uint8_t  IP[240];    /* 0x300 - Interrupt Priority Registers      */
    uint32_t          RSVD5[644];
    volatile uint32_t STIR;       /* 0xE00 - Software Trigger Interrupt        */
} NVIC_TypeDef;

#define NVIC  ((NVIC_TypeDef *)NVIC_BASE)

/* IRQ numbers for STM32F411 (RM0383, Table 38) */
#define TIM2_IRQn   28U

/*
 * NVIC_EnableIRQ(irqn)
 *   Enables interrupt number irqn.
 *   IRQ28 (TIM2) → ISER[0] bit 28.
 *
 * NVIC_SetPriority(irqn, prio)
 *   Sets priority. Lower number = higher priority.
 *   STM32F411 has 4 priority bits in the upper nibble of IP[n].
 *   Valid range: 0 (highest) to 15 (lowest).
 */
#define NVIC_EnableIRQ(irqn) \
    (NVIC->ISER[(irqn) >> 5U] = (1UL << ((irqn) & 0x1FU)))

#define NVIC_SetPriority(irqn, prio) \
    (NVIC->IP[(irqn)] = (uint8_t)((prio) << 4U))

#endif /* STM32F411XX_H */
