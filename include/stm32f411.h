/*
 * stm32f411.h
 *
 * Minimal bare-metal register definitions for STM32F411CEU6.
 * Peripherals included: RCC, GPIOA/B/C, USART2, I2C1, SysTick, NVIC, FLASH, SCB.
 *
 * If you already have your own comprehensive stm32f411.h, just make sure
 * it exposes the same struct/macro names used in uart.c / main.c / startup.c
 * (RCC->..., GPIOA->..., GPIOC->..., USART2->..., SysTick->...), or adjust
 * those files to match your header's naming.
 */

#ifndef STM32F411_H
#define STM32F411_H

#include <stdint.h>

/* ------------------------ Utility macros ------------------------ */
#define SET_BIT(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)                   ((REG) & (BIT))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  ((REG) = (((REG) & ~(CLEARMASK)) | (SETMASK)))

/* ----------------------- Base addresses ----------------------- */
#define PERIPH_BASE         0x40000000UL
#define APB1PERIPH_BASE     (PERIPH_BASE)
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000UL)

#define GPIOA_BASE          (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE          (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800UL)
#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800UL)

#define USART2_BASE         (APB1PERIPH_BASE + 0x4400UL)

#define SCS_BASE            0xE000E000UL
#define SYSTICK_BASE        (SCS_BASE + 0x0010UL)
#define NVIC_BASE            (SCS_BASE + 0x0100UL)
#define SCB_BASE    (SCS_BASE + 0x0D00UL)

#define FLASH_BASE          0x40023C00UL

/* ----------------------------- GPIO ----------------------------- */
typedef struct {
volatile uint32_t MODER;
volatile uint32_t OTYPER;
volatile uint32_t OSPEEDR;
volatile uint32_t PUPDR;
volatile uint32_t IDR;
volatile uint32_t ODR;
volatile uint32_t BSRR;
volatile uint32_t LCKR;
volatile uint32_t AFR[2]; /* AFR[0]=AFRL (pins 0-7), AFR[1]=AFRH (pins 8-15) */
} GPIO_TypeDef;

#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)

/* ------------------------------ RCC ------------------------------ */
typedef struct {
volatile uint32_t CR;
volatile uint32_t PLLCFGR;
volatile uint32_t CFGR;
volatile uint32_t CIR;
volatile uint32_t AHB1RSTR;
volatile uint32_t AHB2RSTR;
uint32_t RESERVED0[2];
volatile uint32_t APB1RSTR;
volatile uint32_t APB2RSTR;
uint32_t RESERVED1[2];
volatile uint32_t AHB1ENR;
volatile uint32_t AHB2ENR;
uint32_t RESERVED2[2];
volatile uint32_t APB1ENR;
volatile uint32_t APB2ENR;
uint32_t RESERVED3[2];
volatile uint32_t AHB1LPENR;
volatile uint32_t AHB2LPENR;
uint32_t RESERVED4[2];
volatile uint32_t APB1LPENR;
volatile uint32_t APB2LPENR;
uint32_t RESERVED5[2];
volatile uint32_t BDCR;
volatile uint32_t CSR;
uint32_t RESERVED6[2];
volatile uint32_t SSCGR;
volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;

#define RCC                 ((RCC_TypeDef *)RCC_BASE)

#define RCC_AHB1ENR_GPIOAEN (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN (1UL << 2)
#define RCC_AHB2ENR_OTGFSEN (1UL << 7)
#define RCC_APB1ENR_USART2EN (1UL << 17)
#define RCC_APB1ENR_I2C1EN  (1UL << 21)

/* RCC_CR */
#define RCC_CR_HSION              (1UL << 0)
#define RCC_CR_HSIRDY             (1UL << 1)
#define RCC_CR_HSEON              (1UL << 16)
#define RCC_CR_HSERDY             (1UL << 17)

#define RCC_CR_PLLON              (1UL << 24)
#define RCC_CR_PLLRDY             (1UL << 25)

/* RCC_CFGR */
#define RCC_CFGR_SW_HSI           (0UL << 0)
#define RCC_CFGR_SW_PLL           (2UL << 0)

#define RCC_CFGR_SWS_MASK         (3UL << 2)
#define RCC_CFGR_SWS_HSI          (0UL << 2)
#define RCC_CFGR_SWS_PLL          (2UL << 2)

#define RCC_CFGR_SW_Pos           0U
#define RCC_CFGR_SW_Msk           (3UL << RCC_CFGR_SW_Pos)

#define RCC_CFGR_SWS_Pos          2U
#define RCC_CFGR_SWS_Msk          (3UL << RCC_CFGR_SWS_Pos)

/*PLLM*/
#define RCC_PLLCFGR_PLLM_Pos      0U
#define RCC_PLLCFGR_PLLM_Msk      (0x3FUL << RCC_PLLCFGR_PLLM_Pos)

/*PLLN*/
#define RCC_PLLCFGR_PLLN_Pos      6U
#define RCC_PLLCFGR_PLLN_Msk      (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)

/*PLLP*/
#define RCC_PLLCFGR_PLLP_Pos      16U
#define RCC_PLLCFGR_PLLP_Msk      (3UL << RCC_PLLCFGR_PLLP_Pos)

/*PLL Source*/
#define RCC_PLLCFGR_PLLSRC_HSI    (0UL << 22)
#define RCC_PLLCFGR_PLLSRC_HSE    (1UL << 22)

/*PLLQ*/
#define RCC_PLLCFGR_PLLQ_Pos      24U
#define RCC_PLLCFGR_PLLQ_Msk      (0xFUL << RCC_PLLCFGR_PLLQ_Pos)

/* AHB Prescaler */
#define RCC_CFGR_HPRE_Msk          (0xFUL << 4)

/* APB1 Prescaler */
#define RCC_CFGR_PPRE1_Msk         (0x7UL << 10)
#define RCC_CFGR_PPRE1_DIV2        (0x4UL << 10)

/* APB2 Prescaler */
#define RCC_CFGR_PPRE2_Msk         (0x7UL << 13)
#define RCC_CFGR_PPRE2_DIV1        (0x0UL << 13)

/* ----------------------------- USART ----------------------------- */
typedef struct {
volatile uint32_t SR;
volatile uint32_t DR;
volatile uint32_t BRR;
volatile uint32_t CR1;
volatile uint32_t CR2;
volatile uint32_t CR3;
volatile uint32_t GTPR;
} USART_TypeDef;

#define USART2              ((USART_TypeDef *)USART2_BASE)

#define USART_SR_TXE        (1UL << 7)
#define USART_SR_TC         (1UL << 6)
#define USART_SR_RXNE       (1UL << 5)

#define USART_CR1_RE        (1UL << 2)
#define USART_CR1_TE        (1UL << 3)
#define USART_CR1_UE        (1UL << 13)

/* ---------------------------- SysTick ---------------------------- */
typedef struct {
volatile uint32_t CTRL;
volatile uint32_t LOAD;
volatile uint32_t VAL;
volatile uint32_t CALIB;
} SysTick_TypeDef;

#define SysTick             ((SysTick_TypeDef *)SYSTICK_BASE)

#define SysTick_CTRL_ENABLE_Msk    (1UL << 0)
#define SysTick_CTRL_TICKINT_Msk   (1UL << 1)
#define SysTick_CTRL_CLKSOURCE_Msk (1UL << 2)

/* Default core clock after reset: HSI, no PLL, no prescalers -> 16 MHz */
extern uint32_t SystemCoreClock;

/* ---------------------------- FLASH ---------------------------- */
typedef struct
{
volatile uint32_t ACR;
volatile uint32_t KEYR;
volatile uint32_t OPTKEYR;
volatile uint32_t SR;
volatile uint32_t CR;
volatile uint32_t OPTCR;
} FLASH_TypeDef;

#define FLASH ((FLASH_TypeDef *)FLASH_BASE)
#define FLASH_ACR_LATENCY_0WS      0x0
#define FLASH_ACR_LATENCY_1WS      0x1
#define FLASH_ACR_LATENCY_2WS      0x2

#define FLASH_ACR_ICEN             (1UL << 9)
#define FLASH_ACR_DCEN             (1UL << 10)
#define FLASH_ACR_PRFTEN           (1UL << 8)

/* ---------------------------- SCB ---------------------------- */
typedef struct
{
volatile uint32_t CPUID;
volatile uint32_t ICSR;
volatile uint32_t VTOR;
volatile uint32_t AIRCR;
volatile uint32_t SCR;
volatile uint32_t CCR;
volatile uint8_t  SHP[12];
volatile uint32_t SHCSR;
volatile uint32_t CFSR;
volatile uint32_t HFSR;
volatile uint32_t DFSR;
volatile uint32_t MMFAR;
volatile uint32_t BFAR;
volatile uint32_t AFSR;
volatile uint32_t PFR[2];
volatile uint32_t DFR;
volatile uint32_t ADR;
volatile uint32_t MMFR[4];
volatile uint32_t ISAR[5];
uint32_t RESERVED0[5];
volatile uint32_t CPACR;
} SCB_TypeDef;

#define SCB ((SCB_TypeDef *)SCB_BASE)

/*------------------------------- I2C ---------------------------*/
#define I2C1_BASE        (0x40005400UL)

typedef struct {
volatile uint32_t CR1;
volatile uint32_t CR2;
volatile uint32_t OAR1;
volatile uint32_t OAR2;
volatile uint32_t DR;
volatile uint32_t SR1;
volatile uint32_t SR2;
volatile uint32_t CCR;
volatile uint32_t TRISE;
volatile uint32_t FLTR;
} I2C_TypeDef;

#define I2C1  ((I2C_TypeDef *)I2C1_BASE)

// CR1
#define I2C_CR1_PE       (1U << 0)
#define I2C_CR1_START    (1U << 8)
#define I2C_CR1_STOP     (1U << 9)
#define I2C_CR1_ACK      (1U << 10)
#define I2C_CR1_POS      (1U << 11)
#define I2C_CR1_SWRST    (1U << 15)

// CR2
#define I2C_CR2_ITERREN  (1U << 8)
#define I2C_CR2_ITEVTEN  (1U << 9)
#define I2C_CR2_ITBUFEN  (1U << 10)

// SR1
#define I2C_SR1_SB       (1U << 0)
#define I2C_SR1_ADDR     (1U << 1)
#define I2C_SR1_BTF      (1U << 2)
#define I2C_SR1_BERR     (1U << 8)
#define I2C_SR1_ARLO     (1U << 9)
#define I2C_SR1_RXNE     (1U << 6)
#define I2C_SR1_TXE      (1U << 7)
#define I2C_SR1_STOPF    (1U << 4)
#define I2C_SR1_AF       (1U << 10)   // ack failure - important for a NACK'd address
#define I2C_SR1_OVR      (1U << 11)
#define I2C_SR1_PECERR   (1U << 12)
#define I2C_SR1_TIMEOUT  (1U << 14)
#define I2C_SR1_SMBALERT (1U << 15)

// SR2
#define I2C_SR2_MSL      (1U << 0)
#define I2C_SR2_BUSY     (1U << 1)
#define I2C_SR2_TRA      (1U << 2)

// CCR
#define I2C_CCR_FS       (1U << 15)   // 0 = Standard mode, 1 = Fast mode
#define I2C_CCR_DUTY     (1U << 14)   // Fast mode duty cycle select (0: Tlow/Thigh=2)

#endif /* STM32F411_H */
