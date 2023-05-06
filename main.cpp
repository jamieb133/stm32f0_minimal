#define STM32F051x8 1

#include "stm32f051x8.h"
#include "stm32f0xx.h"
#include "core_cm0.h"
#include "defs.h"

static void initSystick();
static void configureClock();
static void initGPIO();
static void delay(int);
void SystemInit();

int main()
{
    while(true)
    {
        // Toggle green LED.
        int odr = GPIOC->ODR ;
        GPIOC->BSRR = ((odr & GPIO_PIN_9) << 16) | (~odr & GPIO_PIN_9);
        delay(100000);
    } 
    return 0;
}

static void initSystick()
{
    // Enable prefetch on the buffer.
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // Set systick IRQ at highest priority.
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// See page 942 of the reference manual 
// https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=942&zoom=100,165,146
static void configureClock()
{
    // Enable the internal  clock (HSI) and clock security system (CSS).
    RCC->CR |= RCC_CR_CSSON | RCC_CR_HSION; 

    // Wait for HSI ready bit.
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // Enable 14Mhz internal clock (HSI14).
    RCC->CR2 |= RCC_CR2_HSI14ON;

    // Wait for the HSI14 ready bit.
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));

    // Enable power on the clock.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Set the highest APB divider in order to ensure that we do not go through
    // a non-spec phase whatever we decrease or increase HCLK. 
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_CFGR_PPRE_DIV16);

    // Set prescalar (RCC SysClock not divided).
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    // Wait for HSI ready bit.
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // Configure the system clock source as HSI.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
}

static void initGPIO()
{
    volatile uint32_t readBit;
    constexpr uint32_t position { 9 };
    constexpr uint32_t pushPullModeMask { 0x1UL };

    // Enable clock for GPIO port C.
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    readBit = RCC->AHBENR & RCC_AHBENR_GPIOCEN;
    (void)readBit;

    /*Configure GPIO pin Output Level */
    GPIOC->BRR = (uint32_t)GPIO_PIN_9;

    // Configure direction mode.
    uint32_t temp = GPIOC->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2u));
    temp |= (pushPullModeMask << (position * 2u));
    GPIOC->MODER = temp;
}

static void delay(int d)
{
    while(d--);
}

void SystemInit()
{
    initSystick();
    configureClock();
    initGPIO();
}
