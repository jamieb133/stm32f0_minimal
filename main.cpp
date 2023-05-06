#define STM32F051x8 1

#include "stm32f051x8.h"
#include "stm32f0xx.h"
#include "core_cm0.h"
#include "defs.h"

static void initSystick();
static void configureClock();
static void initGPIO();
static void delay(uint32_t);
void SystemInit();

int main()
{
    while(true)
    {
        // Toggle green LED.
        int odr = GPIOC->ODR ;
        GPIOC->BSRR = ((odr & GPIO_PIN_9) << 16) | (~odr & GPIO_PIN_9);
        delay(1000);
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

    // Configure GPIO pin Output Level.
    GPIOC->BRR = (uint32_t)GPIO_PIN_9;

    // Configure direction mode.
    uint32_t temp = GPIOC->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2u));
    temp |= (pushPullModeMask << (position * 2u));
    GPIOC->MODER = temp;
}

static void delay(uint32_t d)
{
    for(int i = 0; i < 100; i++)
    {
        TIM6->CNT = 0x0000U;
        while(TIM6->CNT < d);
    }
}

static void initTimer()
{
    // Enable timer clock.
    // From page 125 of ref manual:
        //  Bit 4 TIM6EN: TIM6 timer clock enable
        //  Set and cleared by software.
        //  0: TIM6 clock disabled
        //  1: TIM6 clock enabled
    RCC->APB1ENR |= 1 << 4;

    // Set prescalar.
    TIM6->PSC = 6;
    TIM6->ARR = 0xffff;

    // Enable the timer.
    TIM6->CR1 |= 0x00001U;

    // Wait for update flag.
    while(!(TIM6->SR & 0x1U));
}

void SystemInit()
{
    initSystick();
    configureClock();
    initGPIO();
    initTimer();
}
