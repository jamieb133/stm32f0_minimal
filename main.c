#define STM32F051x8 1

#include "stm32f051x8.h"
#include "stm32f0xx.h"
#include "core_cm0.h"
#include "defs.h"

// Clock 
static void systick_init(void);
static void configure_clock(void);

// GPIO
static void gpio_init(void);
static void toggle_led(void);

// Timer
static void timer_init(void);
static void delay(uint32_t);

// ADC
static void adc_init(void);
static void adc_enable(void);
static uint16_t adc_read(void);

#define SET_REG_BIT(REG, POS) REG |= (1 << POS)
#define CLEAR_REG_BIT(REG, POS) REG &= 0xffffffff & ~(1 << POS)
#define WRITE_REG_MASK(REG, MASK, VAL, POS) REG = (REG & ~(MASK << POS)) | (VAL << POS)

static void toggle_example()
{
    toggle_led();
    int delay = 1000000;
    while(--delay);
}

static void timer_example()
{
    toggle_led();
    delay(250);
}

static void adc_example()
{
    uint16_t adc_val = adc_read();
    toggle_led();
    int wait = (int)(((double)adc_val / 4096.0) * 500.0);
    delay(wait);
}

int main()
{
    gpio_init();
    timer_init();
    adc_init();
    adc_enable();

    //enable_timer_isr();

    while(1)
    {
        adc_example();
    }
    
    return 0;
}

void systick_init(void)
{
    // Enable prefetch on the buffer.
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    // Set systick IRQ at highest priority.
    NVIC_SetPriority(SysTick_IRQn, 0);
}

// See page 942 of the reference manual 
// https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=942&zoom=100,165,146
void configure_clock(void)
{
    // Enable the internal  clock (HSI) and clock security system (CSS).
    RCC->CR |= RCC_CR_CSSON | RCC_CR_HSION;

    // Wait for HSI ready bit.
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // Enable power on the clock.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Set the highest APB divider in order to ensure that we do not go through
    // a non-spec phase whatever we decrease or increase HCLK. 
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_CFGR_PPRE_DIV1);

    // Set prescalar (RCC SysClock not divided).
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV2);

    // Wait for HSI ready bit.
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // Configure the system clock source as HSI before we change it to the PLL.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait for the system clock to switch to HSI.
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); 

    // Disable the PLL.
    RCC->CR &=  (uint32_t)(~RCC_CR_PLLON);
    while((RCC->CR & RCC_CR_PLLRDY) != 0);

    // Set PLL mutliplier to 12.
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PLLMUL)) | (RCC_CFGR_PLLMUL12);

    // Enable the PLL.
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0); 

    // Set the system clock as the PLL.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 

    // Enable the clock output (MCO).
    RCC->CFGR |= RCC_CFGR_MCO_HSI;
    while ((RCC->CFGR & RCC_CFGR_MCO) != RCC_CFGR_MCO_HSI); 

}

void delay(uint32_t d)
{
    TIM6->CNT = 0x0000U;
    while(TIM6->CNT < d);
}

void timer_init(void)
{
    // Enable timer clock.
    // From page 125 of ref manual:
        //  Bit 4 TIM6EN: TIM6 timer clock enable
        //  Set and cleared by software.
        //  0: TIM6 clock disabled
        //  1: TIM6 clock enabled
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Set prescalar.
    TIM6->PSC = 48 - 1; // 48Mhz/48 = 1Mhz ~~ 1us delay
    TIM6->PSC = (48 * 1000000) - 1; // 48Mhz/48 = 1Mhz ~~ 1us delay
    TIM6->ARR = 0xffff;

    // Enable the interrupt.
    TIM6->DIER = 0x0001U;

    // Enable the timer.
    TIM6->CR1 |= 0x00001U;

    // Wait for update flag.
    while(!(TIM6->SR & 0x1U));
}

void enable_timer_isr(void)
{
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void gpio_init(void)
{
    volatile uint32_t readBit;
    const uint32_t position = 9;
    const uint32_t pushPullModeMask = 0x1UL;

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

void toggle_led(void)
{
    // Toggle green LED.
    int odr = GPIOC->ODR ;
    GPIOC->BSRR = ((odr & GPIO_PIN_9) << 16) | (~odr & GPIO_PIN_9);
}

void adc_init(void)
{
    // Enable the ADC clock.
    const int adc_clock_position = 9;
    RCC->APB2ENR |= 1 << adc_clock_position;

    // Enable the GPIO port A register.
    const int pa_clock_position = 17;
    RCC->AHBENR |= 1 << pa_clock_position;

    // Setup pin PA0 in analogue mode. 
    const int pin_number = 0;
    const uint8_t analogue_mode_mask = 3U;
    GPIOA->MODER |= analogue_mode_mask << pin_number;

    // PA0 is ADC channel 0, enable it. 
    const int channel_number = 0;
    ADC1->CHSELR = 1 << channel_number;

    // Configure the ADC in 12-bit mode (RES bits of config reg are 00)
    const int res_position = 0;
    ADC1->CFGR1 &= 0xffffffff & ~(3 << res_position);

    // Use the ADC asyncronous clock (val = 0).
    CLEAR_REG_BIT(ADC1->CFGR2, 30);

    // Set the scanning direction to forward.
    CLEAR_REG_BIT(ADC1->CFGR1, 2);

    // Use continuous conversions.
    CLEAR_REG_BIT(ADC1->CFGR2, 13);

    // Set alignment to right alignment.
    CLEAR_REG_BIT(ADC1->CFGR2, 5);

    // Set the sampling time as 1us, the formula is (1.5 + selected_sampling_t) * 14MHz.
    // i.e. (12.5 + 1.5) * (1 / 14000000) seconds = 1e-6 seconds.
    // Therefore select 1.5 ADC clock cycles as closest availble.
    ADC1->SMPR &= ~7;
}

void adc_enable(void)
{
    // Enable the ADC.
    SET_REG_BIT(ADC1->CR, 0);

    // Wait until the ADC is ready.
    const int status_reg_pos = 0;
    while(!(ADC1->ISR & (1 << status_reg_pos)));
}

uint16_t adc_read(void)
{
    // Start the ADC.
    SET_REG_BIT(ADC1->CR, 2);

    // Wait for the conversion to finish.
    const int eoc_flag_pos = 2;
    while(!(ADC1->ISR & (1 << eoc_flag_pos)));

    // Copy from the ADC data register.
    return ADC1->DR;
}

void SystemInit()
{
    systick_init();
    configure_clock();
}

/*******************************/
/*
Interrupt services routines.
*/

void TIM6_DAC_IRQHandler()
{
    toggle_led();
}
