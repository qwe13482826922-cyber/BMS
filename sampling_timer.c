#include "sampling_timer.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#define SAMPLING_PERIOD_MS 10U

static __IO uint8_t sampling_elapsed_flag = 0U;

void SamplingTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_init;
    NVIC_InitTypeDef nvic_init;
    uint16_t prescaler;
    uint16_t period;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    prescaler = (uint16_t)((SystemCoreClock / 1000000U) - 1U);
    period = (uint16_t)((1000U * SAMPLING_PERIOD_MS) - 1U);

    TIM_TimeBaseStructInit(&tim_init);
    tim_init.TIM_Period = period;
    tim_init.TIM_Prescaler = prescaler;
    tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_init);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic_init.NVIC_IRQChannel = TIM2_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init.NVIC_IRQChannelSubPriority = 1;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    TIM_Cmd(TIM2, ENABLE);
}

uint8_t SamplingTimer_IsElapsed(void)
{
    if (sampling_elapsed_flag)
    {
        sampling_elapsed_flag = 0U;
        return 1U;
    }
    return 0U;
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        sampling_elapsed_flag = 1U;
    }
}
