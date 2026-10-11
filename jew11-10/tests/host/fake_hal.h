#ifndef PENDULUM_FAKE_HAL_H
#define PENDULUM_FAKE_HAL_H

/* Host tests only: substitute the HAL boundary, not the production logic. */
#define __MAIN_H
#include <stdint.h>

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
typedef struct {
    uint32_t CR1, PSC, EGR, CNT, SR, ARR, CCR1;
} TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

extern GPIO_TypeDef fake_gpio_b, fake_gpio_c;
extern TIM_TypeDef fake_encoder_timer;
extern uint32_t SystemCoreClock;
#define TIM3 (&fake_encoder_timer)
#define GPIOB (&fake_gpio_b)
#define GPIOC (&fake_gpio_c)
#define LIMIT_A_GPIO_Port GPIOC
#define LIMIT_A_Pin (1u << 8)
#define LIMIT_B_GPIO_Port GPIOC
#define LIMIT_B_Pin (1u << 7)
#define TIM_CHANNEL_1 0u
#define TIM_CR1_CEN 1u
#define TIM_CR1_ARPE (1u << 7)
#define TIM_EGR_UG 1u
#define TIM_SR_UIF 1u
#define TIM_FLAG_UPDATE TIM_SR_UIF
#define TIM_FLAG_CC1 (1u << 1)
#define RESET 0u
#define HAL_OK 0
void fake_nop(void);
uint32_t fake_get_primask(void);
void fake_set_primask(uint32_t mask);
#define __NOP() fake_nop()
#define __get_PRIMASK() fake_get_primask()
#define __disable_irq() fake_set_primask(1u)
#define __set_PRIMASK(mask) fake_set_primask(mask)
#define __HAL_TIM_ENABLE_OCxPRELOAD(handle, channel) ((void)0)
#define __HAL_TIM_DISABLE_OCxPRELOAD(handle, channel) ((void)0)
#define __HAL_TIM_SET_COMPARE(handle, channel, value) \
    ((handle)->Instance->CCR1 = (value))
#define __HAL_TIM_SET_AUTORELOAD(handle, value) \
    ((handle)->Instance->ARR = (value))
#define __HAL_TIM_GET_FLAG(handle, flag) ((handle)->Instance->SR & (flag))
#define __HAL_TIM_CLEAR_FLAG(handle, flag) ((handle)->Instance->SR &= ~(flag))

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_Delay(uint32_t milliseconds);
uint32_t HAL_GetTick(void);
int HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *timer, uint32_t channel);
int HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
