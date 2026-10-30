/* Host-only HAL substitute. Never add this directory to CubeIDE include paths. */
#ifndef OFFLINE_AUDIT_HAL_H
#define OFFLINE_AUDIT_HAL_H

#include <stdint.h>

typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t unused; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

extern TIM_TypeDef host_tim1;
#define TIM1 (&host_tim1)

uint32_t HAL_GetTick(void);

#endif
