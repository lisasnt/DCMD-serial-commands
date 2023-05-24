################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Lib/timer_utils/timer_utils.c 

OBJS += \
./Core/Inc/Lib/timer_utils/timer_utils.o 

C_DEPS += \
./Core/Inc/Lib/timer_utils/timer_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Lib/timer_utils/%.o Core/Inc/Lib/timer_utils/%.su Core/Inc/Lib/timer_utils/%.cyclo: ../Core/Inc/Lib/timer_utils/%.c Core/Inc/Lib/timer_utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L031xx -c -I../Core/Inc -I../Core/Inc/Lib/motor_cmd_utils -I../Core/Inc/Lib/uart_printf -I../Core/Inc/Lib/pwm -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/Lib/ -I../Core/Inc/Lib/timer_utils -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Lib-2f-timer_utils

clean-Core-2f-Inc-2f-Lib-2f-timer_utils:
	-$(RM) ./Core/Inc/Lib/timer_utils/timer_utils.cyclo ./Core/Inc/Lib/timer_utils/timer_utils.d ./Core/Inc/Lib/timer_utils/timer_utils.o ./Core/Inc/Lib/timer_utils/timer_utils.su

.PHONY: clean-Core-2f-Inc-2f-Lib-2f-timer_utils

