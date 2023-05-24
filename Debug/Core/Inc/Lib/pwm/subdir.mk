################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Lib/pwm/pwm.c 

OBJS += \
./Core/Inc/Lib/pwm/pwm.o 

C_DEPS += \
./Core/Inc/Lib/pwm/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Lib/pwm/%.o Core/Inc/Lib/pwm/%.su Core/Inc/Lib/pwm/%.cyclo: ../Core/Inc/Lib/pwm/%.c Core/Inc/Lib/pwm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L031xx -c -I../Core/Inc -I../Core/Inc/Lib/motor_cmd_utils -I../Core/Inc/Lib/uart_printf -I../Core/Inc/Lib/pwm -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/Lib/ -I../Core/Inc/Lib/timer_utils -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Lib-2f-pwm

clean-Core-2f-Inc-2f-Lib-2f-pwm:
	-$(RM) ./Core/Inc/Lib/pwm/pwm.cyclo ./Core/Inc/Lib/pwm/pwm.d ./Core/Inc/Lib/pwm/pwm.o ./Core/Inc/Lib/pwm/pwm.su

.PHONY: clean-Core-2f-Inc-2f-Lib-2f-pwm

