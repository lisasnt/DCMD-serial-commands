################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Lib/uart_printf.c 

OBJS += \
./Core/Inc/Lib/uart_printf.o 

C_DEPS += \
./Core/Inc/Lib/uart_printf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Lib/%.o Core/Inc/Lib/%.su Core/Inc/Lib/%.cyclo: ../Core/Inc/Lib/%.c Core/Inc/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L031xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Lib

clean-Core-2f-Inc-2f-Lib:
	-$(RM) ./Core/Inc/Lib/uart_printf.cyclo ./Core/Inc/Lib/uart_printf.d ./Core/Inc/Lib/uart_printf.o ./Core/Inc/Lib/uart_printf.su

.PHONY: clean-Core-2f-Inc-2f-Lib

