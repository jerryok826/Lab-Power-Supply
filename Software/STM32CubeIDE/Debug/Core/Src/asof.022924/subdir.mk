################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/asof.022924/gcvt_2.c \
../Core/Src/asof.022924/led_drv.c \
../Core/Src/asof.022924/lps_drv.c \
../Core/Src/asof.022924/main.c \
../Core/Src/asof.022924/mcp9808.c \
../Core/Src/asof.022924/stm32f1xx_hal_msp.c \
../Core/Src/asof.022924/stm32f1xx_it.c \
../Core/Src/asof.022924/syscalls.c \
../Core/Src/asof.022924/sysmem.c \
../Core/Src/asof.022924/system_stm32f1xx.c 

OBJS += \
./Core/Src/asof.022924/gcvt_2.o \
./Core/Src/asof.022924/led_drv.o \
./Core/Src/asof.022924/lps_drv.o \
./Core/Src/asof.022924/main.o \
./Core/Src/asof.022924/mcp9808.o \
./Core/Src/asof.022924/stm32f1xx_hal_msp.o \
./Core/Src/asof.022924/stm32f1xx_it.o \
./Core/Src/asof.022924/syscalls.o \
./Core/Src/asof.022924/sysmem.o \
./Core/Src/asof.022924/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/asof.022924/gcvt_2.d \
./Core/Src/asof.022924/led_drv.d \
./Core/Src/asof.022924/lps_drv.d \
./Core/Src/asof.022924/main.d \
./Core/Src/asof.022924/mcp9808.d \
./Core/Src/asof.022924/stm32f1xx_hal_msp.d \
./Core/Src/asof.022924/stm32f1xx_it.d \
./Core/Src/asof.022924/syscalls.d \
./Core/Src/asof.022924/sysmem.d \
./Core/Src/asof.022924/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/asof.022924/%.o Core/Src/asof.022924/%.su Core/Src/asof.022924/%.cyclo: ../Core/Src/asof.022924/%.c Core/Src/asof.022924/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-asof-2e-022924

clean-Core-2f-Src-2f-asof-2e-022924:
	-$(RM) ./Core/Src/asof.022924/gcvt_2.cyclo ./Core/Src/asof.022924/gcvt_2.d ./Core/Src/asof.022924/gcvt_2.o ./Core/Src/asof.022924/gcvt_2.su ./Core/Src/asof.022924/led_drv.cyclo ./Core/Src/asof.022924/led_drv.d ./Core/Src/asof.022924/led_drv.o ./Core/Src/asof.022924/led_drv.su ./Core/Src/asof.022924/lps_drv.cyclo ./Core/Src/asof.022924/lps_drv.d ./Core/Src/asof.022924/lps_drv.o ./Core/Src/asof.022924/lps_drv.su ./Core/Src/asof.022924/main.cyclo ./Core/Src/asof.022924/main.d ./Core/Src/asof.022924/main.o ./Core/Src/asof.022924/main.su ./Core/Src/asof.022924/mcp9808.cyclo ./Core/Src/asof.022924/mcp9808.d ./Core/Src/asof.022924/mcp9808.o ./Core/Src/asof.022924/mcp9808.su ./Core/Src/asof.022924/stm32f1xx_hal_msp.cyclo ./Core/Src/asof.022924/stm32f1xx_hal_msp.d ./Core/Src/asof.022924/stm32f1xx_hal_msp.o ./Core/Src/asof.022924/stm32f1xx_hal_msp.su ./Core/Src/asof.022924/stm32f1xx_it.cyclo ./Core/Src/asof.022924/stm32f1xx_it.d ./Core/Src/asof.022924/stm32f1xx_it.o ./Core/Src/asof.022924/stm32f1xx_it.su ./Core/Src/asof.022924/syscalls.cyclo ./Core/Src/asof.022924/syscalls.d ./Core/Src/asof.022924/syscalls.o ./Core/Src/asof.022924/syscalls.su ./Core/Src/asof.022924/sysmem.cyclo ./Core/Src/asof.022924/sysmem.d ./Core/Src/asof.022924/sysmem.o ./Core/Src/asof.022924/sysmem.su ./Core/Src/asof.022924/system_stm32f1xx.cyclo ./Core/Src/asof.022924/system_stm32f1xx.d ./Core/Src/asof.022924/system_stm32f1xx.o ./Core/Src/asof.022924/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src-2f-asof-2e-022924

