################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/at42qt1070.c \
../Core/Src/i2c_scan.c \
../Core/Src/ina219_drv.c \
../Core/Src/led_drv.c \
../Core/Src/main.c \
../Core/Src/main_loop.c \
../Core/Src/mcp23017.c \
../Core/Src/mcp4728_dac.c \
../Core/Src/mcp9808.c \
../Core/Src/oled_drv.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/ugui.c 

OBJS += \
./Core/Src/at42qt1070.o \
./Core/Src/i2c_scan.o \
./Core/Src/ina219_drv.o \
./Core/Src/led_drv.o \
./Core/Src/main.o \
./Core/Src/main_loop.o \
./Core/Src/mcp23017.o \
./Core/Src/mcp4728_dac.o \
./Core/Src/mcp9808.o \
./Core/Src/oled_drv.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/ugui.o 

C_DEPS += \
./Core/Src/at42qt1070.d \
./Core/Src/i2c_scan.d \
./Core/Src/ina219_drv.d \
./Core/Src/led_drv.d \
./Core/Src/main.d \
./Core/Src/main_loop.d \
./Core/Src/mcp23017.d \
./Core/Src/mcp4728_dac.d \
./Core/Src/mcp9808.d \
./Core/Src/oled_drv.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/ugui.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/at42qt1070.cyclo ./Core/Src/at42qt1070.d ./Core/Src/at42qt1070.o ./Core/Src/at42qt1070.su ./Core/Src/i2c_scan.cyclo ./Core/Src/i2c_scan.d ./Core/Src/i2c_scan.o ./Core/Src/i2c_scan.su ./Core/Src/ina219_drv.cyclo ./Core/Src/ina219_drv.d ./Core/Src/ina219_drv.o ./Core/Src/ina219_drv.su ./Core/Src/led_drv.cyclo ./Core/Src/led_drv.d ./Core/Src/led_drv.o ./Core/Src/led_drv.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/main_loop.cyclo ./Core/Src/main_loop.d ./Core/Src/main_loop.o ./Core/Src/main_loop.su ./Core/Src/mcp23017.cyclo ./Core/Src/mcp23017.d ./Core/Src/mcp23017.o ./Core/Src/mcp23017.su ./Core/Src/mcp4728_dac.cyclo ./Core/Src/mcp4728_dac.d ./Core/Src/mcp4728_dac.o ./Core/Src/mcp4728_dac.su ./Core/Src/mcp9808.cyclo ./Core/Src/mcp9808.d ./Core/Src/mcp9808.o ./Core/Src/mcp9808.su ./Core/Src/oled_drv.cyclo ./Core/Src/oled_drv.d ./Core/Src/oled_drv.o ./Core/Src/oled_drv.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/ugui.cyclo ./Core/Src/ugui.d ./Core/Src/ugui.o ./Core/Src/ugui.su

.PHONY: clean-Core-2f-Src

