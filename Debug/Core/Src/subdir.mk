################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lsm6dsl_hal.c \
../Core/Src/lsm6dsl_reg.c \
../Core/Src/main.c \
../Core/Src/motor_ll.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tca9548a.c \
../Core/Src/tcs32725.c \
../Core/Src/vl53l0x.c 

OBJS += \
./Core/Src/lsm6dsl_hal.o \
./Core/Src/lsm6dsl_reg.o \
./Core/Src/main.o \
./Core/Src/motor_ll.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tca9548a.o \
./Core/Src/tcs32725.o \
./Core/Src/vl53l0x.o 

C_DEPS += \
./Core/Src/lsm6dsl_hal.d \
./Core/Src/lsm6dsl_reg.d \
./Core/Src/main.d \
./Core/Src/motor_ll.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tca9548a.d \
./Core/Src/tcs32725.d \
./Core/Src/vl53l0x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/lsm6dsl_hal.cyclo ./Core/Src/lsm6dsl_hal.d ./Core/Src/lsm6dsl_hal.o ./Core/Src/lsm6dsl_hal.su ./Core/Src/lsm6dsl_reg.cyclo ./Core/Src/lsm6dsl_reg.d ./Core/Src/lsm6dsl_reg.o ./Core/Src/lsm6dsl_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_ll.cyclo ./Core/Src/motor_ll.d ./Core/Src/motor_ll.o ./Core/Src/motor_ll.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tca9548a.cyclo ./Core/Src/tca9548a.d ./Core/Src/tca9548a.o ./Core/Src/tca9548a.su ./Core/Src/tcs32725.cyclo ./Core/Src/tcs32725.d ./Core/Src/tcs32725.o ./Core/Src/tcs32725.su ./Core/Src/vl53l0x.cyclo ./Core/Src/vl53l0x.d ./Core/Src/vl53l0x.o ./Core/Src/vl53l0x.su

.PHONY: clean-Core-2f-Src

