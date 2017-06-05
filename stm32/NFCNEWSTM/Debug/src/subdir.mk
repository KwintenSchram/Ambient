################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/PN532.c \
../src/main.c \
../src/stm32l1xx_hal_msp.c \
../src/stm32l1xx_it.c 

OBJS += \
./src/PN532.o \
./src/main.o \
./src/stm32l1xx_hal_msp.o \
./src/stm32l1xx_it.o 

C_DEPS += \
./src/PN532.d \
./src/main.d \
./src/stm32l1xx_hal_msp.d \
./src/stm32l1xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DSTM32L162xD -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32l1xx" -I"../system/include/cmsis/device" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


