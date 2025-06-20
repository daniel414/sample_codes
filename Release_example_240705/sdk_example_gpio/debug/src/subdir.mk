################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/clock_config.c \
../src/main.c 

OBJS += \
./src/clock_config.o \
./src/main.o 

C_DEPS += \
./src/clock_config.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DDEV_ERROR_DETECT -I../sdk/platform/devices/common -I../sdk/platform/devices/tw9001/include -I../sdk/platform/devices/tw9001/startup -I../sdk/platform/devices -I../sdk/platform/drivers/inc -I../sdk/platform/drivers/inc/pufs -I../inc -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


