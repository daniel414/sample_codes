################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sdk/platform/devices/tw9001/startup/system_tw9001.c 

OBJS += \
./sdk/platform/devices/tw9001/startup/system_tw9001.o 

C_DEPS += \
./sdk/platform/devices/tw9001/startup/system_tw9001.d 


# Each subdirectory must supply rules for building sources it contributes
sdk/platform/devices/tw9001/startup/%.o: ../sdk/platform/devices/tw9001/startup/%.c sdk/platform/devices/tw9001/startup/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DDEV_ERROR_DETECT -I../sdk/platform/devices/common -I../sdk/platform/devices/tw9001/include -I../sdk/platform/devices/tw9001/startup -I../sdk/platform/devices -I../sdk/platform/drivers/inc -I../sdk/platform/drivers/inc/pufs -I../inc -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


