################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sdk/platform/devices/startup.c 

OBJS += \
./sdk/platform/devices/startup.o 

C_DEPS += \
./sdk/platform/devices/startup.d 


# Each subdirectory must supply rules for building sources it contributes
sdk/platform/devices/%.o: ../sdk/platform/devices/%.c sdk/platform/devices/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DDEV_ERROR_DETECT -I../sdk/platform/devices/common -I../sdk/platform/devices/tw9001/include -I../sdk/platform/devices/tw9001/startup -I../sdk/platform/devices -I../sdk/platform/drivers/inc -I../sdk/platform/drivers/inc/pufs -I../inc -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


