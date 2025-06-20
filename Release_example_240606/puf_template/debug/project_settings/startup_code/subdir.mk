################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../project_settings/startup_code/startup_tw9001.S 

OBJS += \
./project_settings/startup_code/startup_tw9001.o 

S_UPPER_DEPS += \
./project_settings/startup_code/startup_tw9001.d 


# Each subdirectory must supply rules for building sources it contributes
project_settings/startup_code/%.o: ../project_settings/startup_code/%.S project_settings/startup_code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


