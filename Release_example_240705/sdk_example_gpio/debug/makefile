################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include sdk/platform/drivers/src/subdir.mk
-include sdk/platform/devices/tw9001/startup/subdir.mk
-include sdk/platform/devices/subdir.mk
-include project_settings/startup_code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(SX_DEPS)),)
-include $(SX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := sdk-example-gpio
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
sdk-example-gpio.hex \

SECONDARY_LIST += \
sdk-example-gpio.lst \

SECONDARY_SIZE += \
sdk-example-gpio.siz \


# All Target
all: main-build

# Main-build Target
main-build: sdk-example-gpio.elf secondary-outputs

# Tool invocations
sdk-example-gpio.elf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Arm Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -T ../project_settings/linker_files/tw9001_flash.ld -Xlinker --gc-sections -L../sdk/platform/drivers/lib -Wl,-Map,"sdk-example-gpio.map" --specs=nano.specs --specs=nosys.specs -Wl,--start-group -lgcc -lc -lm -Wl,--end-group -o "sdk-example-gpio.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

sdk-example-gpio.hex: sdk-example-gpio.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GNU Arm Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "sdk-example-gpio.elf"  "sdk-example-gpio.hex"
	@echo 'Finished building: $@'
	@echo ' '

sdk-example-gpio.lst: sdk-example-gpio.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GNU Arm Cross Create Listing'
	arm-none-eabi-objdump --source --all-headers --demangle --line-numbers --wide "sdk-example-gpio.elf" > "sdk-example-gpio.lst"
	@echo 'Finished building: $@'
	@echo ' '

sdk-example-gpio.siz: sdk-example-gpio.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GNU Arm Cross Print Size'
	arm-none-eabi-size --format=berkeley "sdk-example-gpio.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SX_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) sdk-example-gpio.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build

-include ../makefile.targets
