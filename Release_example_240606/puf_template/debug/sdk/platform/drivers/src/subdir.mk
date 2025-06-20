################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sdk/platform/drivers/src/clock_tw9001.c \
../sdk/platform/drivers/src/edma_driver.c \
../sdk/platform/drivers/src/interrupt_manager.c \
../sdk/platform/drivers/src/lptmr_driver.c \
../sdk/platform/drivers/src/lpuart_driver.c \
../sdk/platform/drivers/src/osif_driver.c \
../sdk/platform/drivers/src/pins_driver.c \
../sdk/platform/drivers/src/power_manager.c \
../sdk/platform/drivers/src/smc_driver.c 

OBJS += \
./sdk/platform/drivers/src/clock_tw9001.o \
./sdk/platform/drivers/src/edma_driver.o \
./sdk/platform/drivers/src/interrupt_manager.o \
./sdk/platform/drivers/src/lptmr_driver.o \
./sdk/platform/drivers/src/lpuart_driver.o \
./sdk/platform/drivers/src/osif_driver.o \
./sdk/platform/drivers/src/pins_driver.o \
./sdk/platform/drivers/src/power_manager.o \
./sdk/platform/drivers/src/smc_driver.o 

C_DEPS += \
./sdk/platform/drivers/src/clock_tw9001.d \
./sdk/platform/drivers/src/edma_driver.d \
./sdk/platform/drivers/src/interrupt_manager.d \
./sdk/platform/drivers/src/lptmr_driver.d \
./sdk/platform/drivers/src/lpuart_driver.d \
./sdk/platform/drivers/src/osif_driver.d \
./sdk/platform/drivers/src/pins_driver.d \
./sdk/platform/drivers/src/power_manager.d \
./sdk/platform/drivers/src/smc_driver.d 


# Each subdirectory must supply rules for building sources it contributes
sdk/platform/drivers/src/%.o: ../sdk/platform/drivers/src/%.c sdk/platform/drivers/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -DDEV_ERROR_DETECT -I../sdk/platform/devices/common -I../sdk/platform/devices/tw9001/include -I../sdk/platform/devices/tw9001/startup -I../sdk/platform/devices -I../sdk/platform/drivers/inc -I../sdk/platform/drivers/inc/pufs -I../inc -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


