################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DS1307_RTC/DS1307_RTC.c 

OBJS += \
./DS1307_RTC/DS1307_RTC.o 

C_DEPS += \
./DS1307_RTC/DS1307_RTC.d 


# Each subdirectory must supply rules for building sources it contributes
DS1307_RTC/%.o DS1307_RTC/%.su DS1307_RTC/%.cyclo: ../DS1307_RTC/%.c DS1307_RTC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/DS1307_RTC" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MPU6050" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/SD_Card_FATFS" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DS1307_RTC

clean-DS1307_RTC:
	-$(RM) ./DS1307_RTC/DS1307_RTC.cyclo ./DS1307_RTC/DS1307_RTC.d ./DS1307_RTC/DS1307_RTC.o ./DS1307_RTC/DS1307_RTC.su

.PHONY: clean-DS1307_RTC

