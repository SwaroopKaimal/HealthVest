################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SD_Card_FATFS/fatfs_sd.c 

OBJS += \
./SD_Card_FATFS/fatfs_sd.o 

C_DEPS += \
./SD_Card_FATFS/fatfs_sd.d 


# Each subdirectory must supply rules for building sources it contributes
SD_Card_FATFS/%.o SD_Card_FATFS/%.su SD_Card_FATFS/%.cyclo: ../SD_Card_FATFS/%.c SD_Card_FATFS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MPU6050" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/SD_Card_FATFS" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/DS3231_RTC" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MLX90614" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SD_Card_FATFS

clean-SD_Card_FATFS:
	-$(RM) ./SD_Card_FATFS/fatfs_sd.cyclo ./SD_Card_FATFS/fatfs_sd.d ./SD_Card_FATFS/fatfs_sd.o ./SD_Card_FATFS/fatfs_sd.su

.PHONY: clean-SD_Card_FATFS

