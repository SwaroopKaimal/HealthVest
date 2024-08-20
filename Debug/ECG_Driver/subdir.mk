################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECG_Driver/ECG.c 

OBJS += \
./ECG_Driver/ECG.o 

C_DEPS += \
./ECG_Driver/ECG.d 


# Each subdirectory must supply rules for building sources it contributes
ECG_Driver/%.o ECG_Driver/%.su ECG_Driver/%.cyclo: ../ECG_Driver/%.c ECG_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MPU6050" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/SD_Card_FATFS" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/DS3231_RTC" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MLX90614" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECG_Driver

clean-ECG_Driver:
	-$(RM) ./ECG_Driver/ECG.cyclo ./ECG_Driver/ECG.d ./ECG_Driver/ECG.o ./ECG_Driver/ECG.su

.PHONY: clean-ECG_Driver

