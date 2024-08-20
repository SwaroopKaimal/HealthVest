################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MLX90614/MLX90614.c 

OBJS += \
./MLX90614/MLX90614.o 

C_DEPS += \
./MLX90614/MLX90614.d 


# Each subdirectory must supply rules for building sources it contributes
MLX90614/%.o MLX90614/%.su MLX90614/%.cyclo: ../MLX90614/%.c MLX90614/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MPU6050" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/SD_Card_FATFS" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/DS3231_RTC" -I"D:/EmbeddedSystems/Workspace_Polysomnography/VitalSense_Main/MLX90614" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MLX90614

clean-MLX90614:
	-$(RM) ./MLX90614/MLX90614.cyclo ./MLX90614/MLX90614.d ./MLX90614/MLX90614.o ./MLX90614/MLX90614.su

.PHONY: clean-MLX90614

