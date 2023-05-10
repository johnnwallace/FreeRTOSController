################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BNO055/bno055.c 

OBJS += \
./Drivers/BNO055/bno055.o 

C_DEPS += \
./Drivers/BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BNO055/%.o Drivers/BNO055/%.su Drivers/BNO055/%.cyclo: ../Drivers/BNO055/%.c Drivers/BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/Users/jowallace/STM32CubeIDE/workspace_1.12.0/nucleo-f401re-controller/Drivers/BNO055" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BNO055

clean-Drivers-2f-BNO055:
	-$(RM) ./Drivers/BNO055/bno055.cyclo ./Drivers/BNO055/bno055.d ./Drivers/BNO055/bno055.o ./Drivers/BNO055/bno055.su

.PHONY: clean-Drivers-2f-BNO055

