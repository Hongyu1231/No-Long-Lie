################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.c \
../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.c 

OBJS += \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.o \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o 

C_DEPS += \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.d \
./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L4S5I-IOT01/%.o Drivers/BSP/B-L4S5I-IOT01/%.su Drivers/BSP/B-L4S5I-IOT01/%.cyclo: ../Drivers/BSP/B-L4S5I-IOT01/%.c Drivers/BSP/B-L4S5I-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01

clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01:
	-$(RM) ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.su ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.cyclo ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o ./Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01

