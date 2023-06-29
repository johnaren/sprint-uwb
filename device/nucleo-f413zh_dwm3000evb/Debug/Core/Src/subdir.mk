################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/config_options.c \
../Core/Src/deca_mutex.c \
../Core/Src/deca_probe_interface.c \
../Core/Src/deca_sleep.c \
../Core/Src/deca_spi.c \
../Core/Src/ds_twr_initiator.c \
../Core/Src/ds_twr_responder.c \
../Core/Src/main.c \
../Core/Src/port.c \
../Core/Src/shared_functions.c \
../Core/Src/ss_twr_initiator.c \
../Core/Src/ss_twr_responder.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/user.c 

OBJS += \
./Core/Src/config_options.o \
./Core/Src/deca_mutex.o \
./Core/Src/deca_probe_interface.o \
./Core/Src/deca_sleep.o \
./Core/Src/deca_spi.o \
./Core/Src/ds_twr_initiator.o \
./Core/Src/ds_twr_responder.o \
./Core/Src/main.o \
./Core/Src/port.o \
./Core/Src/shared_functions.o \
./Core/Src/ss_twr_initiator.o \
./Core/Src/ss_twr_responder.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/user.o 

C_DEPS += \
./Core/Src/config_options.d \
./Core/Src/deca_mutex.d \
./Core/Src/deca_probe_interface.d \
./Core/Src/deca_sleep.d \
./Core/Src/deca_spi.d \
./Core/Src/ds_twr_initiator.d \
./Core/Src/ds_twr_responder.d \
./Core/Src/main.d \
./Core/Src/port.d \
./Core/Src/shared_functions.d \
./Core/Src/ss_twr_initiator.d \
./Core/Src/ss_twr_responder.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/user.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=c11 -g3 -DANCHOR '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DDEBUG -DUSE_HAL_DRIVER -DSTM32F413xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/DWT_UWB_Driver/Inc -includesys/time.h -includestdio.h -includemain.h -O0 -ffunction-sections -Wall -Wextra -pedantic -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/config_options.cyclo ./Core/Src/config_options.d ./Core/Src/config_options.o ./Core/Src/config_options.su ./Core/Src/deca_mutex.cyclo ./Core/Src/deca_mutex.d ./Core/Src/deca_mutex.o ./Core/Src/deca_mutex.su ./Core/Src/deca_probe_interface.cyclo ./Core/Src/deca_probe_interface.d ./Core/Src/deca_probe_interface.o ./Core/Src/deca_probe_interface.su ./Core/Src/deca_sleep.cyclo ./Core/Src/deca_sleep.d ./Core/Src/deca_sleep.o ./Core/Src/deca_sleep.su ./Core/Src/deca_spi.cyclo ./Core/Src/deca_spi.d ./Core/Src/deca_spi.o ./Core/Src/deca_spi.su ./Core/Src/ds_twr_initiator.cyclo ./Core/Src/ds_twr_initiator.d ./Core/Src/ds_twr_initiator.o ./Core/Src/ds_twr_initiator.su ./Core/Src/ds_twr_responder.cyclo ./Core/Src/ds_twr_responder.d ./Core/Src/ds_twr_responder.o ./Core/Src/ds_twr_responder.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/port.cyclo ./Core/Src/port.d ./Core/Src/port.o ./Core/Src/port.su ./Core/Src/shared_functions.cyclo ./Core/Src/shared_functions.d ./Core/Src/shared_functions.o ./Core/Src/shared_functions.su ./Core/Src/ss_twr_initiator.cyclo ./Core/Src/ss_twr_initiator.d ./Core/Src/ss_twr_initiator.o ./Core/Src/ss_twr_initiator.su ./Core/Src/ss_twr_responder.cyclo ./Core/Src/ss_twr_responder.d ./Core/Src/ss_twr_responder.o ./Core/Src/ss_twr_responder.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/user.cyclo ./Core/Src/user.d ./Core/Src/user.o ./Core/Src/user.su

.PHONY: clean-Core-2f-Src

