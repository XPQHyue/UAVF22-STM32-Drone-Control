################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
../Core/Src/MPU6050.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/inv_mpu.c \
../Core/Src/inv_mpu_dmp_motion_driver.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/pid_incomplete_derivative.c \
../Core/Src/pid_incremental.c \
../Core/Src/pid_integral_separation.c \
../Core/Src/pid_study.c \
../Core/Src/pid_variable_integral.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/MPU6050.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/inv_mpu.o \
./Core/Src/inv_mpu_dmp_motion_driver.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/pid_incomplete_derivative.o \
./Core/Src/pid_incremental.o \
./Core/Src/pid_integral_separation.o \
./Core/Src/pid_study.o \
./Core/Src/pid_variable_integral.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/MPU6050.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/inv_mpu.d \
./Core/Src/inv_mpu_dmp_motion_driver.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/pid_incomplete_derivative.d \
./Core/Src/pid_incremental.d \
./Core/Src/pid_integral_separation.d \
./Core/Src/pid_study.d \
./Core/Src/pid_variable_integral.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# 每个子目录必须为构建它所贡献的源提供规则
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MPU6050.cyclo ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/inv_mpu.cyclo ./Core/Src/inv_mpu.d ./Core/Src/inv_mpu.o ./Core/Src/inv_mpu.su ./Core/Src/inv_mpu_dmp_motion_driver.cyclo ./Core/Src/inv_mpu_dmp_motion_driver.d ./Core/Src/inv_mpu_dmp_motion_driver.o ./Core/Src/inv_mpu_dmp_motion_driver.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid_incomplete_derivative.cyclo ./Core/Src/pid_incomplete_derivative.d ./Core/Src/pid_incomplete_derivative.o ./Core/Src/pid_incomplete_derivative.su ./Core/Src/pid_incremental.cyclo ./Core/Src/pid_incremental.d ./Core/Src/pid_incremental.o ./Core/Src/pid_incremental.su ./Core/Src/pid_integral_separation.cyclo ./Core/Src/pid_integral_separation.d ./Core/Src/pid_integral_separation.o ./Core/Src/pid_integral_separation.su ./Core/Src/pid_study.cyclo ./Core/Src/pid_study.d ./Core/Src/pid_study.o ./Core/Src/pid_study.su ./Core/Src/pid_variable_integral.cyclo ./Core/Src/pid_variable_integral.d ./Core/Src/pid_variable_integral.o ./Core/Src/pid_variable_integral.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

