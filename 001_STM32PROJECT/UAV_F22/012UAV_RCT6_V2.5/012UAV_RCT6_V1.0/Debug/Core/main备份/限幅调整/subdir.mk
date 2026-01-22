################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
../Core/main备份/限幅调整/main.c 

OBJS += \
./Core/main备份/限幅调整/main.o 

C_DEPS += \
./Core/main备份/限幅调整/main.d 


# 每个子目录必须为构建它所贡献的源提供规则
Core/main备份/限幅调整/%.o Core/main备份/限幅调整/%.su Core/main备份/限幅调整/%.cyclo: ../Core/main备份/限幅调整/%.c Core/main备份/限幅调整/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-main-5907--4efd--2f--9650--5e45--8c03--6574-

clean-Core-2f-main-5907--4efd--2f--9650--5e45--8c03--6574-:
	-$(RM) ./Core/main备份/限幅调整/main.cyclo ./Core/main备份/限幅调整/main.d ./Core/main备份/限幅调整/main.o ./Core/main备份/限幅调整/main.su

.PHONY: clean-Core-2f-main-5907--4efd--2f--9650--5e45--8c03--6574-

