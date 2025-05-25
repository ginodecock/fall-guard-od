################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/subdir.mk
-include STM32Cube_FW_N6/Utilities/lcd/subdir.mk
-include STM32Cube_FW_N6/Middlewares/ST/threadx/utility/execution_profile_kit/subdir.mk
-include STM32Cube_FW_N6/Middlewares/ST/threadx/ports/cortex_m55/gnu/src/subdir.mk
-include STM32Cube_FW_N6/Middlewares/ST/threadx/common/src/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/TLS/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Network/Ethernet\ Phy\ Interface/RTL8211_Phy_Interface/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/NX\ Core/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Ethernet\ Interface/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Crypto/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Addons\ SNTP/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Addons\ MQTT/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Addons\ DNS/subdir.mk
-include STM32Cube_FW_N6/Middlewares/NetXDuo/Addons\ DHCP\ Client/subdir.mk
-include STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Src/subdir.mk
-include STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/gcc/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/Components/rtl8211/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/subdir.mk
-include STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/subdir.mk
-include Model/subdir.mk
-include Lib/lib_vision_models_pp/lib_vision_models_pp/Src/subdir.mk
-include Lib/Camera_Middleware/sensors/vd6g/subdir.mk
-include Lib/Camera_Middleware/sensors/vd55g1/subdir.mk
-include Lib/Camera_Middleware/sensors/imx335/subdir.mk
-include Lib/Camera_Middleware/sensors/subdir.mk
-include Lib/Camera_Middleware/ISP_Library/isp/Src/subdir.mk
-include Lib/Camera_Middleware/subdir.mk
-include Lib/AI_Runtime/Npu/ll_aton/subdir.mk
-include Lib/AI_Runtime/Npu/Devices/STM32N6XX/subdir.mk
-include Gcc/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := fall-guard-od
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
fall-guard-od.elf \

MAP_FILES += \
fall-guard-od.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
fall-guard-od.list \

OBJCOPY_BIN += \
fall-guard-od.bin \


# All Target
all: main-build

# Main-build Target
main-build: fall-guard-od.elf secondary-outputs

# Tool invocations
fall-guard-od.elf fall-guard-od.map: $(OBJS) $(USER_OBJS) ../../Gcc/STM32N657xx.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "fall-guard-od.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m55 -T"../../Gcc/STM32N657xx.ld" --specs=nosys.specs -Wl,-Map="fall-guard-od.map" -Wl,--gc-sections -static -Wl,--cmse-implib -Wl,--out-implib=./secure_nsclib.o -L../../Lib/AI_Runtime/Lib/GCC/ARMCortexM55 -L../../Lib/Camera_Middleware/ISP_Library/evision/Lib -u _printf_float -Wl,--no-warn-rwx-segments --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

fall-guard-od.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "fall-guard-od.list"
	@echo 'Finished building: $@'
	@echo ' '

fall-guard-od.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "fall-guard-od.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout fall-guard-od.bin fall-guard-od.elf fall-guard-od.list fall-guard-od.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
