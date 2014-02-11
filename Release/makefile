################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
robot_aic.lss \

FLASH_IMAGE += \
robot_aic.hex \

EEPROM_IMAGE += \
robot_aic.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: robot_aic.elf secondary-outputs

# Tool invocations
robot_aic.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,robot_aic.map -L/home/victor/workspace/robot_all_in_c/robot_aic/src -mmcu=atmega328p -o "robot_aic.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

robot_aic.lss: robot_aic.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S robot_aic.elf  >"robot_aic.lss"
	@echo 'Finished building: $@'
	@echo ' '

robot_aic.hex: robot_aic.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex robot_aic.elf  "robot_aic.hex"
	@echo 'Finished building: $@'
	@echo ' '

robot_aic.eep: robot_aic.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex robot_aic.elf  "robot_aic.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: robot_aic.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p robot_aic.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) robot_aic.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
