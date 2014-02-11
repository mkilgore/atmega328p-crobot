################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/motores.c \
../src/myDelay.c \
../src/sensorIR.c \
../src/uart.c \
../src/ultrasom.c \
../src/ultrasom_stepper.c 

OBJS += \
./src/motores.o \
./src/myDelay.o \
./src/sensorIR.o \
./src/uart.o \
./src/ultrasom.o \
./src/ultrasom_stepper.o 

C_DEPS += \
./src/motores.d \
./src/myDelay.d \
./src/sensorIR.d \
./src/uart.d \
./src/ultrasom.d \
./src/ultrasom_stepper.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I/home/victor/workspace/robot_all_in_c/robot_aic/src -Wall -O3 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


