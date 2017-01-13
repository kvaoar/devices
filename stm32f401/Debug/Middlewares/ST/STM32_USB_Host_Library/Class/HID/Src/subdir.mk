################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c 

OBJS += \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.o: ../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F401xC -I"C:/Users/kvaoar/workspace/stm32f401/Inc" -I"C:/Users/kvaoar/workspace/stm32f401/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/kvaoar/workspace/stm32f401/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kvaoar/workspace/stm32f401/Middlewares/ST/STM32_USB_Host_Library/Core/Inc" -I"C:/Users/kvaoar/workspace/stm32f401/Middlewares/ST/STM32_USB_Host_Library/Class/HID/Inc" -I"C:/Users/kvaoar/workspace/stm32f401/Drivers/CMSIS/Include" -I"C:/Users/kvaoar/workspace/stm32f401/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/kvaoar/workspace/stm32f401/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


