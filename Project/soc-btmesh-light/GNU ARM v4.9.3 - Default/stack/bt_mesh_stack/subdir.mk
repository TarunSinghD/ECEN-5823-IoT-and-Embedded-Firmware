################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stack/bt_mesh_stack/mesh_lib.c 

OBJS += \
./stack/bt_mesh_stack/mesh_lib.o 

C_DEPS += \
./stack/bt_mesh_stack/mesh_lib.d 


# Each subdirectory must supply rules for building sources it contributes
stack/bt_mesh_stack/mesh_lib.o: ../stack/bt_mesh_stack/mesh_lib.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-D__STACK_SIZE=0xa00' '-D__HEAP_SIZE=0x1200' '-DRETARGET_VCOM=1' '-DSERIAL_ECHO=1' '-DSILABS_AF_USE_HWCONF=1' '-DMESH_LIB_NATIVE=1' '-DEFR32BG13P632F512GM48=1' -I"/home/tarun/SimplicityStudio/PWM_111/soc-btmesh-light" -I"/home/tarun/SimplicityStudio/PWM_111/soc-btmesh-light/src" -I"/home/tarun/SimplicityStudio/PWM_111/soc-btmesh-light/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/ble_mesh/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/include/common" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//protocol/bluetooth_dev/include/soc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/CMSIS/Include" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/Device/SiliconLabs/EFR32BG13P/Include" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/common/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/dmadrv/config" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/dmadrv/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/sleep/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/uartdrv/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/uartdrv/config" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/gpiointerrupt/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/tempdrv/config" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emdrv/tempdrv/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/emlib/inc" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/middleware/glib" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/dmd" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/dmd/ssd2119" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/middleware/glib/glib" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//hardware/kit/EFR32BG13_BRD4104A/config" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//hardware/kit/common/bsp" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//hardware/kit/common/drivers" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//hardware/kit/EFR32MG13_BRD4104A/config" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/chip/efr32/rf/common/cortex" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/common" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//platform/radio/rail_lib/chip/efr32" -I"/home/tarun/Downloads/SimplicityStudio_v4/developer/sdks/blemesh/v1.1//app/bluetooth_dev/appbuilder/sample-apps/common/read_char" -O2 -fno-builtin -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"stack/bt_mesh_stack/mesh_lib.d" -MT"stack/bt_mesh_stack/mesh_lib.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


