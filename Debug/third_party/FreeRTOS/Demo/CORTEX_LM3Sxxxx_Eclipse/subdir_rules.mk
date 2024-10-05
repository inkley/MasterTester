################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-249887599:
	@$(MAKE) --no-print-directory -Onone -f third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/subdir_rules.mk build-249887599-inproc

build-249887599-inproc: ../third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/fury_ft2232.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"D:/ti/ccs1260/xdctools_3_62_01_16_core/xs" --xdcpath= xdc.tools.configuro -o configPkg -r debug -c "D:/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-249887599 ../third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/fury_ft2232.cfg
configPkg/compiler.opt: build-249887599
configPkg: build-249887599

build-1583930463:
	@$(MAKE) --no-print-directory -Onone -f third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/subdir_rules.mk build-1583930463-inproc

build-1583930463-inproc: ../third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/fury_ft2232_flash.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"D:/ti/ccs1260/xdctools_3_62_01_16_core/xs" --xdcpath= xdc.tools.configuro -o configPkg -r debug -c "D:/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1583930463 ../third_party/FreeRTOS/Demo/CORTEX_LM3Sxxxx_Eclipse/fury_ft2232_flash.cfg
configPkg/compiler.opt: build-1583930463
configPkg: build-1583930463


