#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/FastTransfer.c ../src/LidarDecoder.c ../src/uart_handler.c ../src/LidarDataInterpreter.c ../src/main.c ../src/app.c ../src/GroundOBJprocessing.c ../src/STD_HelperMethods.c ../src/FastTransfer1.c ../src/MultiLIDAR.c C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c ../../../../framework/system/int/src/sys_int_pic32.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/FastTransfer.o ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o ${OBJECTDIR}/_ext/1360937237/uart_handler.o ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o ${OBJECTDIR}/_ext/878039722/timers.o ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d ${OBJECTDIR}/_ext/1360937237/uart_handler.o.d ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d ${OBJECTDIR}/_ext/878039722/timers.o.d ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1360937237/FastTransfer.o ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o ${OBJECTDIR}/_ext/1360937237/uart_handler.o ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o ${OBJECTDIR}/_ext/878039722/timers.o ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o

# Source Files
SOURCEFILES=../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/system_tasks.c ../src/FastTransfer.c ../src/LidarDecoder.c ../src/uart_handler.c ../src/LidarDataInterpreter.c ../src/main.c ../src/app.c ../src/GroundOBJprocessing.c ../src/STD_HelperMethods.c ../src/FastTransfer1.c ../src/MultiLIDAR.c C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c ../../../../framework/system/int/src/sys_int_pic32.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFH064
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -DPICkit3PlatformTool=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=PICkit3PlatformTool=1
	
else
${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/340578644/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ../src/system_config/default/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/FastTransfer.o: ../src/FastTransfer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d" -o ${OBJECTDIR}/_ext/1360937237/FastTransfer.o ../src/FastTransfer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LidarDecoder.o: ../src/LidarDecoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d" -o ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o ../src/LidarDecoder.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/uart_handler.o: ../src/uart_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_handler.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_handler.o ../src/uart_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o: ../src/LidarDataInterpreter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d" -o ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o ../src/LidarDataInterpreter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o: ../src/GroundOBJprocessing.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d" -o ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o ../src/GroundOBJprocessing.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o: ../src/STD_HelperMethods.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d" -o ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o ../src/STD_HelperMethods.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/FastTransfer1.o: ../src/FastTransfer1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d" -o ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o ../src/FastTransfer1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o: ../src/MultiLIDAR.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d" -o ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o ../src/MultiLIDAR.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/878039722/timers.o: C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878039722" 
	@${RM} ${OBJECTDIR}/_ext/878039722/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/878039722/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878039722/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/878039722/timers.o.d" -o ${OBJECTDIR}/_ext/878039722/timers.o C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/878039722/lidarCalibrate.o: C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878039722" 
	@${RM} ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d 
	@${RM} ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d" -o ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/122796885/sys_int_pic32.o: ../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/122796885" 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ../../../../framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon.o ../src/system_config/default/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o: ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/340578644" 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/340578644/sys_devcon_pic32mz.o ../src/system_config/default/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/FastTransfer.o: ../src/FastTransfer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/FastTransfer.o.d" -o ${OBJECTDIR}/_ext/1360937237/FastTransfer.o ../src/FastTransfer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LidarDecoder.o: ../src/LidarDecoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LidarDecoder.o.d" -o ${OBJECTDIR}/_ext/1360937237/LidarDecoder.o ../src/LidarDecoder.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/uart_handler.o: ../src/uart_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_handler.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_handler.o ../src/uart_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o: ../src/LidarDataInterpreter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o.d" -o ${OBJECTDIR}/_ext/1360937237/LidarDataInterpreter.o ../src/LidarDataInterpreter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o: ../src/GroundOBJprocessing.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o.d" -o ${OBJECTDIR}/_ext/1360937237/GroundOBJprocessing.o ../src/GroundOBJprocessing.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o: ../src/STD_HelperMethods.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o.d" -o ${OBJECTDIR}/_ext/1360937237/STD_HelperMethods.o ../src/STD_HelperMethods.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/FastTransfer1.o: ../src/FastTransfer1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/FastTransfer1.o.d" -o ${OBJECTDIR}/_ext/1360937237/FastTransfer1.o ../src/FastTransfer1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o: ../src/MultiLIDAR.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o.d" -o ${OBJECTDIR}/_ext/1360937237/MultiLIDAR.o ../src/MultiLIDAR.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/878039722/timers.o: C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878039722" 
	@${RM} ${OBJECTDIR}/_ext/878039722/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/878039722/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878039722/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/878039722/timers.o.d" -o ${OBJECTDIR}/_ext/878039722/timers.o C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/878039722/lidarCalibrate.o: C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/878039722" 
	@${RM} ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d 
	@${RM} ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/878039722/lidarCalibrate.o.d" -o ${OBJECTDIR}/_ext/878039722/lidarCalibrate.o C:/microchip/harmony/v2_05_01/apps/LidarWiiCombo/firmware/src/lidarCalibrate.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/122796885/sys_int_pic32.o: ../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/122796885" 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/default" -I"../../../../../../John/Documents/tracee/LidarWiiCombo/firmware/src/system_config/default/framework" -I"../../../../../../../microchip/harmony/v2_02_00b/framework" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../../microchip/harmony/v2_05_01/framework" -I"../src/system_config/default/framework" -I"../../../../framework" -MMD -MF "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ../../../../framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/microchip/harmony/v2_05_01/bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a ../../../../bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -DPICkit3PlatformTool=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    C:\microchip\harmony\v2_05_01\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a ..\..\..\..\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x37F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=PICkit3PlatformTool=1,--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/microchip/harmony/v2_05_01/bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a ../../../../bin/framework/peripheral/PIC32MZ2048EFH064_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    C:\microchip\harmony\v2_05_01\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a ..\..\..\..\bin\framework\peripheral\PIC32MZ2048EFH064_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/LidarWiiCombo.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
