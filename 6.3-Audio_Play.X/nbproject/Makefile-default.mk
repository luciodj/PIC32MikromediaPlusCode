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
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Microchip/Graphics/Primitive.c "../Microchip/MDD File System/FSIO.c" "../Microchip/MDD File System/SD-SPI.c" ../Microchip/Common/TimeDelay.c ../uMB/VS1053.c ../uMB/uMedia.c main.c ../Microchip/Graphics/Drivers/SSD1963.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1713430119/Primitive.o ${OBJECTDIR}/_ext/2054364014/FSIO.o ${OBJECTDIR}/_ext/2054364014/SD-SPI.o ${OBJECTDIR}/_ext/221508487/TimeDelay.o ${OBJECTDIR}/_ext/1360937979/VS1053.o ${OBJECTDIR}/_ext/1360937979/uMedia.o ${OBJECTDIR}/main.o ${OBJECTDIR}/_ext/1575306571/SSD1963.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1713430119/Primitive.o.d ${OBJECTDIR}/_ext/2054364014/FSIO.o.d ${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d ${OBJECTDIR}/_ext/221508487/TimeDelay.o.d ${OBJECTDIR}/_ext/1360937979/VS1053.o.d ${OBJECTDIR}/_ext/1360937979/uMedia.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/_ext/1575306571/SSD1963.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1713430119/Primitive.o ${OBJECTDIR}/_ext/2054364014/FSIO.o ${OBJECTDIR}/_ext/2054364014/SD-SPI.o ${OBJECTDIR}/_ext/221508487/TimeDelay.o ${OBJECTDIR}/_ext/1360937979/VS1053.o ${OBJECTDIR}/_ext/1360937979/uMedia.o ${OBJECTDIR}/main.o ${OBJECTDIR}/_ext/1575306571/SSD1963.o

# Source Files
SOURCEFILES=../Microchip/Graphics/Primitive.c ../Microchip/MDD File System/FSIO.c ../Microchip/MDD File System/SD-SPI.c ../Microchip/Common/TimeDelay.c ../uMB/VS1053.c ../uMB/uMedia.c main.c ../Microchip/Graphics/Drivers/SSD1963.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1713430119/Primitive.o: ../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1713430119 
	@${RM} ${OBJECTDIR}/_ext/1713430119/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/1713430119/Primitive.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1713430119/Primitive.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1713430119/Primitive.o.d" -o ${OBJECTDIR}/_ext/1713430119/Primitive.o ../Microchip/Graphics/Primitive.c   
	
${OBJECTDIR}/_ext/2054364014/FSIO.o: ../Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2054364014 
	@${RM} ${OBJECTDIR}/_ext/2054364014/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/2054364014/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2054364014/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/2054364014/FSIO.o.d" -o ${OBJECTDIR}/_ext/2054364014/FSIO.o "../Microchip/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/2054364014/SD-SPI.o: ../Microchip/MDD\ File\ System/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2054364014 
	@${RM} ${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/2054364014/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/2054364014/SD-SPI.o "../Microchip/MDD File System/SD-SPI.c"   
	
${OBJECTDIR}/_ext/221508487/TimeDelay.o: ../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/221508487 
	@${RM} ${OBJECTDIR}/_ext/221508487/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/221508487/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/221508487/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/221508487/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/221508487/TimeDelay.o ../Microchip/Common/TimeDelay.c   
	
${OBJECTDIR}/_ext/1360937979/VS1053.o: ../uMB/VS1053.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937979 
	@${RM} ${OBJECTDIR}/_ext/1360937979/VS1053.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937979/VS1053.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937979/VS1053.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1360937979/VS1053.o.d" -o ${OBJECTDIR}/_ext/1360937979/VS1053.o ../uMB/VS1053.c   
	
${OBJECTDIR}/_ext/1360937979/uMedia.o: ../uMB/uMedia.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937979 
	@${RM} ${OBJECTDIR}/_ext/1360937979/uMedia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937979/uMedia.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937979/uMedia.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1360937979/uMedia.o.d" -o ${OBJECTDIR}/_ext/1360937979/uMedia.o ../uMB/uMedia.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/_ext/1575306571/SSD1963.o: ../Microchip/Graphics/Drivers/SSD1963.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1575306571 
	@${RM} ${OBJECTDIR}/_ext/1575306571/SSD1963.o.d 
	@${RM} ${OBJECTDIR}/_ext/1575306571/SSD1963.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1575306571/SSD1963.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1575306571/SSD1963.o.d" -o ${OBJECTDIR}/_ext/1575306571/SSD1963.o ../Microchip/Graphics/Drivers/SSD1963.c   
	
else
${OBJECTDIR}/_ext/1713430119/Primitive.o: ../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1713430119 
	@${RM} ${OBJECTDIR}/_ext/1713430119/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/1713430119/Primitive.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1713430119/Primitive.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1713430119/Primitive.o.d" -o ${OBJECTDIR}/_ext/1713430119/Primitive.o ../Microchip/Graphics/Primitive.c   
	
${OBJECTDIR}/_ext/2054364014/FSIO.o: ../Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2054364014 
	@${RM} ${OBJECTDIR}/_ext/2054364014/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/2054364014/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2054364014/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/2054364014/FSIO.o.d" -o ${OBJECTDIR}/_ext/2054364014/FSIO.o "../Microchip/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/2054364014/SD-SPI.o: ../Microchip/MDD\ File\ System/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2054364014 
	@${RM} ${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/2054364014/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/2054364014/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/2054364014/SD-SPI.o "../Microchip/MDD File System/SD-SPI.c"   
	
${OBJECTDIR}/_ext/221508487/TimeDelay.o: ../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/221508487 
	@${RM} ${OBJECTDIR}/_ext/221508487/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/221508487/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/221508487/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/221508487/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/221508487/TimeDelay.o ../Microchip/Common/TimeDelay.c   
	
${OBJECTDIR}/_ext/1360937979/VS1053.o: ../uMB/VS1053.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937979 
	@${RM} ${OBJECTDIR}/_ext/1360937979/VS1053.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937979/VS1053.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937979/VS1053.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1360937979/VS1053.o.d" -o ${OBJECTDIR}/_ext/1360937979/VS1053.o ../uMB/VS1053.c   
	
${OBJECTDIR}/_ext/1360937979/uMedia.o: ../uMB/uMedia.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937979 
	@${RM} ${OBJECTDIR}/_ext/1360937979/uMedia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937979/uMedia.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937979/uMedia.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1360937979/uMedia.o.d" -o ${OBJECTDIR}/_ext/1360937979/uMedia.o ../uMB/uMedia.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/_ext/1575306571/SSD1963.o: ../Microchip/Graphics/Drivers/SSD1963.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1575306571 
	@${RM} ${OBJECTDIR}/_ext/1575306571/SSD1963.o.d 
	@${RM} ${OBJECTDIR}/_ext/1575306571/SSD1963.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1575306571/SSD1963.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../uMB" -I"../Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1575306571/SSD1963.o.d" -o ${OBJECTDIR}/_ext/1575306571/SSD1963.o ../Microchip/Graphics/Drivers/SSD1963.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}           -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/6.3-Audio_Play.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
