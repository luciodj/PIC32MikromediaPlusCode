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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../Microchip/Common/TimeDelay.c ../../Microchip/Graphics/Primitive.c ../../Microchip/Graphics/GOLFontDefault.c ../../Microchip/Graphics/Drivers/SSD1963.c ../../uMB/uMedia.c ../main.c ../resources.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/385479254/TimeDelay.o ${OBJECTDIR}/_ext/375547254/Primitive.o ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o ${OBJECTDIR}/_ext/62016858/SSD1963.o ${OBJECTDIR}/_ext/1445275434/uMedia.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/resources.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/385479254/TimeDelay.o.d ${OBJECTDIR}/_ext/375547254/Primitive.o.d ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d ${OBJECTDIR}/_ext/62016858/SSD1963.o.d ${OBJECTDIR}/_ext/1445275434/uMedia.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/resources.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/385479254/TimeDelay.o ${OBJECTDIR}/_ext/375547254/Primitive.o ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o ${OBJECTDIR}/_ext/62016858/SSD1963.o ${OBJECTDIR}/_ext/1445275434/uMedia.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/resources.o

# Source Files
SOURCEFILES=../../Microchip/Common/TimeDelay.c ../../Microchip/Graphics/Primitive.c ../../Microchip/Graphics/GOLFontDefault.c ../../Microchip/Graphics/Drivers/SSD1963.c ../../uMB/uMedia.c ../main.c ../resources.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/385479254/TimeDelay.o: ../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/385479254/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/385479254/TimeDelay.o ../../Microchip/Common/TimeDelay.c   -legacy-libc
	
${OBJECTDIR}/_ext/375547254/Primitive.o: ../../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/375547254 
	@${RM} ${OBJECTDIR}/_ext/375547254/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/375547254/Primitive.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/375547254/Primitive.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/375547254/Primitive.o.d" -o ${OBJECTDIR}/_ext/375547254/Primitive.o ../../Microchip/Graphics/Primitive.c   -legacy-libc
	
${OBJECTDIR}/_ext/375547254/GOLFontDefault.o: ../../Microchip/Graphics/GOLFontDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/375547254 
	@${RM} ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d" -o ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o ../../Microchip/Graphics/GOLFontDefault.c   -legacy-libc
	
${OBJECTDIR}/_ext/62016858/SSD1963.o: ../../Microchip/Graphics/Drivers/SSD1963.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/62016858 
	@${RM} ${OBJECTDIR}/_ext/62016858/SSD1963.o.d 
	@${RM} ${OBJECTDIR}/_ext/62016858/SSD1963.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/62016858/SSD1963.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/62016858/SSD1963.o.d" -o ${OBJECTDIR}/_ext/62016858/SSD1963.o ../../Microchip/Graphics/Drivers/SSD1963.c   -legacy-libc
	
${OBJECTDIR}/_ext/1445275434/uMedia.o: ../../uMB/uMedia.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1445275434 
	@${RM} ${OBJECTDIR}/_ext/1445275434/uMedia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1445275434/uMedia.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1445275434/uMedia.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1445275434/uMedia.o.d" -o ${OBJECTDIR}/_ext/1445275434/uMedia.o ../../uMB/uMedia.c   -legacy-libc
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c   -legacy-libc
	
${OBJECTDIR}/_ext/1472/resources.o: ../resources.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/resources.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/resources.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/resources.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1472/resources.o.d" -o ${OBJECTDIR}/_ext/1472/resources.o ../resources.c   -legacy-libc
	
else
${OBJECTDIR}/_ext/385479254/TimeDelay.o: ../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/385479254 
	@${RM} ${OBJECTDIR}/_ext/385479254/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/385479254/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/385479254/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/385479254/TimeDelay.o.d" -o ${OBJECTDIR}/_ext/385479254/TimeDelay.o ../../Microchip/Common/TimeDelay.c   -legacy-libc
	
${OBJECTDIR}/_ext/375547254/Primitive.o: ../../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/375547254 
	@${RM} ${OBJECTDIR}/_ext/375547254/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/375547254/Primitive.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/375547254/Primitive.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/375547254/Primitive.o.d" -o ${OBJECTDIR}/_ext/375547254/Primitive.o ../../Microchip/Graphics/Primitive.c   -legacy-libc
	
${OBJECTDIR}/_ext/375547254/GOLFontDefault.o: ../../Microchip/Graphics/GOLFontDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/375547254 
	@${RM} ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/375547254/GOLFontDefault.o.d" -o ${OBJECTDIR}/_ext/375547254/GOLFontDefault.o ../../Microchip/Graphics/GOLFontDefault.c   -legacy-libc
	
${OBJECTDIR}/_ext/62016858/SSD1963.o: ../../Microchip/Graphics/Drivers/SSD1963.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/62016858 
	@${RM} ${OBJECTDIR}/_ext/62016858/SSD1963.o.d 
	@${RM} ${OBJECTDIR}/_ext/62016858/SSD1963.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/62016858/SSD1963.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/62016858/SSD1963.o.d" -o ${OBJECTDIR}/_ext/62016858/SSD1963.o ../../Microchip/Graphics/Drivers/SSD1963.c   -legacy-libc
	
${OBJECTDIR}/_ext/1445275434/uMedia.o: ../../uMB/uMedia.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1445275434 
	@${RM} ${OBJECTDIR}/_ext/1445275434/uMedia.o.d 
	@${RM} ${OBJECTDIR}/_ext/1445275434/uMedia.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1445275434/uMedia.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1445275434/uMedia.o.d" -o ${OBJECTDIR}/_ext/1445275434/uMedia.o ../../uMB/uMedia.c   -legacy-libc
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c   -legacy-libc
	
${OBJECTDIR}/_ext/1472/resources.o: ../resources.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/resources.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/resources.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/resources.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -ffunction-sections -mno-float -I".." -I"../../uMB" -I"../../Microchip/Include" -mcci -MMD -MF "${OBJECTDIR}/_ext/1472/resources.o.d" -o ${OBJECTDIR}/_ext/1472/resources.o ../resources.c   -legacy-libc
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -legacy-libc  -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -legacy-libc -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/3.2-Bitmaps.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
