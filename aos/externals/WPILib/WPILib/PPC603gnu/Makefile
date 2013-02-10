# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/WPILib



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I.. -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = WPILib/$(MODE_DIR)/WPILib.a \
	 JavaCameraLib/$(MODE_DIR)/JavaCameraLib.a


#Rules

# WPILib
ifeq ($(DEBUG_MODE),1)
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
WPILib/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
WPILib/$(MODE_DIR)/% : IDE_INCLUDES = -I.. -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
WPILib/$(MODE_DIR)/% : IDE_LIBRARIES = 
WPILib/$(MODE_DIR)/% : IDE_DEFINES = 
WPILib/$(MODE_DIR)/% : PROJECT_TYPE = DKM
WPILib/$(MODE_DIR)/% : DEFINES = 
WPILib/$(MODE_DIR)/% : EXPAND_DBG = 0
WPILib/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
WPILib/$(MODE_DIR)/% : CPU = PPC603
WPILib/$(MODE_DIR)/% : TOOL_FAMILY = gnu
WPILib/$(MODE_DIR)/% : TOOL = gnu
WPILib/$(MODE_DIR)/% : TOOL_PATH = 
WPILib/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
WPILib/$(MODE_DIR)/% : LIBPATH = 
WPILib/$(MODE_DIR)/% : LIBS = 
WPILib/$(MODE_DIR)/% : OBJ_DIR := WPILib/$(MODE_DIR)

WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_I2C.o : $(PRJ_ROOT_DIR)/ADXL345_I2C.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_SPI.o : $(PRJ_ROOT_DIR)/ADXL345_SPI.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Accelerometer.o : $(PRJ_ROOT_DIR)/Accelerometer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/AnalogChannel.o : $(PRJ_ROOT_DIR)/AnalogChannel.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/AnalogModule.o : $(PRJ_ROOT_DIR)/AnalogModule.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTrigger.o : $(PRJ_ROOT_DIR)/AnalogTrigger.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTriggerOutput.o : $(PRJ_ROOT_DIR)/AnalogTriggerOutput.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/AnalogIOButton.o : $(PRJ_ROOT_DIR)/Buttons/AnalogIOButton.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Button.o : $(PRJ_ROOT_DIR)/Buttons/Button.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/ButtonScheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/DigitalIOButton.o : $(PRJ_ROOT_DIR)/Buttons/DigitalIOButton.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/HeldButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/HeldButtonScheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/InternalButton.o : $(PRJ_ROOT_DIR)/Buttons/InternalButton.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/JoystickButton.o : $(PRJ_ROOT_DIR)/Buttons/JoystickButton.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/NetworkButton.o : $(PRJ_ROOT_DIR)/Buttons/NetworkButton.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/PressedButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/PressedButtonScheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ReleasedButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/ReleasedButtonScheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.o : $(PRJ_ROOT_DIR)/CANJaguar.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAccelerometer.o : $(PRJ_ROOT_DIR)/CInterfaces/CAccelerometer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAnalogChannel.o : $(PRJ_ROOT_DIR)/CInterfaces/CAnalogChannel.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCompressor.o : $(PRJ_ROOT_DIR)/CInterfaces/CCompressor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCounter.o : $(PRJ_ROOT_DIR)/CInterfaces/CCounter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalInput.o : $(PRJ_ROOT_DIR)/CInterfaces/CDigitalInput.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalOutput.o : $(PRJ_ROOT_DIR)/CInterfaces/CDigitalOutput.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDriverStation.o : $(PRJ_ROOT_DIR)/CInterfaces/CDriverStation.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CEncoder.o : $(PRJ_ROOT_DIR)/CInterfaces/CEncoder.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGearTooth.o : $(PRJ_ROOT_DIR)/CInterfaces/CGearTooth.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGyro.o : $(PRJ_ROOT_DIR)/CInterfaces/CGyro.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJaguar.o : $(PRJ_ROOT_DIR)/CInterfaces/CJaguar.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJoystick.o : $(PRJ_ROOT_DIR)/CInterfaces/CJoystick.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CPWM.o : $(PRJ_ROOT_DIR)/CInterfaces/CPWM.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRelay.o : $(PRJ_ROOT_DIR)/CInterfaces/CRelay.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRobotDrive.o : $(PRJ_ROOT_DIR)/CInterfaces/CRobotDrive.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSerialPort.o : $(PRJ_ROOT_DIR)/CInterfaces/CSerialPort.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CServo.o : $(PRJ_ROOT_DIR)/CInterfaces/CServo.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSolenoid.o : $(PRJ_ROOT_DIR)/CInterfaces/CSolenoid.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CTimer.o : $(PRJ_ROOT_DIR)/CInterfaces/CTimer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CUltrasonic.o : $(PRJ_ROOT_DIR)/CInterfaces/CUltrasonic.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CVictor.o : $(PRJ_ROOT_DIR)/CInterfaces/CVictor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CWatchdog.o : $(PRJ_ROOT_DIR)/CInterfaces/CWatchdog.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/SimpleCRobot.o : $(PRJ_ROOT_DIR)/CInterfaces/SimpleCRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Command.o : $(PRJ_ROOT_DIR)/Commands/Command.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroup.o : $(PRJ_ROOT_DIR)/Commands/CommandGroup.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroupEntry.o : $(PRJ_ROOT_DIR)/Commands/CommandGroupEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDCommand.o : $(PRJ_ROOT_DIR)/Commands/PIDCommand.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDSubsystem.o : $(PRJ_ROOT_DIR)/Commands/PIDSubsystem.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PrintCommand.o : $(PRJ_ROOT_DIR)/Commands/PrintCommand.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Scheduler.o : $(PRJ_ROOT_DIR)/Commands/Scheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/StartCommand.o : $(PRJ_ROOT_DIR)/Commands/StartCommand.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Subsystem.o : $(PRJ_ROOT_DIR)/Commands/Subsystem.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitCommand.o : $(PRJ_ROOT_DIR)/Commands/WaitCommand.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitForChildren.o : $(PRJ_ROOT_DIR)/Commands/WaitForChildren.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitUntilCommand.o : $(PRJ_ROOT_DIR)/Commands/WaitUntilCommand.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Compressor.o : $(PRJ_ROOT_DIR)/Compressor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Counter.o : $(PRJ_ROOT_DIR)/Counter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Dashboard.o : $(PRJ_ROOT_DIR)/Dashboard.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DigitalInput.o : $(PRJ_ROOT_DIR)/DigitalInput.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DigitalModule.o : $(PRJ_ROOT_DIR)/DigitalModule.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DigitalOutput.o : $(PRJ_ROOT_DIR)/DigitalOutput.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DigitalSource.o : $(PRJ_ROOT_DIR)/DigitalSource.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DoubleSolenoid.o : $(PRJ_ROOT_DIR)/DoubleSolenoid.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DriverStation.o : $(PRJ_ROOT_DIR)/DriverStation.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationEnhancedIO.o : $(PRJ_ROOT_DIR)/DriverStationEnhancedIO.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationLCD.o : $(PRJ_ROOT_DIR)/DriverStationLCD.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Encoder.o : $(PRJ_ROOT_DIR)/Encoder.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Error.o : $(PRJ_ROOT_DIR)/Error.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/ErrorBase.o : $(PRJ_ROOT_DIR)/ErrorBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/GearTooth.o : $(PRJ_ROOT_DIR)/GearTooth.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Gyro.o : $(PRJ_ROOT_DIR)/Gyro.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicCompass.o : $(PRJ_ROOT_DIR)/HiTechnicCompass.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/I2C.o : $(PRJ_ROOT_DIR)/I2C.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/InterruptableSensorBase.o : $(PRJ_ROOT_DIR)/InterruptableSensorBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/IterativeRobot.o : $(PRJ_ROOT_DIR)/IterativeRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Jaguar.o : $(PRJ_ROOT_DIR)/Jaguar.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Joystick.o : $(PRJ_ROOT_DIR)/Joystick.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Kinect.o : $(PRJ_ROOT_DIR)/Kinect.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/KinectStick.o : $(PRJ_ROOT_DIR)/KinectStick.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Module.o : $(PRJ_ROOT_DIR)/Module.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.o : $(PRJ_ROOT_DIR)/MotorSafetyHelper.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/BooleanEntry.o : $(PRJ_ROOT_DIR)/NetworkTables/BooleanEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Buffer.o : $(PRJ_ROOT_DIR)/NetworkTables/Buffer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Confirmation.o : $(PRJ_ROOT_DIR)/NetworkTables/Confirmation.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Connection.o : $(PRJ_ROOT_DIR)/NetworkTables/Connection.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/ConnectionManager.o : $(PRJ_ROOT_DIR)/NetworkTables/ConnectionManager.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Data.o : $(PRJ_ROOT_DIR)/NetworkTables/Data.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Denial.o : $(PRJ_ROOT_DIR)/NetworkTables/Denial.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/DoubleEntry.o : $(PRJ_ROOT_DIR)/NetworkTables/DoubleEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Entry.o : $(PRJ_ROOT_DIR)/NetworkTables/Entry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/IntegerEntry.o : $(PRJ_ROOT_DIR)/NetworkTables/IntegerEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Key.o : $(PRJ_ROOT_DIR)/NetworkTables/Key.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkQueue.o : $(PRJ_ROOT_DIR)/NetworkTables/NetworkQueue.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkTable.o : $(PRJ_ROOT_DIR)/NetworkTables/NetworkTable.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/OldData.o : $(PRJ_ROOT_DIR)/NetworkTables/OldData.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Reader.o : $(PRJ_ROOT_DIR)/NetworkTables/Reader.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/StringEntry.o : $(PRJ_ROOT_DIR)/NetworkTables/StringEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableAssignment.o : $(PRJ_ROOT_DIR)/NetworkTables/TableAssignment.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableEntry.o : $(PRJ_ROOT_DIR)/NetworkTables/TableEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionEnd.o : $(PRJ_ROOT_DIR)/NetworkTables/TransactionEnd.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionStart.o : $(PRJ_ROOT_DIR)/NetworkTables/TransactionStart.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.o : $(PRJ_ROOT_DIR)/Notifier.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/PIDController.o : $(PRJ_ROOT_DIR)/PIDController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/PWM.o : $(PRJ_ROOT_DIR)/PWM.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Preferences.o : $(PRJ_ROOT_DIR)/Preferences.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Relay.o : $(PRJ_ROOT_DIR)/Relay.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Resource.o : $(PRJ_ROOT_DIR)/Resource.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/RobotBase.o : $(PRJ_ROOT_DIR)/RobotBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/RobotDrive.o : $(PRJ_ROOT_DIR)/RobotDrive.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SPI.o : $(PRJ_ROOT_DIR)/SPI.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SafePWM.o : $(PRJ_ROOT_DIR)/SafePWM.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SensorBase.o : $(PRJ_ROOT_DIR)/SensorBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SerialPort.o : $(PRJ_ROOT_DIR)/SerialPort.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Servo.o : $(PRJ_ROOT_DIR)/Servo.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SimpleRobot.o : $(PRJ_ROOT_DIR)/SimpleRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableChooser.o : $(PRJ_ROOT_DIR)/SmartDashboard/SendableChooser.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableGyro.o : $(PRJ_ROOT_DIR)/SmartDashboard/SendableGyro.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendablePIDController.o : $(PRJ_ROOT_DIR)/SmartDashboard/SendablePIDController.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.o : $(PRJ_ROOT_DIR)/SmartDashboard/SmartDashboard.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.o : $(PRJ_ROOT_DIR)/Solenoid.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.o : $(PRJ_ROOT_DIR)/SolenoidBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.o : $(PRJ_ROOT_DIR)/Synchronized.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Task.o : $(PRJ_ROOT_DIR)/Task.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Timer.o : $(PRJ_ROOT_DIR)/Timer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Ultrasonic.o : $(PRJ_ROOT_DIR)/Ultrasonic.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Utility.o : $(PRJ_ROOT_DIR)/Utility.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Victor.o : $(PRJ_ROOT_DIR)/Victor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCamera.o : $(PRJ_ROOT_DIR)/Vision/AxisCamera.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCameraParams.o : $(PRJ_ROOT_DIR)/Vision/AxisCameraParams.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/BinaryImage.o : $(PRJ_ROOT_DIR)/Vision/BinaryImage.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ColorImage.o : $(PRJ_ROOT_DIR)/Vision/ColorImage.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/EnumCameraParameter.o : $(PRJ_ROOT_DIR)/Vision/EnumCameraParameter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/HSLImage.o : $(PRJ_ROOT_DIR)/Vision/HSLImage.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ImageBase.o : $(PRJ_ROOT_DIR)/Vision/ImageBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/IntCameraParameter.o : $(PRJ_ROOT_DIR)/Vision/IntCameraParameter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/MonoImage.o : $(PRJ_ROOT_DIR)/Vision/MonoImage.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/PCVideoServer.o : $(PRJ_ROOT_DIR)/Vision/PCVideoServer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/RGBImage.o : $(PRJ_ROOT_DIR)/Vision/RGBImage.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision/Threshold.o : $(PRJ_ROOT_DIR)/Vision/Threshold.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/AxisCamera.o : $(PRJ_ROOT_DIR)/Vision2009/AxisCamera.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/BaeUtilities.o : $(PRJ_ROOT_DIR)/Vision2009/BaeUtilities.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/FrcError.o : $(PRJ_ROOT_DIR)/Vision2009/FrcError.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/TrackAPI.o : $(PRJ_ROOT_DIR)/Vision2009/TrackAPI.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/VisionAPI.o : $(PRJ_ROOT_DIR)/Vision2009/VisionAPI.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Watchdog.o : $(PRJ_ROOT_DIR)/Watchdog.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


OBJECTS_WPILib = WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_I2C.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_SPI.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Accelerometer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogChannel.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogModule.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTrigger.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTriggerOutput.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/AnalogIOButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Button.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/DigitalIOButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/HeldButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/InternalButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/JoystickButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/NetworkButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/PressedButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ReleasedButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAccelerometer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAnalogChannel.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCompressor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCounter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalInput.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalOutput.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDriverStation.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CEncoder.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGearTooth.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGyro.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJaguar.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJoystick.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CPWM.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRelay.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRobotDrive.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSerialPort.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CServo.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSolenoid.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CTimer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CUltrasonic.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CVictor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CWatchdog.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/SimpleCRobot.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Command.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroup.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroupEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDCommand.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDSubsystem.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PrintCommand.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Scheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/StartCommand.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Subsystem.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitCommand.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitForChildren.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitUntilCommand.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Compressor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Counter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Dashboard.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalInput.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalModule.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalOutput.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalSource.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DoubleSolenoid.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DriverStation.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationEnhancedIO.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationLCD.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Encoder.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Error.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/ErrorBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/GearTooth.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Gyro.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicCompass.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/I2C.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/InterruptableSensorBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/IterativeRobot.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Jaguar.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Joystick.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Kinect.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/KinectStick.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Module.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/BooleanEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Buffer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Confirmation.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Connection.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/ConnectionManager.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Data.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Denial.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/DoubleEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Entry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/IntegerEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Key.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkQueue.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkTable.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/OldData.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Reader.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/StringEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableAssignment.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionEnd.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionStart.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/PIDController.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/PWM.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Preferences.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Relay.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Resource.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/RobotBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/RobotDrive.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SPI.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SafePWM.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SensorBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SerialPort.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Servo.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SimpleRobot.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableChooser.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableGyro.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendablePIDController.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Task.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Timer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Ultrasonic.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Utility.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Victor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCamera.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCameraParams.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/BinaryImage.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ColorImage.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/EnumCameraParameter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/HSLImage.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ImageBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/IntCameraParameter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/MonoImage.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/PCVideoServer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/RGBImage.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/Threshold.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/AxisCamera.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/BaeUtilities.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/FrcError.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/TrackAPI.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/VisionAPI.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Watchdog.o

WPILib/$(MODE_DIR)/WPILib.a : $(OBJECTS_WPILib)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)arppc crus "$@" $(OBJECTS_WPILib)

WPILib/$(MODE_DIR)/WPILib_compile_file : $(FILE) ;

_clean :: WPILib/$(MODE_DIR)/WPILib_clean

WPILib/$(MODE_DIR)/WPILib_clean : 
	$(TRACE_FLAG)if [ -d "WPILib" ]; then cd "WPILib"; rm -rf $(MODE_DIR); fi


# JavaCameraLib
ifeq ($(DEBUG_MODE),1)
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
JavaCameraLib/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
JavaCameraLib/$(MODE_DIR)/% : IDE_INCLUDES = -I.. -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
JavaCameraLib/$(MODE_DIR)/% : IDE_LIBRARIES = 
JavaCameraLib/$(MODE_DIR)/% : IDE_DEFINES = 
JavaCameraLib/$(MODE_DIR)/% : PROJECT_TYPE = DKM
JavaCameraLib/$(MODE_DIR)/% : DEFINES = 
JavaCameraLib/$(MODE_DIR)/% : EXPAND_DBG = 0
JavaCameraLib/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
JavaCameraLib/$(MODE_DIR)/% : CPU = PPC603
JavaCameraLib/$(MODE_DIR)/% : TOOL_FAMILY = gnu
JavaCameraLib/$(MODE_DIR)/% : TOOL = gnu
JavaCameraLib/$(MODE_DIR)/% : TOOL_PATH = 
JavaCameraLib/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
JavaCameraLib/$(MODE_DIR)/% : LIBPATH = 
JavaCameraLib/$(MODE_DIR)/% : LIBS = 
JavaCameraLib/$(MODE_DIR)/% : OBJ_DIR := JavaCameraLib/$(MODE_DIR)

JavaCameraLib/$(MODE_DIR)/Objects/PCVideoServer.o : $(PRJ_ROOT_DIR)/Vision/PCVideoServer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/AxisCamera.o : $(PRJ_ROOT_DIR)/Vision/AxisCamera.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/AxisCameraParams.o : $(PRJ_ROOT_DIR)/Vision/AxisCameraParams.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/EnumCameraParameter.o : $(PRJ_ROOT_DIR)/Vision/EnumCameraParameter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/IntCameraParameter.o : $(PRJ_ROOT_DIR)/Vision/IntCameraParameter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/Error.o : $(PRJ_ROOT_DIR)/Error.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/ErrorBase.o : $(PRJ_ROOT_DIR)/ErrorBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/Task.o : $(PRJ_ROOT_DIR)/Task.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/Timer.o : $(PRJ_ROOT_DIR)/Timer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/Synchronized.o : $(PRJ_ROOT_DIR)/Synchronized.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


JavaCameraLib/$(MODE_DIR)/Objects/Utility.o : $(PRJ_ROOT_DIR)/Utility.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -DJAVA_CAMERA_LIB=1 -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


OBJECTS_JavaCameraLib = JavaCameraLib/$(MODE_DIR)/Objects/PCVideoServer.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/AxisCamera.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/AxisCameraParams.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/EnumCameraParameter.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/IntCameraParameter.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/Error.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/ErrorBase.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/Task.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/Timer.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/Synchronized.o \
	 JavaCameraLib/$(MODE_DIR)/Objects/Utility.o

JavaCameraLib/$(MODE_DIR)/JavaCameraLib.a : $(OBJECTS_JavaCameraLib)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)arppc crus "$@" $(OBJECTS_JavaCameraLib)

JavaCameraLib/$(MODE_DIR)/JavaCameraLib_compile_file : $(FILE) ;

_clean :: JavaCameraLib/$(MODE_DIR)/JavaCameraLib_clean

JavaCameraLib/$(MODE_DIR)/JavaCameraLib_clean : 
	$(TRACE_FLAG)if [ -d "JavaCameraLib" ]; then cd "JavaCameraLib"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_I2C.d WPILib/$(MODE_DIR)/Objects/WPILib/ADXL345_SPI.d WPILib/$(MODE_DIR)/Objects/WPILib/Accelerometer.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogChannel.d WPILib/$(MODE_DIR)/Objects/WPILib/AnalogModule.d WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTrigger.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/AnalogTriggerOutput.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/AnalogIOButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Button.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/DigitalIOButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/HeldButtonScheduler.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/InternalButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/JoystickButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/NetworkButton.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/PressedButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ReleasedButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAccelerometer.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CAnalogChannel.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCompressor.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CCounter.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalInput.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDigitalOutput.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CDriverStation.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CEncoder.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGearTooth.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CGyro.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJaguar.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CJoystick.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CPWM.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRelay.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CRobotDrive.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSerialPort.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CServo.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CSolenoid.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CTimer.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CUltrasonic.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CVictor.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/CWatchdog.d WPILib/$(MODE_DIR)/Objects/WPILib/CInterfaces/SimpleCRobot.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Command.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroup.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroupEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDCommand.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDSubsystem.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PrintCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Scheduler.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/StartCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Subsystem.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitCommand.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitForChildren.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitUntilCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Compressor.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Counter.d WPILib/$(MODE_DIR)/Objects/WPILib/Dashboard.d WPILib/$(MODE_DIR)/Objects/WPILib/DigitalInput.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalModule.d WPILib/$(MODE_DIR)/Objects/WPILib/DigitalOutput.d WPILib/$(MODE_DIR)/Objects/WPILib/DigitalSource.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DoubleSolenoid.d WPILib/$(MODE_DIR)/Objects/WPILib/DriverStation.d WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationEnhancedIO.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationLCD.d WPILib/$(MODE_DIR)/Objects/WPILib/Encoder.d WPILib/$(MODE_DIR)/Objects/WPILib/Error.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/ErrorBase.d WPILib/$(MODE_DIR)/Objects/WPILib/GearTooth.d WPILib/$(MODE_DIR)/Objects/WPILib/Gyro.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicCompass.d WPILib/$(MODE_DIR)/Objects/WPILib/I2C.d WPILib/$(MODE_DIR)/Objects/WPILib/InterruptableSensorBase.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/IterativeRobot.d WPILib/$(MODE_DIR)/Objects/WPILib/Jaguar.d WPILib/$(MODE_DIR)/Objects/WPILib/Joystick.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Kinect.d WPILib/$(MODE_DIR)/Objects/WPILib/KinectStick.d WPILib/$(MODE_DIR)/Objects/WPILib/Module.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/BooleanEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Buffer.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Confirmation.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Connection.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/ConnectionManager.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Data.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Denial.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/DoubleEntry.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Entry.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/IntegerEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Key.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkQueue.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/NetworkTable.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/OldData.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/Reader.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/StringEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableAssignment.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TableEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionEnd.d WPILib/$(MODE_DIR)/Objects/WPILib/NetworkTables/TransactionStart.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.d WPILib/$(MODE_DIR)/Objects/WPILib/PIDController.d WPILib/$(MODE_DIR)/Objects/WPILib/PWM.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Preferences.d WPILib/$(MODE_DIR)/Objects/WPILib/Relay.d WPILib/$(MODE_DIR)/Objects/WPILib/Resource.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/RobotBase.d WPILib/$(MODE_DIR)/Objects/WPILib/RobotDrive.d WPILib/$(MODE_DIR)/Objects/WPILib/SPI.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SafePWM.d WPILib/$(MODE_DIR)/Objects/WPILib/SensorBase.d WPILib/$(MODE_DIR)/Objects/WPILib/SerialPort.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Servo.d WPILib/$(MODE_DIR)/Objects/WPILib/SimpleRobot.d WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableChooser.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableGyro.d WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendablePIDController.d WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.d WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.d WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Task.d WPILib/$(MODE_DIR)/Objects/WPILib/Timer.d WPILib/$(MODE_DIR)/Objects/WPILib/Ultrasonic.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Utility.d WPILib/$(MODE_DIR)/Objects/WPILib/Victor.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCamera.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCameraParams.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/BinaryImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ColorImage.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/EnumCameraParameter.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/HSLImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ImageBase.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/IntCameraParameter.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/MonoImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/PCVideoServer.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/RGBImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/Threshold.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/AxisCamera.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/BaeUtilities.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/FrcError.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/TrackAPI.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/VisionAPI.d WPILib/$(MODE_DIR)/Objects/WPILib/Watchdog.d JavaCameraLib/$(MODE_DIR)/Objects/PCVideoServer.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/AxisCamera.d JavaCameraLib/$(MODE_DIR)/Objects/AxisCameraParams.d JavaCameraLib/$(MODE_DIR)/Objects/EnumCameraParameter.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/IntCameraParameter.d JavaCameraLib/$(MODE_DIR)/Objects/Error.d JavaCameraLib/$(MODE_DIR)/Objects/ErrorBase.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/Task.d JavaCameraLib/$(MODE_DIR)/Objects/Timer.d JavaCameraLib/$(MODE_DIR)/Objects/Synchronized.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/Utility.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
