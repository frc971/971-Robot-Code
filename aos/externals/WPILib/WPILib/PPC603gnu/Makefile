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


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/CancelButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/CancelButtonScheduler.cpp $(FORCE_FILE_BUILD)
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


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ToggleButtonScheduler.o : $(PRJ_ROOT_DIR)/Buttons/ToggleButtonScheduler.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Trigger.o : $(PRJ_ROOT_DIR)/Buttons/Trigger.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.o : $(PRJ_ROOT_DIR)/CANJaguar.cpp $(FORCE_FILE_BUILD)
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


WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicColorSensor.o : $(PRJ_ROOT_DIR)/HiTechnicColorSensor.cpp $(FORCE_FILE_BUILD)
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


WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindow.o : $(PRJ_ROOT_DIR)/LiveWindow/LiveWindow.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindowStatusListener.o : $(PRJ_ROOT_DIR)/LiveWindow/LiveWindowStatusListener.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Module.o : $(PRJ_ROOT_DIR)/Module.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.o : $(PRJ_ROOT_DIR)/MotorSafetyHelper.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.o : $(PRJ_ROOT_DIR)/Notifier.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Synchronized.o : $(PRJ_ROOT_DIR)/OSAL/Synchronized.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Task.o : $(PRJ_ROOT_DIR)/OSAL/Task.cpp $(FORCE_FILE_BUILD)
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


WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.o : $(PRJ_ROOT_DIR)/SmartDashboard/SmartDashboard.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.o : $(PRJ_ROOT_DIR)/Solenoid.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.o : $(PRJ_ROOT_DIR)/SolenoidBase.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.o : $(PRJ_ROOT_DIR)/Synchronized.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/Talon.o : $(PRJ_ROOT_DIR)/Talon.cpp $(FORCE_FILE_BUILD)
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


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTable.o : $(PRJ_ROOT_DIR)/networktables/NetworkTable.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableConnectionListenerAdapter.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableConnectionListenerAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableKeyListenerAdapter.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableKeyListenerAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableListenerAdapter.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableListenerAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableMode.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableMode.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableProvider.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableProvider.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableSubListenerAdapter.o : $(PRJ_ROOT_DIR)/networktables/NetworkTableSubListenerAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/AbstractNetworkTableEntryStore.o : $(PRJ_ROOT_DIR)/networktables2/AbstractNetworkTableEntryStore.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableEntry.o : $(PRJ_ROOT_DIR)/networktables2/NetworkTableEntry.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableNode.o : $(PRJ_ROOT_DIR)/networktables2/NetworkTableNode.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/OutgoingEntryReciever.o : $(PRJ_ROOT_DIR)/networktables2/OutgoingEntryReciever.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TableKeyExistsWithDifferentTypeException.o : $(PRJ_ROOT_DIR)/networktables2/TableKeyExistsWithDifferentTypeException.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TransactionDirtier.o : $(PRJ_ROOT_DIR)/networktables2/TransactionDirtier.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/WriteManager.o : $(PRJ_ROOT_DIR)/networktables2/WriteManager.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionAdapter.o : $(PRJ_ROOT_DIR)/networktables2/client/ClientConnectionAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionState.o : $(PRJ_ROOT_DIR)/networktables2/client/ClientConnectionState.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientNetworkTableEntryStore.o : $(PRJ_ROOT_DIR)/networktables2/client/ClientNetworkTableEntryStore.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/NetworkTableClient.o : $(PRJ_ROOT_DIR)/networktables2/client/NetworkTableClient.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/BadMessageException.o : $(PRJ_ROOT_DIR)/networktables2/connection/BadMessageException.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/ConnectionMonitorThread.o : $(PRJ_ROOT_DIR)/networktables2/connection/ConnectionMonitorThread.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/DataIOStream.o : $(PRJ_ROOT_DIR)/networktables2/connection/DataIOStream.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/NetworkTableConnection.o : $(PRJ_ROOT_DIR)/networktables2/connection/NetworkTableConnection.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/NetworkTableServer.o : $(PRJ_ROOT_DIR)/networktables2/server/NetworkTableServer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionAdapter.o : $(PRJ_ROOT_DIR)/networktables2/server/ServerConnectionAdapter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionList.o : $(PRJ_ROOT_DIR)/networktables2/server/ServerConnectionList.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionState.o : $(PRJ_ROOT_DIR)/networktables2/server/ServerConnectionState.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerIncomingStreamMonitor.o : $(PRJ_ROOT_DIR)/networktables2/server/ServerIncomingStreamMonitor.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerNetworkTableEntryStore.o : $(PRJ_ROOT_DIR)/networktables2/server/ServerNetworkTableEntryStore.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/FDIOStream.o : $(PRJ_ROOT_DIR)/networktables2/stream/FDIOStream.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketServerStreamProvider.o : $(PRJ_ROOT_DIR)/networktables2/stream/SocketServerStreamProvider.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreamFactory.o : $(PRJ_ROOT_DIR)/networktables2/stream/SocketStreamFactory.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreams.o : $(PRJ_ROOT_DIR)/networktables2/stream/SocketStreams.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/thread/DefaultThreadManger.o : $(PRJ_ROOT_DIR)/networktables2/thread/DefaultThreadManger.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayData.o : $(PRJ_ROOT_DIR)/networktables2/type/ArrayData.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayEntryType.o : $(PRJ_ROOT_DIR)/networktables2/type/ArrayEntryType.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/BooleanArray.o : $(PRJ_ROOT_DIR)/networktables2/type/BooleanArray.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexData.o : $(PRJ_ROOT_DIR)/networktables2/type/ComplexData.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexEntryType.o : $(PRJ_ROOT_DIR)/networktables2/type/ComplexEntryType.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/DefaultEntryTypes.o : $(PRJ_ROOT_DIR)/networktables2/type/DefaultEntryTypes.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryType.o : $(PRJ_ROOT_DIR)/networktables2/type/NetworkTableEntryType.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryTypeManager.o : $(PRJ_ROOT_DIR)/networktables2/type/NetworkTableEntryTypeManager.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NumberArray.o : $(PRJ_ROOT_DIR)/networktables2/type/NumberArray.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/StringArray.o : $(PRJ_ROOT_DIR)/networktables2/type/StringArray.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/EOFException.o : $(PRJ_ROOT_DIR)/networktables2/util/EOFException.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IOException.o : $(PRJ_ROOT_DIR)/networktables2/util/IOException.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IllegalStateException.o : $(PRJ_ROOT_DIR)/networktables2/util/IllegalStateException.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/StringCache.o : $(PRJ_ROOT_DIR)/networktables2/util/StringCache.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/System.o : $(PRJ_ROOT_DIR)/networktables2/util/System.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL -D'SVN_REV="$(shell svnversion -n ..)"' $(DEFINES) -o "$@" -c "$<"


WPILib/$(MODE_DIR)/Objects/WPILib/tables/TableKeyNotDefinedException.o : $(PRJ_ROOT_DIR)/tables/TableKeyNotDefinedException.cpp $(FORCE_FILE_BUILD)
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
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/CancelButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/DigitalIOButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/HeldButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/InternalButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/JoystickButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/NetworkButton.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/PressedButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ReleasedButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ToggleButtonScheduler.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Trigger.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.o \
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
	 WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicColorSensor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicCompass.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/I2C.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/InterruptableSensorBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/IterativeRobot.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Jaguar.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Joystick.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Kinect.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/KinectStick.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindow.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindowStatusListener.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Module.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Synchronized.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Task.o \
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
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Talon.o \
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
	 WPILib/$(MODE_DIR)/Objects/WPILib/Watchdog.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTable.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableConnectionListenerAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableKeyListenerAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableListenerAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableMode.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableProvider.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableSubListenerAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/AbstractNetworkTableEntryStore.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableEntry.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableNode.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/OutgoingEntryReciever.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TableKeyExistsWithDifferentTypeException.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TransactionDirtier.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/WriteManager.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionState.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientNetworkTableEntryStore.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/NetworkTableClient.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/BadMessageException.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/ConnectionMonitorThread.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/DataIOStream.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/NetworkTableConnection.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/NetworkTableServer.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionAdapter.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionList.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionState.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerIncomingStreamMonitor.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerNetworkTableEntryStore.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/FDIOStream.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketServerStreamProvider.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreamFactory.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreams.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/thread/DefaultThreadManger.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayData.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayEntryType.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/BooleanArray.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexData.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexEntryType.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/DefaultEntryTypes.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryType.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryTypeManager.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NumberArray.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/StringArray.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/EOFException.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IOException.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IllegalStateException.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/StringCache.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/System.o \
	 WPILib/$(MODE_DIR)/Objects/WPILib/tables/TableKeyNotDefinedException.o

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
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/CancelButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/DigitalIOButton.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/HeldButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/InternalButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/JoystickButton.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/NetworkButton.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/PressedButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ReleasedButtonScheduler.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/ToggleButtonScheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Buttons/Trigger.d WPILib/$(MODE_DIR)/Objects/WPILib/CANJaguar.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Command.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroup.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/CommandGroupEntry.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PIDSubsystem.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/PrintCommand.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Scheduler.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/StartCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/Subsystem.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitCommand.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitForChildren.d WPILib/$(MODE_DIR)/Objects/WPILib/Commands/WaitUntilCommand.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Compressor.d WPILib/$(MODE_DIR)/Objects/WPILib/Counter.d WPILib/$(MODE_DIR)/Objects/WPILib/Dashboard.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalInput.d WPILib/$(MODE_DIR)/Objects/WPILib/DigitalModule.d WPILib/$(MODE_DIR)/Objects/WPILib/DigitalOutput.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DigitalSource.d WPILib/$(MODE_DIR)/Objects/WPILib/DoubleSolenoid.d WPILib/$(MODE_DIR)/Objects/WPILib/DriverStation.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationEnhancedIO.d WPILib/$(MODE_DIR)/Objects/WPILib/DriverStationLCD.d WPILib/$(MODE_DIR)/Objects/WPILib/Encoder.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Error.d WPILib/$(MODE_DIR)/Objects/WPILib/ErrorBase.d WPILib/$(MODE_DIR)/Objects/WPILib/GearTooth.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Gyro.d WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicColorSensor.d WPILib/$(MODE_DIR)/Objects/WPILib/HiTechnicCompass.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/I2C.d WPILib/$(MODE_DIR)/Objects/WPILib/InterruptableSensorBase.d WPILib/$(MODE_DIR)/Objects/WPILib/IterativeRobot.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Jaguar.d WPILib/$(MODE_DIR)/Objects/WPILib/Joystick.d WPILib/$(MODE_DIR)/Objects/WPILib/Kinect.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/KinectStick.d WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindow.d WPILib/$(MODE_DIR)/Objects/WPILib/LiveWindow/LiveWindowStatusListener.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Module.d WPILib/$(MODE_DIR)/Objects/WPILib/MotorSafetyHelper.d WPILib/$(MODE_DIR)/Objects/WPILib/Notifier.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Synchronized.d WPILib/$(MODE_DIR)/Objects/WPILib/OSAL/Task.d WPILib/$(MODE_DIR)/Objects/WPILib/PIDController.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/PWM.d WPILib/$(MODE_DIR)/Objects/WPILib/Preferences.d WPILib/$(MODE_DIR)/Objects/WPILib/Relay.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Resource.d WPILib/$(MODE_DIR)/Objects/WPILib/RobotBase.d WPILib/$(MODE_DIR)/Objects/WPILib/RobotDrive.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SPI.d WPILib/$(MODE_DIR)/Objects/WPILib/SafePWM.d WPILib/$(MODE_DIR)/Objects/WPILib/SensorBase.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SerialPort.d WPILib/$(MODE_DIR)/Objects/WPILib/Servo.d WPILib/$(MODE_DIR)/Objects/WPILib/SimpleRobot.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SendableChooser.d WPILib/$(MODE_DIR)/Objects/WPILib/SmartDashboard/SmartDashboard.d WPILib/$(MODE_DIR)/Objects/WPILib/Solenoid.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/SolenoidBase.d WPILib/$(MODE_DIR)/Objects/WPILib/Synchronized.d WPILib/$(MODE_DIR)/Objects/WPILib/Talon.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Task.d WPILib/$(MODE_DIR)/Objects/WPILib/Timer.d WPILib/$(MODE_DIR)/Objects/WPILib/Ultrasonic.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Utility.d WPILib/$(MODE_DIR)/Objects/WPILib/Victor.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCamera.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/AxisCameraParams.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/BinaryImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ColorImage.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/EnumCameraParameter.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/HSLImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/ImageBase.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/IntCameraParameter.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/MonoImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/PCVideoServer.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision/RGBImage.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision/Threshold.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/AxisCamera.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/BaeUtilities.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/FrcError.d WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/TrackAPI.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/Vision2009/VisionAPI.d WPILib/$(MODE_DIR)/Objects/WPILib/Watchdog.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTable.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableConnectionListenerAdapter.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableKeyListenerAdapter.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableListenerAdapter.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableMode.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableProvider.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables/NetworkTableSubListenerAdapter.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/AbstractNetworkTableEntryStore.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableEntry.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/NetworkTableNode.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/OutgoingEntryReciever.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TableKeyExistsWithDifferentTypeException.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/TransactionDirtier.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/WriteManager.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionAdapter.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientConnectionState.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/ClientNetworkTableEntryStore.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/client/NetworkTableClient.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/BadMessageException.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/ConnectionMonitorThread.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/DataIOStream.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/connection/NetworkTableConnection.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/NetworkTableServer.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionAdapter.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionList.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerConnectionState.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerIncomingStreamMonitor.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/server/ServerNetworkTableEntryStore.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/FDIOStream.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketServerStreamProvider.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreamFactory.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/stream/SocketStreams.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/thread/DefaultThreadManger.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayData.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ArrayEntryType.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/BooleanArray.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexData.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/ComplexEntryType.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/DefaultEntryTypes.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryType.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NetworkTableEntryTypeManager.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/NumberArray.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/type/StringArray.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/EOFException.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IOException.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/IllegalStateException.d \
	 WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/StringCache.d WPILib/$(MODE_DIR)/Objects/WPILib/networktables2/util/System.d WPILib/$(MODE_DIR)/Objects/WPILib/tables/TableKeyNotDefinedException.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/PCVideoServer.d JavaCameraLib/$(MODE_DIR)/Objects/AxisCamera.d JavaCameraLib/$(MODE_DIR)/Objects/AxisCameraParams.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/EnumCameraParameter.d JavaCameraLib/$(MODE_DIR)/Objects/IntCameraParameter.d JavaCameraLib/$(MODE_DIR)/Objects/Error.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/ErrorBase.d JavaCameraLib/$(MODE_DIR)/Objects/Task.d JavaCameraLib/$(MODE_DIR)/Objects/Timer.d \
	 JavaCameraLib/$(MODE_DIR)/Objects/Synchronized.d JavaCameraLib/$(MODE_DIR)/Objects/Utility.d
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
