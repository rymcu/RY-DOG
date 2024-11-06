
SDK_PATH ?= ../../..

ROOT_DIR := $(SDK_PATH)
BUILD_DIR := build

ifndef PROJECT_NAME
#PROJECT_NAME := program
PROJECT_NAME := $(word $(words $(subst /, ,$(dir $(abspath .)))), $(subst /, ,$(dir $(abspath .))))
else
PROJECT_NAME := $(subst /,-,$(PROJECT_NAME))
endif


USER_OBJS := 
C_SRCS :=
SECONDARY_FLASH := ../firmware/user_code/[0]code.bin 
SECONDARY_LIST := $(BUILD_DIR)/$(PROJECT_NAME).lst
LTO_OPTION = -flto
O_OPTION = -Os

# CC_PREFIX := riscv-none-embed-
CC_PREFIX := riscv-nuclei-elf-
CC = gcc
AS = gcc
LD = g++
OD = objdump
OC = objcopy
SIZE = size

ifeq ($(OS),Windows_NT)
CMDBOX := busybox
LIBS_PATH := libs
else
CMDBOX :=
LIBS_PATH := libs/linux
endif

OBJS := 

LIBS := -lm


C_FLAGS := -march=rv32imafc -mabi=ilp32f -mcmodel=medlow -msmall-data-limit=8 -msave-restore -mfdiv $(O_OPTION) -fmessage-length=0 $(LTO_OPTION)
C_FLAGS += -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-delete-null-pointer-checks -fno-unroll-loops  -g -std=gnu11 -MMD -MP
C_FLAGS += -DCI_CONFIG_FILE=\"user_config.h\" -DCORE_ID=0
# enum only cost 1 byte
C_FLAGS += -fshort-enums
S_FLAGS := -march=rv32imafc -mabi=ilp32f -mcmodel=medlow -msmall-data-limit=8 -msave-restore -mfdiv $(O_OPTION) -fmessage-length=0 $(LTO_OPTION)
S_FLAGS += -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-delete-null-pointer-checks -fno-unroll-loops  -g -x assembler-with-cpp -MMD -MP
S_FLAGS += -I$(ROOT_DIR)/components -I$(ROOT_DIR)/driver/ci130x_chip_driver/inc
LD_FLAGS := -march=rv32imafc -mabi=ilp32f -mcmodel=medlow -msmall-data-limit=8 -msave-restore -mfdiv $(O_OPTION) -fmessage-length=0	$(LTO_OPTION)
LD_FLAGS += -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-delete-null-pointer-checks -fno-unroll-loops
LD_FLAGS += -g -T "../src/ci130x.lds" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"$(BUILD_DIR)/$(PROJECT_NAME).map" --specs=nano.specs

LD_FLAGS += -Wl,--wrap,memcmp
LD_FLAGS += -Wl,--wrap,memcpy
LD_FLAGS += -Wl,--wrap,memmove
LD_FLAGS += -Wl,--wrap,memset
LD_FLAGS += -Wl,--wrap,strcat
LD_FLAGS += -Wl,--wrap,strcmp
LD_FLAGS += -Wl,--wrap,strcpy
LD_FLAGS += -Wl,--wrap,strlen
LD_FLAGS += -Wl,--wrap,strncat
LD_FLAGS += -Wl,--wrap,strncmp
LD_FLAGS += -Wl,--wrap,strncpy
LD_FLAGS += -Wl,--wrap,strstr
LD_FLAGS += -Wl,--wrap,stpcpy
LD_FLAGS += -Wl,--wrap,stpncpy
LD_FLAGS += -Wl,--wrap,strnlen
LD_FLAGS += -Wl,--wrap,strtoul
LD_FLAGS += -Wl,--wrap,atol
LD_FLAGS += -Wl,--wrap,qsort
LD_FLAGS += -Wl,--wrap,malloc
LD_FLAGS += -Wl,--wrap,free
LD_FLAGS += -Wl,--wrap,vsnprintf
LD_FLAGS += -Wl,--wrap,sprintf
LD_FLAGS += -Wl,--wrap,sscanf
LD_FLAGS += -Wl,--wrap,abs
LD_FLAGS += -Wl,--wrap,isnan
LD_FLAGS += -Wl,--wrap,isinf
LD_FLAGS += -Wl,--wrap,cosf
LD_FLAGS += -Wl,--wrap,sinf
LD_FLAGS += -Wl,--wrap,tanf
LD_FLAGS += -Wl,--wrap,expf
LD_FLAGS += -Wl,--wrap,sqrtf
LD_FLAGS += -Wl,--wrap,fabsf
LD_FLAGS += -Wl,--wrap,logf
LD_FLAGS += -Wl,--wrap,log10f
LD_FLAGS += -Wl,--wrap,atoi

LD_FLAGS += -L$(ROOT_DIR)/$(LIBS_PATH) -L$(ROOT_DIR)/components/freertos/portable/GCC
LD_FLAGS += -L$(ROOT_DIR)/components/asr/decoder -L$(ROOT_DIR)/utils

all: secondary-outputs

ifneq ($(MAKECMDGOALS), clean)
-include $(BUILD_DIR)/source_file.mk

# ifeq ($(OS),Windows_NT)
LIB_FILES += $(ROOT_DIR)/$(LIBS_PATH)/libcikd_asrBc.a
LIBS += -lcikd_asrBc
# else
# LIB_FILES += $(ROOT_DIR)/$(LIBS_PATH)/libcikd_asrBc_linux.a
# LIBS += -lcikd_asrBc_linux
# endif

endif


$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJS) $(USER_OBJS) build/source_file.mk $(LIB_FILES) ../src/ci130x.lds
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C++ Linker'
	$(CC_PREFIX)$(LD) $(LD_FLAGS) -o "$(BUILD_DIR)/$(PROJECT_NAME).elf" $(OBJS) $(USER_OBJS) $(LIBS) 
	@echo 'Finished building target: $@'
	@echo ' '

$(SECONDARY_FLASH): $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	@mkdir -p ../firmware/user_code && $(CC_PREFIX)$(OC) -O binary "$(BUILD_DIR)/$(PROJECT_NAME).elf"  $(SECONDARY_FLASH)
	@cp $(SDK_PATH)/libs/libfbin.a ../firmware/user_code/[1]code.bin
	@$(SDK_PATH)/tools/ci-tool-kit merge user-file -i ../firmware/user_code > /dev/null
	@rm ../firmware/user_code/[1]code.bin
	@echo 'Finished building: $@'
	@echo ' '

$(BUILD_DIR)/$(PROJECT_NAME).lst: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	$(CC_PREFIX)$(OD) --demangle --disassemble --reloc --wide "$(BUILD_DIR)/$(PROJECT_NAME).elf" > "$(BUILD_DIR)/$(PROJECT_NAME).lst"
	$(CC_PREFIX)$(SIZE) --format=berkeley "$(BUILD_DIR)/$(PROJECT_NAME).elf"
	@echo 'Finished building: $@'
	@echo ' '



secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST)


include $(ROOT_DIR)/utils/common_tail.mk

