

ifndef PROJECT_NAME
# PROJECT_NAME := $(notdir $(shell pwd))
PROJECT_NAME := $(BUILD_DIR)/program
endif

ifeq ($(OS),Windows_NT)
CMDBOX := busybox
LIBS_PATH := libs
LUA := $(ROOT_DIR)/tools/build-tools/bin/lua.exe
else
CMDBOX :=
LIBS_PATH := libs/linux
LUA := $(ROOT_DIR)/tools/build-tools/bin/lua
endif

USER_OBJS := 
C_SRCS :=

# CC_PREFIX := riscv-none-embed-
CC_PREFIX := riscv-nuclei-elf-
CC = gcc
AS = gcc
AR = gcc-ar
LD = g++
OD = objdump
OC = objcopy
SIZE = size
OBJS := 

C_FLAGS :=
S_FLAGS :=

LTO_OPTION :=
O_OPTION := -Os







