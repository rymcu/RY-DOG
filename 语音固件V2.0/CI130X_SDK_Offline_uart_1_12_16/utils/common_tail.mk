
ifeq ($(OS),Windows_NT)
LUA := $(ROOT_DIR)/tools/build-tools/bin/lua.exe
else
LUA := $(ROOT_DIR)/tools/build-tools/bin/lua
endif

%.o : %.c
	$(CC_PREFIX)$(CC) $(C_FLAGS) -c -o "$@" "$<"

%.o : %.S
	$(CC_PREFIX)$(AS) $(S_FLAGS) -c -o "$@" "$<"



build/source_file.mk: source_file.prj
	# sh $(ROOT_DIR)/tools/generate_makefile.sh
	$(LUA) $(ROOT_DIR)/utils/generate_makefile.lua source_file.prj $(ROOT_DIR)

include $(ROOT_DIR)/utils/common_firmware.mk


clean:
	-$(RM) -rf build/*
	-@echo ' '

.PHONY: all clean dependents
