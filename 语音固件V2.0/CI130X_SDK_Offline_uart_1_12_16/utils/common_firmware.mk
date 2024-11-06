
%.mp3 : %.wav
	$(ROOT_DIR)/tools/lame --silent --cbr -b24 -t --resample 16000 "$<" "$@"
	@cp "$@" $(FIRMWARE_PATH)/voice/mp3/


FIRMWARE_PATH := ../firmware
CI_TOOL_KIT := $(SDK_PATH)/tools/ci-tool-kit


WAV_FILES := $(wildcard $(FIRMWARE_PATH)/voice/src/*.wav)
MP3_FILES := $(patsubst %.wav, %.mp3, $(WAV_FILES))
MP3_DIR := $(FIRMWARE_PATH)/voice/mp3

$(MP3_DIR):
ifeq ($(shell if [ -d "../firmware/voice/mp3/" ]; then echo "exist"; else echo "noexist"; fi),noexist)
	mkdir "../firmware/voice/mp3/"
endif

voice: $(MP3_DIR) $(MP3_FILES) 
	@echo make voice
	$(SDK_PATH)/tools/ci-tool-kit ID3-editor --input-dir $(MP3_DIR)
	@$(SDK_PATH)/tools/ci-tool-kit merge user-file -i $(MP3_DIR) -o $(MP3_DIR)
	@$(CMDBOX) mv $(FIRMWARE_PATH)/voice/mp3/mp3.bin $(FIRMWARE_PATH)/voice/voice.bin



user_file:
	@echo make user_file
	@rm -f $(FIRMWARE_PATH)/user_file/[60000]*.xls.bin
	@rm -f $(FIRMWARE_PATH)/user_file/cmd_info/*.bin
	@find $(FIRMWARE_PATH)/user_file/cmd_info/ -name "*[60000]*" | xargs $(CI_TOOL_KIT) cmd-info -V2 -i
	@cp $(FIRMWARE_PATH)/user_file/cmd_info/*.bin $(FIRMWARE_PATH)/user_file/
	@$(CI_TOOL_KIT) merge user-file -i $(FIRMWARE_PATH)/user_file

nn:
	@echo make nn
	@$(CI_TOOL_KIT) merge nn-file -i $(FIRMWARE_PATH)/dnn/ -a $(FIRMWARE_PATH)/asr

user_code: $(FIRMWARE_PATH)/user_code/[0]code.bin
	@echo make user_code
	@cp $(SDK_PATH)/libs/libfbin.a $(FIRMWARE_PATH)/user_code/[1]code.bin
	@$(CI_TOOL_KIT) merge user-file -i $(FIRMWARE_PATH)/user_code
	@rm $(FIRMWARE_PATH)/user_code/[1]code.bin


asr: $(wildcard $(FIRMWARE_PATH)/asr/*.dat)
	@echo start make firmware
	@echo make asr.bin
	@$(CI_TOOL_KIT) merge asr-file -i $(FIRMWARE_PATH)/asr	


# 此命令在使用时需要设置环境变量：CHIP_NAME
firmware: asr user_code nn user_file voice 
	@sh $(ROOT_DIR)/utils/firmware_pack.sh

clean-firmware:
	@rm $(FIRMWARE_PATH)/*.bin
	@rm $(FIRMWARE_PATH)/asr/asr.bin
	@rm $(FIRMWARE_PATH)/dnn/dnn.bin
	@rm $(FIRMWARE_PATH)/user_code/user_code.bin
	@rm $(FIRMWARE_PATH)/voice/voice.bin
	@rm -rf $(FIRMWARE_PATH)/voice/mp3
	@rm -rf $(FIRMWARE_PATH)/voice/src/*.mp3
	@rm $(FIRMWARE_PATH)/user_file/user_file.bin
	@rm $(FIRMWARE_PATH)/user_file/cmd_info/*.bin

