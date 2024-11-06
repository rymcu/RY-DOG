


C_FLAGS += -march=rv32imafc -mabi=ilp32f -mcmodel=medlow -msmall-data-limit=8 -msave-restore -mfdiv -fmessage-length=0 -fshort-enums $(O_OPTION)
C_FLAGS += -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-delete-null-pointer-checks -fno-unroll-loops -std=gnu11 -MMD -MP
C_FLAGS += $(LTO_OPTION)

S_FLAGS += -march=rv32imafc -mabi=ilp32f -mcmodel=medlow -msmall-data-limit=8 -msave-restore -mfdiv -fmessage-length=0 $(O_OPTION)
S_FLAGS += -fsigned-char -ffunction-sections -fdata-sections -fno-common -fno-delete-null-pointer-checks -fno-unroll-loops  -x assembler-with-cpp -MMD -MP

ifneq ($(RELEASE), TRUE)
C_FLAGS += -g
S_FLAGS += -g
endif
