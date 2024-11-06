

case $CHIP_NAME in
    CI1301) ROM_SIZE=$((1*1024*1024))
    ;;
    CI1302) ROM_SIZE=$((2*1024*1024))
    ;;
    CI1303) ROM_SIZE=$((4*1024*1024))
    ;;
    CI1306) ROM_SIZE=$((4*1024*1024))
    ;;
    *)
    echo -e "\033[31mERROR: Please set environment variable: CHIP_NAME\033[0m"
    exit
    ;;
esac

ASR_SIZE=`ls -l "../firmware/asr/asr.bin" | awk '{print $5}'`
CODE_SIZE=`ls -l "../firmware/user_code/user_code.bin" | awk '{print $5}'`
DNN_SIZE=`ls -l "../firmware/dnn/dnn.bin" | awk '{print $5}'`
VOICE_SIZE=`ls -l "../firmware/voice/voice.bin" | awk '{print $5}'`
USER_FILE_SIZE=`ls -l "../firmware/user_file/user_file.bin" | awk '{print $5}'`

ASR_SIZE=$((($ASR_SIZE+4095)/4096*4096))
CODE_SIZE=$((($CODE_SIZE+4095)/4096*4096))
DNN_SIZE=$((($DNN_SIZE+4095)/4096*4096))
VOICE_SIZE=$((($VOICE_SIZE+4095)/4096*4096))
USER_FILE_SIZE=$((($USER_FILE_SIZE+4095)/4096*4096))

echo ASR SIZE:$ASR_SIZE
echo CODE SIZE:$CODE_SIZE
echo NN SIZE:$DNN_SIZE
echo VOICE SIZE:$VOICE_SIZE
echo USER FILE SIZE:$USER_FILE_SIZE
../../../tools/ci-tool-kit mf -f v2\
    --user-code ../firmware/user_code/user_code.bin --user-code-size $CODE_SIZE\
    --asr-file ../firmware/asr/asr.bin --asr-size $ASR_SIZE\
    --nn-file ../firmware/dnn/dnn.bin --nn-size $DNN_SIZE\
    --voice-file ../firmware/voice/voice.bin --voice-size $VOICE_SIZE\
    --user-file ../firmware/user_file/user_file.bin --user-file-size $USER_FILE_SIZE\
    --rom-size $ROM_SIZE\
    --chip-name CI130X\
    --output-path ../firmware

echo make firmware finished!
