@echo off
setlocal enabledelayedexpansion

set SDK_PATH=..\..\..

for /f "tokens=1-3" %%i in (../project_file/makefile) do (
	if "%%i" equ "SDK_PATH" (
		set SDK_PATH=%%k
	)
)
set SDK_PATH=!SDK_PATH:/=\!
set TOOLS_PATH=!SDK_PATH!\tools

del /f /s /q  user_file\[60000]*.xls.bin


echo make asr.bin
%TOOLS_PATH%\ci-tool-kit.exe merge asr-file -i asr

echo make user_code.bin
copy %TOOLS_PATH%\..\libs\libfbin.a user_code\[1]code.bin
%TOOLS_PATH%\ci-tool-kit.exe merge user-file -i user_code
del user_code\[1]code.bin


echo make dnn
%TOOLS_PATH%\ci-tool-kit.exe merge nn-file  -i dnn -a asr

echo make user_file.bin
cd user_file\cmd_info
call cmd_info.bat
cd ..\..
copy user_file\cmd_info\*.bin user_file\
%TOOLS_PATH%\ci-tool-kit.exe merge user-file -i user_file

@REM exit

echo make voice.bin
cd voice
set PATH=..\%TOOLS_PATH%;%PATH%
::exit /b

if exist "adpcm" (
    rd /s /q adpcm >nul
)
if exist "mp3" (
    rd /s /q mp3 >nul
)
if exist "flac" (
    rd /s /q flac >nul
)
if exist "pcm" (
    rd /s /q pcm >nul
)

echo 请选择播报音音频格式:
@REM echo 1. adpcm (默认格式,压缩比4:1,音质一般,解码器小,解码运算量小)
@REM echo 2. pcm   (无压缩,占用flash空间较大,音质好,无解码器,无解码运算)
@REM echo 3. flac  (压缩比大约3:1,音质好,解码器小,解码运算量略大于adpcm)
@REM echo 4. mp3   (压缩比大约10:1,音质较好,解码器较大,解码运算量大)

 
set fmt=4
@REM set /p fmt=default=4:

echo %fmt%

if %fmt% == 1 (
	if not exist "adpcm" (
		mkdir "adpcm"
	)
	for /d %%i in (*) do (
		echo %%i | findstr "\[.*\].*" > nul && call:to_adpcm adpcm %%i
	)
	..\%TOOLS_PATH%\group_merge.exe -i adpcm -o ..\voice
) else if %fmt% equ 4 (
	if not exist "mp3" (
		mkdir "mp3"
	)
	@REM for /d %%i in (*) do (
	@REM 	echo %%i | findstr "\[.*\].*" > nul && call:to_mp3 mp3 %%i
	@REM )
	call:to_mp3 mp3 src
	@REM ..\%TOOLS_PATH%\group_merge.exe -i mp3 -o ..\voice
	..\%TOOLS_PATH%\ci-tool-kit ID3-editor --input-dir mp3
	..\%TOOLS_PATH%\ci-tool-kit merge user-file -i mp3 -o mp3
	move mp3\mp3.bin .\voice.bin
) else if %fmt% equ 3 (
	if not exist "flac" (
		mkdir "flac"
	)
	for /d %%i in (*) do (
		echo %%i | findstr "\[.*\].*" > nul && call:to_flac flac %%i
	)
	..\%TOOLS_PATH%\group_merge.exe -i flac -o ..\voice
) else if %fmt% equ 2 (
	if not exist "pcm" (
		mkdir "pcm"
	)
	for /d %%i in (*) do (
		echo %%i | findstr "\[.*\].*" > nul && call:to_pcm pcm %%i
	)
	..\%TOOLS_PATH%\group_merge.exe -i pcm -o ..\voice
) else (
	echo error: 不支持的音频格式
)

cd ..

@echo on

exit /b


:to_pcm
echo %1\%2
if not exist %1\%2 (
	mkdir %1\%2
)
for %%j in (%2\*.wav) do (
	echo %%j
	..\%TOOLS_PATH%\wav_to_adpcm -f wav -i "%%j" -o "%1\%%j"
)
goto:eof

:to_adpcm
echo %1\%2
if not exist %1\%2 (
	mkdir %1\%2
)
for %%j in (%2\*.wav) do (
	echo %%j
	..\%TOOLS_PATH%\wav_to_adpcm -i "%%j" -o "%1\%%j"
)
goto:eof


:to_mp3
@REM echo %1\%2
@REM if not exist %1\%2 (
@REM 	mkdir %1\%2
@REM )
for %%j in (%2\*) do (
	set ttt=%%j
	set output_name=%1\!ttt:~4,-4!.mp3
	echo !output_name!
    set num=!ttt:~5,5!
    if !num! == 65535 (
        echo !num!
        set cfg_in=%%j
        set cfg_out=%1\!ttt:~4,-4!.txt
    )
	@REM ..\!TOOLS_PATH!\wav_to_mp3_flac.exe -f mp3 -i "!ttt!" -o "!output_name!"
	@REM ..\!TOOLS_PATH!\lame --silent --cbr -b24 -t --resample 16000 --out-file "!output_name!" "!ttt!"  
	..\!TOOLS_PATH!\lame --silent --cbr -b24 -t --resample 16000 "!ttt!" "!output_name!"
	@REM ..\!TOOLS_PATH!\ci-tool-kit ID3-editor -i !output_name!
)
if exist !cfg_in! ( 
    copy "!cfg_in!" "!cfg_out!"
)
goto:eof


:to_flac
echo %1\%2
if not exist %1\%2 (
	mkdir %1\%2
)
for %%j in (%2\*.wav) do (
	set ttt=%%j
	set output_name=%1\!ttt:~0,-4!.flac
	echo !output_name!
	..\!TOOLS_PATH!\wav_to_mp3_flac.exe -f flac -r 8000 -i "!ttt!" -o "!output_name!"
)
goto:eof




@echo on
