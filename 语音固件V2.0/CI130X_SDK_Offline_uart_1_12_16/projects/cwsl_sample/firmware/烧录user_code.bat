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

echo project offline_asr_sample

@REM copy ..\..\bnpu_core\firmware\user_code\[1]code.bin  .\user_code

echo make user.bin
copy %TOOLS_PATH%\..\libs\libfbin.a user_code\[1]code.bin
%TOOLS_PATH%\ci-tool-kit.exe merge user-file -i user_code
del user_code\[1]code.bin


%TOOLS_PATH%\code_program.exe user_code\user_code.bin %1




