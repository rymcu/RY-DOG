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
start !TOOLS_PATH!\PACK_UPDATE_TOOL.exe

@echo on
