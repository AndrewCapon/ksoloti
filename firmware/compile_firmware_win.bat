@REM @ECHO OFF

call :setfirmware "%axoloti_firmware%"
call :sethome "%axoloti_home%"
call :setrelease "%axoloti_release%"

set PATH=%axoloti_runtime%\platform_win\bin

cd %axoloti_firmware%
make BOARDDEF=%1 -f Makefile.patch.mk clean

echo.&&echo %1%

IF "%1%"=="BOARD_KSOLOTI_CORE" (
	echo.&&echo KSOLOTI
	set NAME=ksoloti
)

IF "%1%"=="BOARD_AXOLOTI_CORE" (
	echo.&&echo AXOLOTI
	set NAME=axoloti
)

echo.&&echo Name %NAME%

set FLASHER_PROJECT=%NAME%_flasher
set MOUNTER_PROJECT=%NAME%_mounter

echo.&&echo Compiling firmware flasher... %1 
cd flasher
if not exist ".dep" mkdir .dep
if not exist "flasher_build/%FLASHER_PROJECT%/lst" mkdir "flasher_build/%FLASHER_PROJECT%/lst"
if not exist "flasher_build/%FLASHER_PROJECT%/obj" mkdir "flasher_build/%FLASHER_PROJECT%/obj"
rem FWOPTIONDEF currently not used in flasher
make -j16 BOARDDEF=%1
IF %ERRORLEVEL% NEQ 0 (
	exit /b 1
)
cp "flasher_build/%FLASHER_PROJECT%/%FLASHER_PROJECT%.*" flasher_build/
cd ..

echo.&&echo Compiling firmware mounter... %1
cd mounter
if not exist ".dep" mkdir .dep
if not exist "mounter_build/%MOUNTER_PROJECT%/lst" mkdir "mounter_build/%MOUNTER_PROJECT%/lst"
if not exist "mounter_build/%MOUNTER_PROJECT%/obj" mkdir "mounter_build/%MOUNTER_PROJECT%/obj"
rem FWOPTIONDEF currently not used in mounter
make -j16 BOARDDEF=%1
IF %ERRORLEVEL% NEQ 0 (
	exit /b 1
)
cp "mounter_build/%MOUNTER_PROJECT%/%MOUNTER_PROJECT%.*" mounter_build/
cd ..

echo.&&echo Compiling firmware... %1
set BUILDDIR=build/%NAME%/normal
if not exist ".dep" mkdir .dep
if not exist "%BUILDDIR%\lst" mkdir "%BUILDDIR%\lst"
if not exist "%BUILDDIR%\obj" mkdir "%BUILDDIR%\obj"
make -j16 BOARDDEF=%1
IF %ERRORLEVEL% NEQ 0 (
	exit /b 1
)
cp "%BUILDDIR%/%NAME%.*" build

echo.&&echo Compiling firmware... %1 FW_SPILINK
set BUILDDIR=build/%NAME%/spilink
if not exist ".dep" mkdir .dep
if not exist "%BUILDDIR%\lst" mkdir "%BUILDDIR%\lst"
if not exist "%BUILDDIR%\obj" mkdir "%BUILDDIR%\obj"
make -j16 BOARDDEF=%1 FWOPTIONDEF=FW_SPILINK
IF %ERRORLEVEL% NEQ 0 (
	exit /b 1
)
cp "%BUILDDIR%/%NAME%_spilink.*" build

echo.&&echo Compiling firmware... %1 USB_AUDIO
set BUILDDIR=build/%NAME%/usbaudio
if not exist ".dep" mkdir .dep
if not exist "%BUILDDIR%\lst" mkdir "%BUILDDIR%\lst"
if not exist "%BUILDDIR%\obj" mkdir "%BUILDDIR%\obj"
make -j16 BOARDDEF=%1 FWOPTIONDEF=FW_USBAUDIO
IF %ERRORLEVEL% NEQ 0 (
	exit /b 1
)
cp "%BUILDDIR%/%NAME%_usbaudio.*" build

goto :eof

rem --- path shortening

:setfirmware
set axoloti_firmware=%~s1
goto :eof

:sethome
set axoloti_home=%~s1
goto :eof

:setrelease
set axoloti_release=%~s1
goto :eof
