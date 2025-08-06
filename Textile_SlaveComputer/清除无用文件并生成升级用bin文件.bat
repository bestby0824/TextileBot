echo off
::echo %date%_%time%
set date_num=%date:~0,10%
set date_num=%date_num:-=%
set date_num=%date_num:/=%
set time_num=%time:~0,8%
set time_num=%time_num::=%
if "%time_num:~0,1%"==" " set "time_num=0%time_num:~1%"
echo on

@echo Start
del *.bin

cd Service\Inc
@echo off &setlocal enabledelayedexpansion
findstr "SoftVer_A" DataBaseProcess.h >>Transfer.txt
for /f "tokens=3 delims= " %%a in ( Transfer.txt ) do (
::echo %%a
set str_A=%%a
echo !str_A!
)
del Transfer.txt
findstr "SoftVer_B" DataBaseProcess.h >>Transfer.txt
for /f "tokens=3 delims= " %%b in ( Transfer.txt ) do (
::echo %%b
set str_B=%%b
echo !str_B!
)
del Transfer.txt
findstr "SoftVer_C" DataBaseProcess.h >>Transfer.txt
for /f "tokens=3 delims= " %%c in ( Transfer.txt ) do (
::echo %%c
set str_C=%%c
echo !str_C!
)
del Transfer.txt
cd ..
cd ..

cd Project\bin
echo f | xcopy TextileSlaveBD.bin %~dp0\Textile-71" "701CP_UpdateApp_%str_A%_%str_B%_%str_C%.bin /S /F /R /Y /E

cd ..\Objects
@echo -----------------------Copy  TextileSlaveBD.hex Finished!--------------------------
del *.d
del *.o
del *.axf
del *.htm
del *.hex
del *.lnp
del *.sct
del *.dep
del *.iex
del *.crf
@echo -----------------------Clear \Project\Objects Finished!--------------------------

cd ..\Listings
del *.map

cd ..\bin
del *.bin

@echo -----------------------Clear \Project\Listings Finished!-----------------------------
pause