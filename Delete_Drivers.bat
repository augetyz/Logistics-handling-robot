@echo off
setlocal enabledelayedexpansion

REM 设置要遍历的根目录
set "rootDir=E:\desktop\Logistics-handling"
REM 遍历根目录下的所有子文件夹
for /d %%D in ("%rootDir%\*") do (
    set "driversDir=%%D\Drivers"
    if exist "!driversDir!\" (
        echo Deleting contents of: !driversDir!
        del /q /s "!driversDir!\*"
    )
)

echo Done.
pause