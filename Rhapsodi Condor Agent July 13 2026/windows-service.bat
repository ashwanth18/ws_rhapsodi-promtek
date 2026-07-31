@echo off
:: =====================================
:: Auto-elevate to Administrator
:: =====================================
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo This script requires administrator privileges. Restarting as admin...
    powershell -Command "Start-Process '%~f0' -Verb runAs"
    exit /b
)

:: === CONFIGURATION ===
:: Service parameters
set SERVICE_NAME=PromtekCondorRhapsodiAgent
set DISPLAY_NAME="Promtek Condor Rhapsodi Agent"
set DESCRIPTION="Promtek Condor Rhapsodi Agent used to receive data from Promtek Condor"

:: Directory paths
set ROOT_DIR=c:\promtek
set APP_DIR=%ROOT_DIR%\promtek-condor-rRhapsodi-agent
set BACKUP_DIR=%APP_DIR%.old
set VERSIONS_DIR=%APP_DIR%-versions
set BINARY_PATH=%APP_DIR%\Promtek.Condor.Rhapsodi.Agent.exe

:: === MAIN MENU ===
:menu
cls
echo.
echo 1. Install service
echo 2. Uninstall service
echo 3. Start service
echo 4. Stop service
echo 5. Check service status
echo 6. Show current installed version
echo 7. Update to a different version
echo 8. Exit

echo.
set /p choice="Choose an option (1-8): "

if "%choice%"=="1" goto install
if "%choice%"=="2" goto uninstall
if "%choice%"=="3" goto startservice
if "%choice%"=="4" goto stopservice
if "%choice%"=="5" goto status
if "%choice%"=="6" goto showversion
if "%choice%"=="7" goto update
if "%choice%"=="8" goto end
goto menu

:: === INSTALL SERVICE ===
:install
echo Installing service %SERVICE_NAME%...
sc create %SERVICE_NAME% binPath= "\"%BINARY_PATH%\" --service" DisplayName= %DISPLAY_NAME% start= auto
if errorlevel 1 (
    echo Failed to install service. Make sure you are running as administrator.
) else (
    echo Service installed successfully.
    sc description %SERVICE_NAME% %DESCRIPTION%
    sc start %SERVICE_NAME%
)
pause
goto menu

:: === UNINSTALL SERVICE ===
:uninstall
echo Uninstalling service %SERVICE_NAME%...
sc stop %SERVICE_NAME%
sc delete %SERVICE_NAME%
echo Service %SERVICE_NAME% uninstalled.
pause
goto menu

:: === CHECK STATUS ===
:status
sc query %SERVICE_NAME%
pause
goto menu

:: === START SERVICE ===
:startservice
echo Starting service %SERVICE_NAME%...
sc start %SERVICE_NAME%
pause
goto menu

:: === STOP SERVICE ===
:stopservice
echo Stopping service %SERVICE_NAME%...
sc stop %SERVICE_NAME%
pause
goto menu

:: === SHOW VERSION ===
:showversion
echo Getting version of %BINARY_PATH%...
powershell -Command "(Get-Item '%BINARY_PATH%').VersionInfo.FileVersion"
pause
goto menu

:: === UPDATE APP ===
:update
echo Available versions:
setlocal enabledelayedexpansion
set i=0
for %%f in (%VERSIONS_DIR%\*.zip) do (
    set /a i+=1
    set "ver[!i!]=%%~nxf"
    echo !i!. %%~nxf
)
if "!i!"=="0" (
    echo No zip files found in %VERSIONS_DIR%.
    pause
    endlocal
    goto menu
)

echo.
set /p sel="Enter the number of the version to install: "
if not defined ver[%sel%] (
    echo Invalid selection.
    pause
    endlocal
    goto menu
)

set "zipFile=%VERSIONS_DIR%\!ver[%sel%]!"
echo Selected: !zipFile!

echo Stopping service...
sc stop %SERVICE_NAME% >nul

:: Wait for the service to stop
set maxWait=60
set waitCount=0

:waitloop
sc query %SERVICE_NAME% | find "STATE" | find /i "STOPPED" >nul
if %errorlevel%==0 (
    echo Service stopped.
) else (
    set /a waitCount+=1
    if %waitCount% geq %maxWait% (
        echo Service did not stop within %maxWait% seconds. Aborting update.
        pause
        endlocal
        goto menu
    )
    timeout /t 1 >nul
    goto waitloop
)

echo Backing up current app...

:: Delete existing backup folder, if any
if exist "%BACKUP_DIR%" (
    echo Removing old backup folder...
    rmdir /s /q "%BACKUP_DIR%" || (
        echo Failed to delete old backup folder!
        pause
        endlocal
        goto menu
    )
)

:: Extract just the directory names (no full paths)
for %%I in ("%APP_DIR%") do set "APP_DIR_NAME=%%~nxI"
for %%I in ("%BACKUP_DIR%") do set "BACKUP_DIR_NAME=%%~nxI"

echo APP_DIR_NAME = %APP_DIR_NAME%
echo BACKUP_DIR_NAME = %BACKUP_DIR_NAME%

pushd "%ROOT_DIR%" || (
    echo Failed to navigate to root directory: %ROOT_DIR%
    pause
    endlocal
    goto menu
)

:: Rename the app folder
echo Renaming "%APP_DIR_NAME%" to "%BACKUP_DIR_NAME%"...
rename "%APP_DIR_NAME%" "%BACKUP_DIR_NAME%" || (
    echo Rename failed. Check if service is still locking files.
    popd
    pause
    endlocal
    goto menu
)

popd

echo Extracting new version...
powershell -Command "Expand-Archive -Path '!zipFile!' -DestinationPath '%APP_DIR%' -Force"

echo Starting service...
sc start %SERVICE_NAME%

endlocal
echo Update complete.
pause
goto menu

:end
echo Exiting.
pause
exit
