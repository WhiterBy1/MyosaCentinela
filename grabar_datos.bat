@echo off
REM Arranca la grabacion de datos, por WiFi (20Hz, mejor para caidas) o
REM por LoRa/gateway (mas lento ~4Hz, pero no depende de ninguna red
REM WiFi). Elige segun la sesion. Doble clic para usar.

cd /d "%~dp0"

echo ============================================
echo   Grabador de datos MyosaCentinela
echo ============================================
echo.
echo Que canal quieres usar para esta sesion?
echo   [1] WiFi  (20Hz  - mejor para CAIDAS, necesita que el ESP32
echo             este en la misma red WiFi que esta PC)
echo   [2] LoRa  (~4Hz  - no depende de ninguna red, sirve para
echo             sesiones largas de actividad normal)
echo.
set /p MODO="Elige 1 o 2: "

echo.
set /p LABEL="Nombre de esta sesion (ej. caida_prueba_1, sesion_normal_1): "
if "%LABEL%"=="" set LABEL=sesion

echo.
echo ANTES DE CONTINUAR:
echo   - Desconecta el cable USB del ESP32
echo   - Conecta la bateria del ESP32 y espera a que arranque
if "%MODO%"=="2" (
    echo   - Asegurate que el gateway HT-M7603 este conectado por Ethernet
) else (
    echo   - El ESP32 se conecta solo a WHITERBY o al hotspot Sensorix
)
echo.
echo Abriendo la consola del grabador en otra ventana...
echo   - Esa ventana debe tener el foco para que F/G/Q funcionen
echo   - El dashboard con botones se abre solo en el navegador
echo.

if "%MODO%"=="2" (
    start "Grabador MyosaCentinela" cmd /k python tools\data_recorder.py --lora-listen 1700 --label "%LABEL%"
) else (
    start "Grabador MyosaCentinela" cmd /k python tools\data_recorder.py --udp-listen 5005 --label "%LABEL%"
)

timeout /t 2 /nobreak >nul
start "" http://localhost:8766

echo Listo. Revisa la otra ventana y el navegador.
echo Esta ventana ya puede cerrarse.
pause
