@echo off
REM Arranca el servidor del dashboard LoRa (UDP del gateway + web en vivo)
REM y abre el navegador. Doble clic para usar, o ejecutar desde cmd.

cd /d "%~dp0"

echo Matando instancias previas de python (si hay)...
taskkill /F /IM python.exe >nul 2>&1

echo Abriendo el dashboard en el navegador...
start "" http://localhost:8765

echo Arrancando servidor (Ctrl+C en esta ventana para detenerlo)...
python tools\lora_dashboard_server.py

pause
