# Guía rápida — probar MyosaCentinela

Este documento es para quien va a probar el sistema en su propia PC. El ESP32 ya viene con el firmware más reciente flasheado — no hay que tocarle nada por ahí. Todo lo que necesitas está en esta carpeta (`tools/`, `grabar_datos.bat` y esta guía) — no hace falta nada del proyecto de ESP-IDF (compilador, etc.).

## Qué vas a poder hacer

1. **Ver el dashboard en vivo** (orientación, temperatura, señal).
2. **Grabar datos** (sesiones normales o pruebas de caída) con segmentos etiquetados — genera los CSV que luego se analizan.

Hay **dos formas de conectar** el ESP32, y puedes usar la que te sea más fácil:

| Canal | Velocidad | Requisito | Cuándo usarlo |
|---|---|---|---|
| **WiFi** | 20Hz (mejor calidad) | El ESP32 debe unirse a una red que YA tiene programada (ver abajo) | Sesiones de **caída** — necesitas la mejor resolución |
| **LoRa** (gateway por Ethernet) | ~4Hz | Ninguno — no depende de ninguna red WiFi | Sesiones largas de actividad normal, o si el WiFi no aplica |

---

## Opción A — Por WiFi (recomendada para caídas)

El ESP32 ya tiene programadas estas dos redes (intenta conectarse a la primera que encuentre disponible):
- Red de casa del dueño original (`WHITERBY`) — no te va a servir a menos que estés en esa casa.
- Un hotspot llamado **`Sensorix`** con contraseña **`Jose1234`**.

**Para que te funcione:** crea un hotspot en tu celular con exactamente ese nombre y contraseña (`Sensorix` / `Jose1234`), y conecta tu PC al mismo hotspot. El ESP32 se unirá solo — no hace falta configurar ninguna IP, el sistema la calcula automáticamente sin importar qué IP te asigne tu celular.

### Instala Python 3
Si no lo tienes: [python.org](https://www.python.org/downloads/). No hace falta ninguna librería adicional.

### Corre el grabador
Doble clic en **`grabar_datos.bat`**, elige la opción **1 (WiFi)**, pon un nombre de sesión, y conecta la batería del ESP32.

O manualmente:
```powershell
python tools\data_recorder.py --udp-listen 5005 --label mi_prueba_1
```

Abre: **http://localhost:8766**

---

## Opción B — Por LoRa / gateway (no depende de ninguna WiFi)

### Paso 1 — Conecta el gateway
Cable Ethernet directo del gateway HT-M7603 a tu PC (sin router ni switch de por medio).

### Paso 2 — Ponle una IP fija a tu adaptador Ethernet
El gateway espera un servidor en `192.168.9.50`. Abre PowerShell **como Administrador**:
```powershell
Get-NetAdapter   # confirma el nombre de tu adaptador Ethernet (puede no llamarse "Ethernet")
Set-NetIPInterface -InterfaceAlias "Ethernet" -AddressFamily IPv4 -Dhcp Disabled
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.9.50 -PrefixLength 24
```

### Paso 3 — Verifica
```powershell
ping 192.168.9.20
```
Si responde, ya estás conectado.

### Corre el grabador
Doble clic en **`grabar_datos.bat`**, elige la opción **2 (LoRa)**.

O manualmente:
```powershell
python tools\data_recorder.py --lora-listen 1700 --label mi_prueba_1
```
(Para solo ver en vivo sin grabar: `python tools\lora_dashboard_server.py` → http://localhost:8765)

---

## Usando el dashboard del grabador (http://localhost:8766)

Vas a ver: magnitud de aceleración en vivo (con líneas de umbral), orientación, y — si viene por LoRa — señal (RSSI/SNR) y a qué red WiFi está conectado el ESP32 (solo diagnóstico).

**Segmentos:** al presionar **CAÍDA** o **NORMAL**, ESE segmento queda activo (se etiqueta en cada fila del CSV) hasta que presiones el otro botón — no es un punto instantáneo, es un rango continuo. Mantenlos **cortos**: unos segundos justo antes/durante/después del evento, no minutos. El dashboard muestra cuánto tiempo llevas en el segmento activo y se pone en amarillo pasados los 15s como recordatorio.

### Dónde quedan los datos
En la carpeta `recordings/`, una subcarpeta por sesión con:
- `telemetria.csv` — señal completa (acelerómetro, giroscopio, orientación) con su segmento etiquetado en cada fila
- `eventos.csv` — cambios de segmento, gestos, y candidatos automáticos de caída (picos de aceleración detectados solos)
- `meta.json` — resumen de la sesión

---

## Notas técnicas (por si algo falla)

- El ESP32 transmite LoRa en 903.9 MHz / SF7 / BW125 — ya configurado en el gateway, no lo cambies.
- Si el gateway no responde al ping, revisa el cable y que tenga corriente (`Get-NetAdapter` debe mostrar `Status: Up`).
- Los dos dashboards (`8765` solo-vista y `8766` grabador) pueden correr al mismo tiempo, en puertos distintos.
- Si por WiFi no te conecta, confirma que el hotspot se llame EXACTO `Sensorix` con clave `Jose1234` (mayúsculas/minúsculas incluidas).
