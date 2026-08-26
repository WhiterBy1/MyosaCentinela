"""
Grabador de datos para armar un dataset de caidas/actividad normal, de
cara a un futuro detector (TinyML o reglas) de caidas.

Se conecta por USB/UART0 (no por LoRa: ahi solo llega roll/pitch/yaw
derivados cada 250ms, sin el detalle rapido que necesita un detector de
caidas). Por USB llega el protocolo TLM: completo del firmware
(main/myosa_field_main.c) a ~20Hz, con acelerometro y giroscopio crudos.

Cada corrida genera UNA carpeta con:
  telemetria.csv   - una fila por cada TLM: recibido (señal continua)
  eventos.csv       - gestos del APDS9960 + marcas manuales (tecla) +
                       candidatos de caida detectados automaticamente
                       por umbral de magnitud de aceleracion
  meta.json         - etiqueta de la sesion, hora de inicio/fin, conteos

Uso:
  python tools/data_recorder.py --port COM15 --label sesion_normal_1
  python tools/data_recorder.py --port COM15 --label caida_prueba_1

Mientras graba (con la ventana de esta consola en foco):
  F  -> marca "CAIDA" en el instante actual (usalo justo cuando simules
        la caida)
  G  -> marca "NORMAL" (para delimitar tramos de actividad normal)
  Q  -> termina la grabacion y cierra los archivos

Ademas, sin que presiones nada: si la magnitud de aceleracion cruza un
umbral (impacto o caida libre), se guarda solo un marcador
"CANDIDATO_AUTO" para revisar despues - no descarta nada, es una ayuda
para no perder eventos si no alcanzas a presionar la tecla a tiempo.
"""
import argparse
import base64
import csv
import json
import math
import os
import re
import sys
import threading
import time
from datetime import datetime, timezone, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

try:
    import msvcrt  # Windows: tecla no bloqueante, sin librerias externas
    HAVE_MSVCRT = True
except ImportError:
    HAVE_MSVCRT = False

# Deteccion en vivo (opcional): si tools/fall_model.joblib existe (ver
# tools/train_fall_model.py) y estan instalados joblib/numpy, se carga y
# se evalua cada candidato automatico 3s despues de disparar - igual
# filosofia que el entrenamiento, no se juzga el golpe aislado. Si falta
# cualquiera de las dos cosas, el grabador sigue funcionando igual, solo
# sin la alerta del modelo.
try:
    import joblib
    from fall_features import extract_window_features, FEATURE_NAMES, POST_WINDOW_S
    HAVE_ML = True
except ImportError:
    HAVE_ML = False
    POST_WINDOW_S = 3.0

FALL_MODEL_PATH = os.path.join(os.path.dirname(__file__), "fall_model.joblib")
BUFFER_KEEP_S = POST_WINDOW_S + 2.0  # margen extra sobre la ventana que necesita el modelo

# Umbral de magnitud de aceleracion (en g) para marcar un candidato
# automatico. >1.8g ~ impacto; <0.3g ~ caida libre (ingravidez momentanea
# tipica de una caida real). Ajusta segun lo que veas en tus pruebas.
ACCEL_IMPACT_G = 1.8
ACCEL_FREEFALL_G = 0.3
AUTO_MARK_COOLDOWN_S = 1.0  # no repetir el mismo candidato varias veces seguidas

TLM_RE = re.compile(r"TLM:(\{.*\})")
EVT_RE = re.compile(r"EVT:(\{.*\})")

TELEMETRY_FIELDS = [
    "recv_iso", "t_ms", "seq", "dt_us", "label",
    "roll", "pitch", "yaw",
    "ax", "ay", "az", "gx", "gy", "gz",
    "accel_mag_g",
    "temp", "pres", "alt", "prox", "light", "batt",
]

EVENT_FIELDS = ["recv_iso", "t_ms", "type", "detail"]

# --- Dashboard web (opcional): HTTP + Server-Sent Events, solo libreria
# estandar. Corre en un hilo aparte del loop principal que lee el
# transporte (serie o TCP), asi que todo acceso al Recorder desde el
# hilo HTTP (el boton de marcar CAIDA/NORMAL) pasa por dash_lock. ---
DASHBOARD_HTTP_PORT = 8766
DASHBOARD_HTML_PATH = os.path.join(os.path.dirname(__file__), "recorder_dashboard.html")

dash_lock = threading.RLock()  # reentrante: handle_tlm() llama publish_dash() con el mismo lock ya tomado
dash_state = {"version": 0, "payload": None}
g_recorder = None  # asignado en main() una vez se crea el Recorder


def publish_dash(payload):
    with dash_lock:
        dash_state["version"] += 1
        dash_state["payload"] = payload


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(DASHBOARD_HTML_PATH, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last_version = 0
            try:
                while True:
                    with dash_lock:
                        v, payload = dash_state["version"], dash_state["payload"]
                    if v != last_version and payload is not None:
                        last_version = v
                        self.wfile.write(f"data: {json.dumps(payload)}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path.startswith("/mark"):
            qs = parse_qs(urlparse(self.path).query)
            kind = (qs.get("type", ["NORMAL"])[0]).upper()
            if kind not in ("CAIDA", "NORMAL"):
                kind = "NORMAL"
            with dash_lock:
                if g_recorder is not None:
                    g_recorder.mark_manual(kind)
            self.send_response(204)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


def start_dashboard_server():
    server = ThreadingHTTPServer(("0.0.0.0", DASHBOARD_HTTP_PORT), DashboardHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"Dashboard en http://localhost:{DASHBOARD_HTTP_PORT}\n")


def find_default_port():
    """Busca un adaptador serie tipico de ESP32 (CH340/CP210x/FTDI) sin
    depender de librerias extra - usa el listado que ya trae pyserial."""
    from serial.tools import list_ports
    candidates = []
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("ch340", "cp210", "ftdi", "usb-serial", "usb serial")):
            candidates.append(p.device)
    return candidates[0] if candidates else None


# El payload compacto que manda el firmware por LoRa (main/myosa_field_main.c,
# lora_build_payload) usa claves cortas para ahorrar airtime. OJO: "t" ahi
# significa TEMPERATURA (no timestamp como en el TLM: de UART) - por eso
# hace falta este mapeo explicito en vez de reusar las claves tal cual.
LORA_KEY_MAP = {
    "seq": "seq", "r": "roll", "p": "pitch", "y": "yaw",
    "ax": "ax", "ay": "ay", "az": "az",
    "gx": "gx", "gy": "gy", "gz": "gz",
    "t": "temp", "pr": "pres", "px": "prox", "lt": "light",
}


def parse_lora_payload(text):
    """'seq=3,r=5.9,...,net=WHITERBY' -> dict con los MISMOS nombres de
    campo que el protocolo TLM: de UART, para alimentar directo a
    Recorder.handle_tlm sin tocarlo."""
    out = {}
    for part in text.split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        if k == "net":
            out["net"] = v
            continue
        field = LORA_KEY_MAP.get(k)
        if field is None:
            continue
        try:
            out[field] = float(v) if ("." in v or (len(v) > 1 and v[0] == "-")) else int(v)
        except ValueError:
            out[field] = v
    return out


def open_lora_transport(args):
    """Escucha el packet forwarder Semtech UDP del gateway LoRa (mismo
    protocolo que tools/lora_dashboard_server.py) y lo convierte en
    lineas 'TLM:{...}' para que el resto del pipeline (Recorder,
    dashboard) no tenga que saber que el dato vino por LoRa y no por
    USB. No depende de ninguna red WiFi ni de conocer la IP de nadie -
    el gateway ya reenvia por Ethernet a quien este escuchando aqui."""
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.lora_listen))
    sock.settimeout(0.2)
    print(f"Escuchando gateway LoRa en UDP:{args.lora_listen} (packet forwarder Semtech)...")

    PKT_PUSH_DATA, PKT_PUSH_ACK, PKT_PULL_DATA, PKT_PULL_ACK = 0, 1, 2, 4

    def read_chunk():
        try:
            data, addr = sock.recvfrom(65535)
        except (socket.timeout, OSError):
            return b""
        if len(data) < 4:
            return b""
        version, token, pkt_type = data[0], data[1:3], data[3]

        if pkt_type == PKT_PUSH_DATA and len(data) > 12:
            try:
                payload = json.loads(data[12:])
            except json.JSONDecodeError:
                payload = {}
            out = b""
            for rxpk in payload.get("rxpk", []):
                try:
                    raw = base64.b64decode(rxpk.get("data", ""))
                    text = raw.decode("ascii")
                except (ValueError, UnicodeDecodeError):
                    continue
                obj = parse_lora_payload(text)
                obj["_rssi"] = rxpk.get("rssi")
                obj["_snr"] = rxpk.get("lsnr")
                out += ("TLM:" + json.dumps(obj) + "\n").encode("utf-8")
            sock.sendto(bytes([version]) + token + bytes([PKT_PUSH_ACK]), addr)
            return out
        elif pkt_type == PKT_PULL_DATA:
            sock.sendto(bytes([version]) + token + bytes([PKT_PULL_ACK]), addr)
            return b""
        return b""

    return read_chunk, sock.close


class Recorder:
    def __init__(self, label, out_dir):
        ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        safe_label = re.sub(r"[^a-zA-Z0-9_-]+", "_", label) or "sesion"
        self.session_dir = os.path.join(out_dir, f"{safe_label}_{ts}")
        os.makedirs(self.session_dir, exist_ok=True)

        self.label = label
        self.start_time = datetime.now(timezone.utc)

        self.tlm_path = os.path.join(self.session_dir, "telemetria.csv")
        self.evt_path = os.path.join(self.session_dir, "eventos.csv")
        self.tlm_file = open(self.tlm_path, "w", newline="", encoding="utf-8")
        self.evt_file = open(self.evt_path, "w", newline="", encoding="utf-8")
        self.tlm_writer = csv.DictWriter(self.tlm_file, fieldnames=TELEMETRY_FIELDS)
        self.evt_writer = csv.DictWriter(self.evt_file, fieldnames=EVENT_FIELDS)
        self.tlm_writer.writeheader()
        self.evt_writer.writeheader()

        self.n_tlm = 0
        self.n_evt = 0
        self.n_manual_marks = 0
        self.n_auto_candidates = 0
        self._last_auto_mark_at = 0.0

        # Etiquetado por SEGMENTOS: al presionar F/G, ese segmento queda
        # activo (y se escribe en cada fila de telemetria.csv) hasta que
        # presiones el otro boton - no es un punto instantaneo, es un
        # rango continuo. "SIN_MARCAR" son las filas antes del primer
        # boton que presiones.
        self.current_label = "SIN_MARCAR"
        self.label_counts = {}  # cuantas filas de telemetria cayeron en cada segmento

        # --- Deteccion en vivo con el modelo entrenado (opcional) ---
        self.fall_model = None
        if HAVE_ML and os.path.exists(FALL_MODEL_PATH):
            try:
                bundle = joblib.load(FALL_MODEL_PATH)
                self.fall_model = bundle["model"]
                print(f"Detector de caidas cargado ({bundle['n_examples']} ejemplos, "
                      f"{bundle['n_caidas']} caidas en el entrenamiento)")
            except Exception as e:
                print(f"No se pudo cargar el modelo de deteccion ({FALL_MODEL_PATH}): {e}")
        elif not HAVE_ML:
            print("Deteccion en vivo desactivada (falta joblib/numpy/pandas o fall_features.py)")
        else:
            print(f"Deteccion en vivo desactivada (no existe {FALL_MODEL_PATH} - corre train_fall_model.py)")

        self.buffer = []          # [(datetime, accel_mag_g, gyro_mag, roll, pitch), ...]
        self.pending_evals = []   # [{"trigger_dt": datetime, "trigger_t_ms": ...}]
        self.n_alerts = 0

    def log_event(self, t_ms, ev_type, detail=""):
        self.evt_writer.writerow({
            "recv_iso": datetime.now(timezone.utc).isoformat(),
            "t_ms": t_ms,
            "type": ev_type,
            "detail": detail,
        })
        self.evt_file.flush()
        self.n_evt += 1
        publish_dash({"_kind": "evt", "type": ev_type, "detail": detail})

    def handle_tlm(self, obj):
        ax, ay, az = obj.get("ax", 0.0), obj.get("ay", 0.0), obj.get("az", 0.0)
        gx, gy, gz = obj.get("gx", 0.0), obj.get("gy", 0.0), obj.get("gz", 0.0)
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        now_dt = datetime.now(timezone.utc)

        row = {
            "recv_iso": now_dt.isoformat(),
            "accel_mag_g": round(mag, 3),
            "t_ms": obj.get("t"),  # el firmware manda el campo como "t", no "t_ms"
            "label": self.current_label,
        }
        for k in TELEMETRY_FIELDS:
            if k in obj:
                row[k] = obj[k]
        # gx/gy/gz solo existen si ya agregaste el cambio de firmware;
        # si no vienen, quedan vacios en el CSV en vez de romper.
        self.tlm_writer.writerow(row)
        self.n_tlm += 1
        self.label_counts[self.current_label] = self.label_counts.get(self.current_label, 0) + 1
        if self.n_tlm % 20 == 0:  # cada ~1s a 20Hz - no se pierde casi nada si el script se cae
            self.tlm_file.flush()

        publish_dash({
            **row, "session_label": self.label, "segment_label": self.current_label,
            "n_tlm": self.n_tlm,
            "n_manual_marks": self.n_manual_marks, "n_auto_candidates": self.n_auto_candidates,
            # Solo presentes si el dato vino por LoRa (net = a que WiFi esta
            # conectado el ESP32, como diagnostico; _rssi/_snr = calidad del
            # enlace LoRa). None si vino por USB/TCP - el dashboard lo maneja.
            "net": obj.get("net"), "_rssi": obj.get("_rssi"), "_snr": obj.get("_snr"),
        })

        # Buffer rodante para el detector: guarda solo lo que hace falta
        # para armar la ventana post-disparador (BUFFER_KEEP_S de margen),
        # descarta lo mas viejo cada vuelta.
        self.buffer.append((now_dt, mag, gyro_mag, obj.get("roll", 0.0), obj.get("pitch", 0.0)))
        while self.buffer and (now_dt - self.buffer[0][0]).total_seconds() > BUFFER_KEEP_S:
            self.buffer.pop(0)

        now = time.monotonic()
        if (mag > ACCEL_IMPACT_G or mag < ACCEL_FREEFALL_G) and \
           (now - self._last_auto_mark_at > AUTO_MARK_COOLDOWN_S):
            self._last_auto_mark_at = now
            self.n_auto_candidates += 1
            kind = "impacto" if mag > ACCEL_IMPACT_G else "caida_libre"
            self.log_event(obj.get("t_ms"), "CANDIDATO_AUTO", f"{kind} mag={mag:.2f}g")
            print(f"\n  [!] candidato automatico: {kind}, magnitud={mag:.2f}g")
            if self.fall_model is not None:
                self.pending_evals.append({"trigger_dt": now_dt, "trigger_t_ms": obj.get("t_ms")})

        self._process_pending_evals(now_dt)

        if self.n_tlm % 20 == 0:  # ~1 vez por segundo a 20Hz
            print(f"\r  muestras={self.n_tlm}  mag={mag:.2f}g  "
                  f"marcas={self.n_manual_marks}  candidatos={self.n_auto_candidates}  "
                  f"alertas={self.n_alerts}   ",
                  end="", flush=True)

        self._last_t_ms = obj.get("t_ms")

    def _process_pending_evals(self, now_dt):
        """Revisa si algun candidato ya cumplio los POST_WINDOW_S segundos
        de espera - si es asi, arma la ventana del buffer, corre el
        modelo, y publica el resultado (positivo o negativo, ambos se
        registran) como evento + alerta en el dashboard."""
        if not self.pending_evals:
            return
        still_pending = []
        for ev in self.pending_evals:
            elapsed = (now_dt - ev["trigger_dt"]).total_seconds()
            if elapsed < POST_WINDOW_S:
                still_pending.append(ev)
                continue
            window = [b for b in self.buffer
                      if ev["trigger_dt"] <= b[0] < ev["trigger_dt"] + timedelta(seconds=POST_WINDOW_S)]
            if len(window) < 5:
                continue  # no alcanzo a llenarse la ventana (se perdieron muestras) - se descarta
            mags = [w[1] for w in window]
            gyros = [w[2] for w in window]
            rolls = [w[3] for w in window]
            pitches = [w[4] for w in window]
            feats = extract_window_features(mags, gyros, rolls, pitches)
            if feats is None:
                continue
            vec = [[feats[name] for name in FEATURE_NAMES]]
            pred = self.fall_model.predict(vec)[0]
            proba = self.fall_model.predict_proba(vec)[0]
            confianza = float(proba[1] if pred == 1 else proba[0])

            if pred == 1:
                self.n_alerts += 1
                self.log_event(ev["trigger_t_ms"], "ALERTA_CAIDA", f"confianza={confianza:.2f}")
                print(f"\n  *** ALERTA: posible CAIDA detectada (confianza={confianza:.2f}) ***")
                publish_dash({"_kind": "alert", "confidence": confianza})
            else:
                self.log_event(ev["trigger_t_ms"], "MODELO_DESCARTA", f"confianza={confianza:.2f}")
        self.pending_evals = still_pending

    def handle_evt(self, obj):
        """Rutea los EVT: que llegan del ESP32 por su campo "type". Antes
        solo existia GESTURE y se asumia siempre - ahora el firmware
        tambien manda ALERTA_CAIDA/MODELO_DESCARTA/ALARMA_TIMEOUT/
        HUMANO_DESCARTA del detector de caidas embebido (ver
        main/myosa_field_main.c, modulo de buzzer), asi que hay que
        distinguirlos para no perder la etiqueta real en eventos.csv."""
        ev_type = obj.get("type", "GESTURE")
        if ev_type == "GESTURE":
            self.log_event(obj.get("t"), "GESTO", obj.get("dir", ""))
        elif ev_type == "ALERTA_CAIDA":
            self.n_alerts += 1
            self.log_event(obj.get("t"), "ALERTA_CAIDA", "deteccion en el ESP32 (modelo embebido)")
            print(f"\n  *** ALERTA (ESP32): posible CAIDA detectada - buzzer activo ***")
            publish_dash({"_kind": "alert", "confidence": None, "source": "esp32"})
        elif ev_type == "HUMANO_DESCARTA":
            self.log_event(obj.get("t"), "HUMANO_DESCARTA", f"cancelado con gesto {obj.get('dir', '')}")
        else:
            # MODELO_DESCARTA, ALARMA_TIMEOUT, o cualquier tipo nuevo: se
            # registra tal cual llega, sin perder informacion.
            self.log_event(obj.get("t"), ev_type, obj.get("dir", ""))

    def mark_manual(self, label):
        """Inicia un SEGMENTO con esta etiqueta: desde ahora, cada fila
        de telemetria.csv lleva label=<label> hasta que se llame de
        nuevo con otra etiqueta (o termine la sesion)."""
        if label == self.current_label:
            return  # ya estabamos en ese segmento, no hace nada
        previous = self.current_label
        self.current_label = label
        self.n_manual_marks += 1
        self.log_event(getattr(self, "_last_t_ms", None), "SEGMENTO",
                        f"{previous} -> {label}")
        print(f"\n  >>> segmento activo ahora: {label} (antes: {previous})")

    def close(self):
        self.tlm_file.close()
        self.evt_file.close()
        meta = {
            "label": self.label,
            "start_utc": self.start_time.isoformat(),
            "end_utc": datetime.now(timezone.utc).isoformat(),
            "muestras_telemetria": self.n_tlm,
            "eventos_totales": self.n_evt,
            "marcas_manuales": self.n_manual_marks,
            "candidatos_automaticos": self.n_auto_candidates,
            "umbral_impacto_g": ACCEL_IMPACT_G,
            "umbral_caida_libre_g": ACCEL_FREEFALL_G,
            "muestras_por_segmento": self.label_counts,
        }
        with open(os.path.join(self.session_dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        return meta


def open_serial_transport(args):
    try:
        import serial
    except ImportError:
        print("Falta pyserial para grabar por USB. Si corres esto con el Python normal del sistema:")
        print("  pip install pyserial")
        print("O corre este script con el Python del entorno ESP-IDF (ya lo trae).")
        sys.exit(1)

    port = args.port or find_default_port()
    if not port:
        print("No se especifico --port y no se detecto ningun adaptador USB-serie conocido.")
        print("Revisa el Administrador de dispositivos y pasa --port COMx explicitamente.")
        sys.exit(1)
    print(f"Conectando a {port} @ {args.baud}...")
    ser = serial.Serial(port, args.baud, timeout=0.1)

    def read_chunk():
        return ser.read(ser.in_waiting or 1)

    return read_chunk, ser.close


def open_udp_transport(args):
    """Escucha broadcast UDP en args.udp_listen. El firmware (UDP
    broadcast, no TCP con IP fija) manda a "todos" en su subred actual -
    no hace falta que la PC sepa su propia IP de antemano, ni que el
    ESP32 sepa la del PC. Funciona igual sin importar a que red WiFi
    este conectado el ESP32 (casa, hotspot, la del amigo), sin
    reflashear para cambiar de red."""
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.udp_listen))
    sock.settimeout(0.2)
    print(f"Escuchando broadcast UDP:{args.udp_listen}, esperando al ESP32 (conectalo a tu WiFi)...")

    def read_chunk():
        try:
            data, _addr = sock.recvfrom(4096)
            return data
        except (socket.timeout, OSError):
            return b""

    return read_chunk, sock.close


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="Puerto COM (ej. COM15). Si se omite, se intenta detectar.")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp-listen", type=int, default=None,
                     help="En vez de USB, escucha por WiFi este puerto UDP (broadcast) - el ESP32 debe estar "
                          "en la MISMA red WiFi que la PC, pero no hace falta conocer ninguna IP de antemano. "
                          "Ej: --udp-listen 5005")
    ap.add_argument("--lora-listen", type=int, default=None,
                     help="Graba via el gateway LoRa (Ethernet), sin depender de ninguna red WiFi ni de "
                          "reflashear para cambiar de lugar. Puerto UDP donde el gateway reenvia el packet "
                          "forwarder (el mismo 'Port Up' configurado en el panel del gateway). Ej: --lora-listen 1700")
    ap.add_argument("--label", required=True, help="Nombre de la sesion, ej. sesion_normal_1 o caida_prueba_1")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "recordings"))
    ap.add_argument("--no-dashboard", action="store_true", help="No levantar el dashboard web (solo consola).")
    args = ap.parse_args()

    global g_recorder

    if not args.no_dashboard:
        start_dashboard_server()

    if args.lora_listen:
        read_chunk, close_transport = open_lora_transport(args)
    elif args.udp_listen:
        read_chunk, close_transport = open_udp_transport(args)
    else:
        read_chunk, close_transport = open_serial_transport(args)

    rec = Recorder(args.label, os.path.abspath(args.out))
    g_recorder = rec
    print(f"Grabando en: {rec.session_dir}")
    print("Teclas: F = marcar CAIDA | G = marcar NORMAL | Q = terminar\n")

    buf = ""
    running = True
    try:
        while running:
            data = read_chunk()
            if data:
                buf += data.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    m = TLM_RE.search(line)
                    if m:
                        try:
                            obj = json.loads(m.group(1))
                        except json.JSONDecodeError:
                            continue
                        with dash_lock:
                            rec.handle_tlm(obj)
                        continue
                    m = EVT_RE.search(line)
                    if m:
                        try:
                            obj = json.loads(m.group(1))
                        except json.JSONDecodeError:
                            continue
                        with dash_lock:
                            rec.handle_evt(obj)

            if HAVE_MSVCRT and msvcrt.kbhit():
                key = msvcrt.getch().decode("utf-8", errors="ignore").upper()
                with dash_lock:
                    if key == "F":
                        rec.mark_manual("CAIDA")
                    elif key == "G":
                        rec.mark_manual("NORMAL")
                if key == "Q":
                    running = False
    except KeyboardInterrupt:
        pass
    finally:
        close_transport()
        meta = rec.close()
        print("\n\n--- Resumen de la sesion ---")
        for k, v in meta.items():
            print(f"  {k}: {v}")
        print(f"\nArchivos en: {rec.session_dir}")


if __name__ == "__main__":
    main()
