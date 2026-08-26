"""
Servidor local del dashboard de campo (solo libreria estandar de Python,
sin pip install). Dos trabajos en paralelo:

  1) Escucha el packet forwarder Semtech UDP del gateway HT-M7603 (puerto
     1700), igual que tools/lora_field_listener.py, y decodifica el
     payload compacto que manda main/myosa_field_main.c
     ("seq=..,r=..,p=..,y=..,t=..,pr=..,px=..,lt=..,ev=..").

  2) Sirve tools/lora_dashboard.html en http://localhost:8765 y empuja
     cada paquete nuevo al navegador via Server-Sent Events (/events) -
     no hace falta websockets ni ninguna libreria externa, EventSource
     es nativo del navegador.

Uso: python tools/lora_dashboard_server.py
Despues abre http://localhost:8765 en el navegador.
"""
import base64
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_PORT = 1700
HTTP_PORT = 8765
HTML_PATH = os.path.join(os.path.dirname(__file__), "lora_dashboard.html")

state_lock = threading.Lock()
state = {"version": 0, "data": None}

PKT_PUSH_DATA = 0
PKT_PUSH_ACK = 1
PKT_PULL_DATA = 2
PKT_PULL_ACK = 4


def parse_payload(text):
    """'seq=3,r=5.9,p=-0.9,...' -> {'seq': 3, 'r': 5.9, 'p': -0.9, ...}"""
    out = {}
    for part in text.split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        try:
            out[k] = float(v) if ("." in v or "-" in v[1:]) else int(v)
        except ValueError:
            out[k] = v
    return out


def publish(data):
    with state_lock:
        state["version"] += 1
        state["data"] = data


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    print(f"[udp] escuchando gateway LoRa en UDP:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(65535)
        if len(data) < 4:
            continue
        version, token, pkt_type = data[0], data[1:3], data[3]

        if pkt_type == PKT_PUSH_DATA and len(data) > 12:
            try:
                payload = json.loads(data[12:])
            except json.JSONDecodeError:
                continue
            for rxpk in payload.get("rxpk", []):
                try:
                    raw = base64.b64decode(rxpk.get("data", ""))
                    text = raw.decode("ascii")
                except (ValueError, UnicodeDecodeError):
                    continue
                parsed = parse_payload(text)
                parsed["_rssi"] = rxpk.get("rssi")
                parsed["_snr"] = rxpk.get("lsnr")
                publish(parsed)
                print(f"[udp] {text}  rssi={rxpk.get('rssi')} snr={rxpk.get('lsnr')}")
            ack = bytes([version]) + token + bytes([PKT_PUSH_ACK])
            sock.sendto(ack, addr)
        elif pkt_type == PKT_PULL_DATA:
            ack = bytes([version]) + token + bytes([PKT_PULL_ACK])
            sock.sendto(ack, addr)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(HTML_PATH, "rb") as f:
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
                    with state_lock:
                        v, d = state["version"], state["data"]
                    if v != last_version and d is not None:
                        last_version = v
                        self.wfile.write(f"data: {json.dumps(d)}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


def main():
    threading.Thread(target=udp_listener, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[http] dashboard listo en http://localhost:{HTTP_PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
