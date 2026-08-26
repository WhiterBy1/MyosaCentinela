"""
Listener del packet forwarder Semtech UDP (gateway HT-M7603) para la prueba
de campo del Wio-E5 en modo P2P (main/lora_wioe5_test_main.c).

El firmware actual NO usa LoRaWAN (no hay join/OTAA/ABP/AES): manda strings
planos por AT+TEST=TXLRSTR. Este script solo desempaqueta el protocolo del
gateway (Semtech UDP) y muestra el payload crudo + RSSI/SNR de cada paquete.

Si mas adelante el firmware pasa a LoRaWAN real, este script deja de
alcanzar (habria que agregar descifrado AES con las claves del dispositivo).

Uso: python lora_field_listener.py
"""
import base64
import json
import socket
from datetime import datetime, timezone

PORT = 1700

PKT_PUSH_DATA = 0
PKT_PUSH_ACK = 1
PKT_PULL_DATA = 2
PKT_PULL_ACK = 4


def decode_payload(b64_data: str) -> str:
    raw = base64.b64decode(b64_data)
    try:
        return raw.decode("ascii")
    except UnicodeDecodeError:
        return raw.hex()


def handle_push_data(sock, addr, version, token, body: bytes):
    gw_eui = body[:8].hex()
    try:
        payload = json.loads(body[8:])
    except json.JSONDecodeError:
        print(f"[{addr}] PUSH_DATA con JSON invalido: {body[8:]!r}")
        return

    for rxpk in payload.get("rxpk", []):
        text = decode_payload(rxpk.get("data", ""))
        now = datetime.now(timezone.utc).strftime("%H:%M:%S")
        print(
            f"[{now}] gw={gw_eui} freq={rxpk.get('freq')}MHz "
            f"datr={rxpk.get('datr')} rssi={rxpk.get('rssi')}dBm "
            f"snr={rxpk.get('lsnr')}dB -> {text!r}"
        )

    if "stat" in payload:
        s = payload["stat"]
        print(f"  [stat] rxnb={s.get('rxnb')} rxok={s.get('rxok')} temp={s.get('temp')}C")

    ack = bytes([version]) + token + bytes([PKT_PUSH_ACK])
    sock.sendto(ack, addr)


def handle_pull_data(sock, addr, version, token):
    ack = bytes([version]) + token + bytes([PKT_PULL_ACK])
    sock.sendto(ack, addr)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"Escuchando gateway LoRa en UDP:{PORT} (Ctrl+C para salir)")
    print("Firmware esperado: P2P plano (sin LoRaWAN). Payloads tipo 'MYOSA-N'.\n")

    while True:
        data, addr = sock.recvfrom(65535)
        if len(data) < 4:
            continue
        version, token, pkt_type = data[0], data[1:3], data[3]

        if pkt_type == PKT_PUSH_DATA:
            handle_push_data(sock, addr, version, token, data[4:])
        elif pkt_type == PKT_PULL_DATA:
            handle_pull_data(sock, addr, version, token)


if __name__ == "__main__":
    main()
