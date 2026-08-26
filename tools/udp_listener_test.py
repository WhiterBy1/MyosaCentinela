"""
Listener UDP simple para probar el packet forwarder Semtech del gateway HT-M7603.
No decodifica LoRaWAN, solo confirma que los paquetes UDP llegan a la PC.
Uso: python udp_listener_test.py
"""
import socket
import json

PORT = 1700

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
print(f"Escuchando UDP en puerto {PORT}... (Ctrl+C para salir)")

# Tipos de paquete del protocolo Semtech UDP packet forwarder
PKT_TYPES = {
    0: "PUSH_DATA",
    1: "PUSH_ACK",
    2: "PULL_DATA",
    3: "PULL_RESP",
    4: "PULL_ACK",
    5: "TX_ACK",
}

while True:
    data, addr = sock.recvfrom(65535)
    if len(data) < 4:
        print(f"[{addr}] paquete demasiado corto: {data!r}")
        continue

    version, token, pkt_type = data[0], data[1:3], data[3]
    type_name = PKT_TYPES.get(pkt_type, f"desconocido({pkt_type})")
    print(f"\n[{addr}] tipo={type_name} version={version}")

    if pkt_type == 0 and len(data) > 12:  # PUSH_DATA trae un JSON despues del header de 12 bytes
        gw_eui = data[4:12].hex()
        try:
            payload = json.loads(data[12:])
            print(f"  gateway_eui={gw_eui}")
            print(f"  json={json.dumps(payload, indent=2)}")
        except json.JSONDecodeError:
            print(f"  (no se pudo parsear JSON) raw={data[12:]!r}")

        # Responder PUSH_ACK para que el gateway no reintente
        ack = bytes([version]) + token + bytes([1])
        sock.sendto(ack, addr)

    elif pkt_type == 2:  # PULL_DATA -> responder PULL_ACK para mantener el "keep-alive"
        ack = bytes([version]) + token + bytes([4])
        sock.sendto(ack, addr)
        print("  (PULL_DATA recibido, PULL_ACK enviado)")
