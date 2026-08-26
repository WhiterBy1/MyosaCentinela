---
publishDate: 2026-08-25
title: CENTINELA
excerpt: A wearable safety device for lone/remote workers that detects falls with an on-device (embedded) ML model, sounds a buzzer alarm the worker can cancel with a gesture, and reports live over WiFi or a private LoRa gateway.
image: myosa-centinela/cover.jpg
tags:
- ESP32
- LoRaWAN
- Wearable-Safety
- Embedded-ML
- Fall-Detection
---

> A wearable "sentinel" that detects a worker's fall on-device, gives them a chance to cancel a false alarm with a gesture, and reports live over WiFi or a private LoRa link.

---

## Acknowledgements

Thanks to the MYOSA Sensors team for the MYOSA 6.0 program and the hardware kit. This project uses the official `espressif/apds9960` driver (vendored and locally patched), the ESP-IDF v6.0 framework from Espressif, and `micromlgen` to export a scikit-learn model into embeddable C++.

Thanks to our faculty mentor, **Alberto Patiño Vanegas** (Universidad Tecnológica de Bolívar), for his guidance throughout the project.

Special thanks to **Frank Luis Monsalve Morillo** for his help with everything related to 3D printing, and to professors **Alcides Augusto Ramos Zambrano** and **George Washington Archbold Taylor** for their support with any questions regarding the embedded system and electronics.

## Overview

We're from Colombia's Caribbean coast, and out here the heat is no small thing — on a lot of days it's brutal enough on its own. Now picture a worker out alone in a field, a farm, a plant, somewhere off on their own with no one nearby: a heat stroke, a fainting spell for any reason, a hard blow to the head that knocks them down. Nobody sees it happen. There's often no cell signal to call for help either. In that situation, the most likely outcome is that help arrives too late — or doesn't arrive in time to save them at all. That's the exact problem **CENTINELA** was built to close, and the MYOSA kit's components are what let us actually build it.

**CENTINELA** is a wearable safety device for lone or remote worker monitoring ("lone worker safety"). An ESP32 with a small suite of I2C sensors continuously tracks the wearer's orientation and motion; when it recognizes the pattern of a fall, it doesn't just react to the instant of impact — it evaluates roughly three seconds of what happens *after* the trigger, because a real fall is followed by stillness, while a false alarm (setting the device down, a hard step) is not. If the model decides it's a real fall, an onboard buzzer sounds an alarm that gets progressively faster over 10 seconds; the worker can cancel it at any time with a simple gesture sequence (any pair of opposite swipes — left→right, right→left, up→down, or down→up) detected by the onboard gesture sensor. Only if nobody cancels within that window does the event escalate.

**A note on scope:** our original proposal framed the target signal as "abnormal immobility" in general — heat stroke, fainting, dehydration, or a fall can all leave a worker motionless. In practice, a fall is the one of those events an IMU can detect with high confidence and low ambiguity: it has both a distinctive kinematic signature (an impact or free-fall spike) and a reliable aftermath (the wearer goes still). Heat stroke or fainting *without* an accompanying fall don't produce that same spike, so falls became the first fully-implemented proxy for immobility — the trigger the shipped classifier is actually trained and validated on. A fall-independent check (an alert if no motion at all is seen for several minutes, regardless of what preceded it) would extend coverage to those other cases, and is the natural next addition — but it isn't part of the detector shipped here.

**What problem it solves:** in field work (industrial, agricultural, mining, etc.) a lone worker can fall, get hurt, or lose consciousness with nobody around to notice — and there's often no cellphone signal to call for help. CENTINELA aims to close that gap with low-cost hardware that (a) makes the fall-detection decision **on the device itself**, so it works even with no WiFi, no cellular data, and no PC nearby, and (b) still gives the worker a fast, deliberate way to dismiss a false alarm before it escalates, instead of spamming false positives.

**Who it's for:** safety supervisors in industrial/field settings who need basic fall detection and live monitoring for workers in areas with poor or no cellular coverage.

**How it works (summary):** the ESP32 splits the work across its two cores. One core reads the IMU (MPU6050) at 100Hz to keep orientation current and feed a rolling buffer; the other core handles gestures, environmental sensors, the OLED display, and radio/network I/O. A lightweight, fixed-threshold trigger (a sudden spike or a moment of free-fall in the acceleration magnitude) arms a 3-second observation window; once it elapses, 15 statistical features (mean/std/min/max of acceleration and gyroscope magnitude, fraction of the window spent in impact/free-fall, and — the single most predictive one — how still the second half of the window is) are extracted and fed to a small Random Forest classifier that runs directly on the ESP32 (no network round-trip required to decide). Telemetry is broadcast live as JSON over WiFi (UDP broadcast, so it works on any network without reflashing) and, for field deployments without WiFi at all, over a LoRa P2P link to a private gateway wired straight into a laptop by Ethernet.

**Key features:**
* On-device (embedded) fall detection: a Random Forest trained on real recorded sessions, exported to C++ and running natively on the ESP32 — no PC or cloud needed to make the call
* Buzzer alarm with an escalating beep pattern, cancellable within 10 seconds by a gesture sequence, so a worker who is fine can dismiss a false alarm before it's ever reported
* Dual connectivity: WiFi (UDP broadcast, auto-adapts to whatever network/subnet it joins) for live monitoring/testing, and a LoRa P2P link to a private gateway for field use with zero WiFi/cellular dependency
* A live "Control Room" web dashboard: animated KPI gauges for acceleration, temperature and pressure, and a two-stage alert view that mirrors the on-device logic (an "evaluating" state while the 10-second cancel window is open, escalating to a confirmed critical alert only if nobody cancels)
* Environmental sensing (temperature, pressure, altitude) and gesture recognition (up/down/left/right) via the APDS9960
* A full offline data-collection and training pipeline used to build the shipped model: segment-based labeling, a data-quality audit step, Leave-One-Out cross-validation, and a script that re-exports a freshly trained model straight to embeddable C++

---

## Demo / Examples

### Images

<p align="center">
  <img src="/assets/images/myosa-centinela/final-device-worn.jpg" width="800"><br/>
  <i>The final CENTINELA wearable, 3D-printed enclosure in white</i>
</p>

<p align="center">
  <img src="/assets/images/myosa-centinela/device-active.jpg" width="800"><br/>
  <i>Device powered on: OLED status readout, gesture sensor, and 18650 cell visible</i>
</p>

<p align="center">
  <img src="/assets/images/myosa-centinela/dashboard-normal.png" width="800"><br/>
  <i>The live Control Room dashboard during normal operation — acceleration, temperature, and pressure gauges, plus the live acceleration chart</i>
</p>

<p align="center">
  <img src="/assets/images/myosa-centinela/dashboard-fall-confirmed.png" width="800"><br/>
  <i>The Control Room dashboard after a fall alert went unconfirmed by the worker and escalated</i>
</p>

**Build process:** the first enclosure iteration was printed in black; we later moved to white for the final version after running out of the original filament and revising the fit based on what we learned from the black prototype.

<p align="center">
  <img src="/assets/images/myosa-centinela/assembly-bench.jpg" width="800"><br/>
  <i>Sensors on the bench during integration: APDS9960 gesture sensor, OLED (showing BMP180 readings), and the ESP32 dev board</i>
</p>

<p align="center">
  <img src="/assets/images/myosa-centinela/first-prototype-black.jpg" width="800"><br/>
  <i>First enclosure iteration (black), alongside the Heltec HT-M7603 gateway and the MYOSA kit</i>
</p>

<p align="center">
  <img src="/assets/images/myosa-centinela/team-with-device.jpg" width="800"><br/>
  <i>The team with the final device</i>
</p>

### Videos

<video controls width="100%">
  <source src="/myosa-centinela-demo.mp4" type="video/mp4">
</video>

---

## Features (Detailed)

### 1. Orientation and motion sensing (MPU6050)

One ESP32 core is dedicated entirely to reading the MPU6050 at ~100Hz and updating a complementary filter (accelerometer + gyroscope) that produces stable roll/pitch/yaw without ever waiting on gestures, the display, or the radio. That same core samples acceleration/gyroscope magnitude at a decimated 20Hz specifically for the fall detector — matching the rate the training data was recorded at, since a model trained on 20Hz statistics doesn't generalize correctly if it's fed 100Hz-computed features (much more incidental motion noise shows up at the higher rate and inflates the model's "stillness" feature).

### 2. Embedded fall detection (on-device Random Forest)

A candidate trigger (acceleration magnitude above ~1.8g, or below ~0.3g during free-fall) arms a 3-second observation window. When it elapses, the firmware extracts 15 features from the buffered window — mean/std/min/max/range of acceleration magnitude, mean/std/max of gyroscope magnitude, fraction of samples in free-fall/impact, and (the strongest predictor across every training run) how still the *second half* of the window is compared to the first half — and runs them through a Random Forest (30 trees, depth 4) trained with scikit-learn and exported to embeddable C++ with `micromlgen`. This is the same feature-extraction logic used during training (`tools/fall_features.py`), reimplemented in C so the on-device numbers match what the model actually learned. The whole decision happens locally: no WiFi, no LoRa, and no PC in the loop.

### 3. Buzzer alarm with gesture-based cancellation

If the model says "fall," a buzzer (driven from a spare GPIO) starts beeping — starting slow and getting progressively faster over a 10-second window. During that window, the worker can cancel the alarm with any pair of opposite gesture swipes on the APDS9960 (left→right, right→left, up→down, down→up) within a couple of seconds of each other. Cancelling logs a "cancelled by worker" event and immediately silences the buzzer; letting the 10 seconds run out without a cancellation confirms the alert. This mirrors the project's core design decision: don't judge an isolated impact spike in isolation — give it a few seconds of context, and give the person a fast way to say "I'm fine" before anything gets escalated.

### 4. Environmental sensing (BMP180) and gestures (APDS9960)

The BMP180 supplies temperature, pressure, and estimated altitude, refreshed in a background cache so it never competes with the higher-rate sensors for I2C bus time. The APDS9960 detects the four directional swipe gestures (with a calibrated cooldown to avoid double-counting the "echo" of a hand pulling away) — used both as the alarm-cancel mechanism above and as a general-purpose manual alert signal.

### 5. Dual connectivity: WiFi broadcast + private LoRa gateway

For live testing, the ESP32 joins WiFi (trying a configurable list of candidate networks) and broadcasts telemetry as UDP — the broadcast address is computed from whatever IP/subnet it's actually assigned, so a listening PC on the same network receives it with zero reconfiguration or reflashing, even across different networks (home WiFi, a phone hotspot, a different office). For real field deployment without any WiFi or cellular coverage, the ESP32 talks over UART to a Seeed Wio-E5 LoRa module (903.9MHz / SF7 / BW125, public sync word), which reaches a private Heltec HT-M7603 gateway running in Semtech UDP packet-forwarder mode. The gateway is wired directly into a laptop over Ethernet — no router, no internet, no LoRaWAN network server required — and a Python listener decodes the packets along with real RSSI/SNR readings.

### 6. Live "Control Room" monitoring dashboard

`tools/recorder_dashboard.html` is a self-contained, animated web dashboard (served locally, no external dependencies) that shows a worker/device presence indicator, animated radial gauges (acceleration, temperature, pressure) colored by severity, a live acceleration chart with the fall thresholds drawn in, and a recent-events log. Its alert UI directly mirrors the two-stage, gesture-cancellable design described above: an "evaluating" state with a live 10-second countdown ring while the on-device window is open, a quiet dismissal notice if the worker cancels in time, and a loud, persistent critical banner only if nobody does.

### 7. Offline data-collection and training pipeline

`tools/data_recorder.py` records labeled sessions (continuous CAIDA/NORMAL segments, not instantaneous marks) over serial, WiFi, or LoRa, with the same live dashboard used for monitoring doubling as a recording aid. `tools/analyze_fall_features.py` runs Leave-One-Out cross-validation to honestly estimate real-world accuracy on a small dataset, and `tools/train_fall_model.py` / `tools/export_model_to_c.py` produce, respectively, the PC-side model and the embeddable C++ version flashed onto the device — trained on 754 labeled windows (121 falls) collected across several sessions, after an explicit data-quality audit step that excludes sessions with sensor glitches, missing labels, or suspiciously stuck readings.

---

## Usage Instructions

### Firmware (ESP32, ESP-IDF v6.0)

```plaintext
# Activate the ESP-IDF environment (Windows example)
idf.py build
idf.py -p COMx flash monitor
```

`main/myosa_field_main.c` is the combined field firmware: sensors, embedded fall detector, buzzer/alarm logic, WiFi, and LoRa all in one image.

### Live monitoring dashboard

```plaintext
# WiFi (device and PC on the same network/hotspot):
python tools/data_recorder.py --label demo --udp-listen 5005

# LoRa (device -> Wio-E5 -> Heltec HT-M7603 gateway -> Ethernet -> PC):
python tools/data_recorder.py --label demo --lora-listen 1700
```

Then open **http://localhost:8766** for the live Control Room dashboard.

### Retraining / re-exporting the embedded model (optional)

```plaintext
python tools/analyze_fall_features.py recordings/*          # honest Leave-One-Out evaluation
python tools/train_fall_model.py recordings/session_a ...   # PC-side model (tools/fall_model.joblib)
python tools/export_model_to_c.py recordings/session_a ...  # regenerates main/fall_model.h for the firmware
```

---

## Tech Stack

* **ESP32** (ESP-IDF v6.0, dual-core FreeRTOS architecture)
* **MPU6050** — IMU (accelerometer + gyroscope) for orientation and the fall-detection feature window
* **BMP180** — temperature / pressure / altitude
* **APDS9960** — gesture recognition (official Espressif driver, vendored with local patches)
* **OLED SSD1306** — onboard status display
* **18650 Li-ion battery** — portable power supply
* **Seeed Wio-E5** — LoRa module (UART, AT command interface)
* **Heltec HT-M7603** — private LoRaWAN indoor gateway (OpenWrt + LuCI, Semtech UDP packet-forwarder mode)
* **scikit-learn** — Random Forest classifier (trained on the PC)
* **micromlgen** — exports the trained model to embeddable C++ that runs directly on the ESP32
* **Python 3** — data recorder, live dashboard backend, training/evaluation scripts
* **HTML / CSS / JS (Canvas + SVG)** — the live "Control Room" monitoring dashboard

---

## Requirements / Installation

```bash
# ESP-IDF v6.0 (via the official Espressif installer)
# See: https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html

# Python side (recorder, dashboard, training/export scripts):
pip install pandas scikit-learn joblib micromlgen

python tools/data_recorder.py --label demo --udp-listen 5005
```

---

## File Structure (Optional)

```
/CENTINELA
├─ main/
│  ├─ myosa_field_main.c      (combined field firmware: sensors + embedded fall detector + buzzer + WiFi/LoRa)
│  ├─ fall_model.h            (Random Forest, generated by tools/export_model_to_c.py - do not hand-edit)
│  └─ fall_model_wrap.cpp     (C++ -> C bridge so myosa_field_main.c can call the embedded model)
├─ components/
│  └─ espressif__apds9960/    (gesture driver, vendored)
├─ tools/
│  ├─ data_recorder.py        (recorder + live dashboard backend: serial / WiFi UDP / LoRa)
│  ├─ recorder_dashboard.html (live "Control Room" monitoring dashboard)
│  ├─ fall_features.py        (feature extraction shared by training and the C port)
│  ├─ analyze_fall_features.py(Leave-One-Out feasibility/evaluation)
│  ├─ train_fall_model.py     (trains the PC-side model, tools/fall_model.joblib)
│  └─ export_model_to_c.py    (re-generates main/fall_model.h from labeled recordings)
└─ myosa-centinela/
   └─ myosa-centinela.md      (this blog)
```

---

## Contribution Notes (Optional)

Developed as part of MYOSA 6.0. Suggestions and issue reports are welcome on the project's repository.
