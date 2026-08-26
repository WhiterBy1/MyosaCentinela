# MyosaCentinela (CENTINELA) — MYOSA 6.0 Submission

**👉 For the full project write-up (overview, demo photos/video, features, usage instructions, tech stack) go to the official submission document:**

## [`myosa-centinela/myosa-centinela.md`](myosa-centinela/myosa-centinela.md)

That file follows the mandatory MYOSA blog submission format and is the primary deliverable for review. Everything below is just a quick map for anyone browsing the code directly.

---

## Repository layout

```
/MyosaCentinela
├─ myosa-centinela/            <- START HERE: the submission write-up, photos, and demo video
│  ├─ myosa-centinela.md
│  ├─ *.jpg / *.png            (hardware photos, dashboard screenshots)
│  └─ myosa-centinela-demo.mp4
├─ main/
│  ├─ myosa_field_main.c       <- combined field firmware (sensors + embedded fall detector + buzzer + WiFi/LoRa)
│  ├─ fall_model.h             (embedded Random Forest, generated - see tools/export_model_to_c.py)
│  └─ fall_model_wrap.cpp
├─ components/
│  └─ espressif__apds9960/     (gesture sensor driver, vendored)
├─ tools/
│  ├─ data_recorder.py         (recorder + live "Control Room" dashboard backend: serial / WiFi UDP / LoRa)
│  ├─ recorder_dashboard.html  (live monitoring dashboard)
│  ├─ fall_features.py / analyze_fall_features.py / train_fall_model.py / export_model_to_c.py
│  └─ fall_model.joblib        (PC-side trained model)
└─ paquete_amigo/              (minimal package for third-party testers - recorder + dashboard only)
```

## Quick start (firmware)

```bash
idf.py build
idf.py -p COMx flash monitor
```

## Quick start (live monitoring dashboard)

```bash
python tools/data_recorder.py --label demo --udp-listen 5005   # WiFi
# or
python tools/data_recorder.py --label demo --lora-listen 1700  # LoRa gateway
```

Then open **http://localhost:8766**.

Full details for all of this — including *why* it's built this way — are in [`myosa-centinela/myosa-centinela.md`](myosa-centinela/myosa-centinela.md).
