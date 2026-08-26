"""
Extraccion de features compartida entre el entrenamiento
(analyze_fall_features.py / train_fall_model.py) y la deteccion en vivo
(data_recorder.py). TIENE QUE SER LA MISMA logica en los dos lados - si
se desalinean, el modelo entrenado no significa nada al aplicarlo en
vivo.

Filosofia (confirmada con datos reales, ver tools/analyze_fall_features.py):
no se juzga el pico del golpe aislado - se evalua una ventana de
POST_WINDOW_S segundos DESPUES de que un candidato (umbral de magnitud)
dispara, porque lo que mas distingue una caida real de una falsa alarma
es la QUIETUD en la segunda mitad de esa ventana, no la magnitud del
pico en si.
"""
import numpy as np

POST_WINDOW_S = 3.0
ACCEL_IMPACT_G = 1.8
ACCEL_FREEFALL_G = 0.3

FEATURE_NAMES = [
    "mag_mean", "mag_std", "mag_min", "mag_max", "mag_range",
    "gyro_mean", "gyro_std", "gyro_max",
    "frac_freefall", "frac_impact",
    "late_std", "late_mean_dev", "early_late_std_ratio",
    "roll_range", "pitch_range",
]


def extract_window_features(mag, gyro_mag, roll, pitch):
    """mag, gyro_mag, roll, pitch: arrays/listas de la ventana ya
    recortada (POST_WINDOW_S segundos desde el disparador). Devuelve un
    dict con las mismas claves en FEATURE_NAMES, o None si la ventana
    trae muy pocas muestras para ser confiable."""
    mag = np.asarray(mag, dtype=float)
    gyro_mag = np.asarray(gyro_mag, dtype=float)
    roll = np.asarray(roll, dtype=float)
    pitch = np.asarray(pitch, dtype=float)

    n = len(mag)
    if n < 5:
        return None

    half = n // 2
    early, late = mag[:half], mag[half:]

    return {
        "mag_mean": float(mag.mean()), "mag_std": float(mag.std()),
        "mag_min": float(mag.min()), "mag_max": float(mag.max()),
        "mag_range": float(mag.max() - mag.min()),
        "gyro_mean": float(gyro_mag.mean()), "gyro_std": float(gyro_mag.std()),
        "gyro_max": float(gyro_mag.max()),
        "frac_freefall": float((mag < ACCEL_FREEFALL_G).mean()),
        "frac_impact": float((mag > ACCEL_IMPACT_G).mean()),
        "late_std": float(late.std()),
        "late_mean_dev": float(abs(late.mean() - 1.0)),
        "early_late_std_ratio": float((early.std() + 1e-6) / (late.std() + 1e-6)),
        "roll_range": float(roll.max() - roll.min()) if len(roll) else 0.0,
        "pitch_range": float(pitch.max() - pitch.min()) if len(pitch) else 0.0,
    }


def features_to_vector(feats):
    """dict -> lista en el orden de FEATURE_NAMES, para pasarle al modelo."""
    return [feats[name] for name in FEATURE_NAMES]
