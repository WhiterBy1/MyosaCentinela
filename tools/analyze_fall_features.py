"""
Prueba de viabilidad de un detector de caidas: NO clasifica el instante
del pico aislado - toma cada candidato automatico (umbral de magnitud ya
detectado durante la grabacion) como "disparador", y evalua una ventana
de POST_WINDOW_S segundos DESPUES de ese disparador para decidir si fue
una caida real o una falsa alarma. Asi es como funcionaria en el
dispositivo real: el umbral dispara, se espera un par de segundos
viendo que pasa despues, y AHI se decide - no en el instante del golpe.

Uso:
  python tools/analyze_fall_features.py recordings/Casa_Junior_2026-08-23_121025
  python tools/analyze_fall_features.py recordings/*   # combina varias sesiones
"""
import glob
import sys
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import LeaveOneOut, cross_val_predict
from sklearn.metrics import confusion_matrix, classification_report

from fall_features import extract_window_features, POST_WINDOW_S


def load_session(folder):
    tlm = pd.read_csv(f"{folder}/telemetria.csv")
    tlm["t"] = pd.to_datetime(tlm["recv_iso"])
    evt = pd.read_csv(f"{folder}/eventos.csv")
    evt["t"] = pd.to_datetime(evt["recv_iso"])
    return tlm, evt


def build_dataset(folders):
    rows = []
    for folder in folders:
        tlm, evt = load_session(folder)
        auto = evt[evt["type"] == "CANDIDATO_AUTO"]
        for _, e in auto.iterrows():
            end_time = e["t"] + pd.Timedelta(seconds=POST_WINDOW_S)
            win = tlm[(tlm["t"] >= e["t"]) & (tlm["t"] < end_time)]
            if len(win) < 5:
                continue
            gyro_mag = (win["gx"]**2 + win["gy"]**2 + win["gz"]**2) ** 0.5
            feats = extract_window_features(
                win["accel_mag_g"], gyro_mag, win["roll"], win["pitch"])
            if feats is None:
                continue
            # etiqueta real: que segmento estaba activo en el momento del disparador
            idx = (tlm["t"] - e["t"]).abs().idxmin()
            label = tlm.loc[idx, "label"]
            feats["label"] = 1 if label == "CAIDA" else 0
            feats["session"] = folder
            feats["t"] = e["t"]
            rows.append(feats)
    return pd.DataFrame(rows)


def main():
    args = sys.argv[1:]
    if not args:
        print("Uso: python tools/analyze_fall_features.py <carpeta_recordings...>")
        sys.exit(1)

    folders = []
    for a in args:
        folders.extend(glob.glob(a))
    folders = [f for f in folders if f]

    print(f"Sesiones: {folders}")
    df = build_dataset(folders)
    print(f"\nTotal ejemplos (disparadores con ventana de {POST_WINDOW_S}s despues): {len(df)}")
    print(df["label"].value_counts().rename({1: "CAIDA", 0: "NORMAL/falsa_alarma"}))

    if len(df) < 8 or df["label"].nunique() < 2:
        print("\nMuy pocos ejemplos o falta una clase - graba mas sesiones antes de seguir.")
        return

    feature_cols = [c for c in df.columns if c not in ("label", "session", "t")]
    X = df[feature_cols].to_numpy()
    y = df["label"].to_numpy()

    # Leave-One-Out: con tan pocos ejemplos (positivos sobre todo), es la
    # forma mas honesta de estimar que tan bien generaliza - entrena con
    # todos menos uno, prueba en el que quedo afuera, repite para cada
    # ejemplo. Un accuracy alto aqui es mas creible que un train/test
    # split normal con tan pocos datos.
    clf = RandomForestClassifier(n_estimators=200, max_depth=4, random_state=0)
    loo = LeaveOneOut()
    y_pred = cross_val_predict(clf, X, y, cv=loo)

    print("\n=== Resultado con validacion Leave-One-Out (honesto, no sobreajustado) ===")
    print(classification_report(y, y_pred, target_names=["NORMAL/falsa_alarma", "CAIDA"], zero_division=0))
    print("Matriz de confusion (filas=real, columnas=prediccion):")
    print("           pred_NORMAL  pred_CAIDA")
    cm = confusion_matrix(y, y_pred)
    for i, row_name in enumerate(["real_NORMAL", "real_CAIDA"]):
        print(f"{row_name:12s} {cm[i][0]:10d}  {cm[i][1]:10d}")

    # Entrena UNA vez con todo para ver que features pesan mas (orientativo,
    # no es el modelo final - eso se decide con mas datos)
    clf.fit(X, y)
    importances = sorted(zip(feature_cols, clf.feature_importances_), key=lambda x: -x[1])
    print("\n=== Features mas importantes (orientativo) ===")
    for name, imp in importances:
        print(f"  {name:20s} {imp:.3f}")

    print(f"\n*** AVISO: {len(df)} ejemplos ({(y==1).sum()} caidas) es MUY poco para un modelo")
    print("real. Esto es una prueba de viabilidad: si el resultado se ve bien, confirma que")
    print("el enfoque (ventana post-evento, no el pico solo) tiene señal util. No es un")
    print("modelo listo para producción - para eso hacen falta muchas mas sesiones y,")
    print("idealmente, mas de una persona/superficie/tipo de caida.")


if __name__ == "__main__":
    main()
