"""
Entrena el modelo FINAL de deteccion de caidas (con TODOS los datos
buenos disponibles, no Leave-One-Out - eso era solo para medir que tan
bien generaliza en tools/analyze_fall_features.py) y lo guarda en
tools/fall_model.joblib para que data_recorder.py lo cargue y detecte
en vivo.

Uso:
  python tools/train_fall_model.py recordings/Casa_Junior_2026-08-23_121025 recordings/casa_junior_2026-08-24_102202 recordings/k_2026-08-23_125154 recordings/okop_2026-08-23_143131
"""
import sys
import glob
import json
import joblib
from sklearn.ensemble import RandomForestClassifier

# Reusa la construccion del dataset del script de viabilidad, para no
# duplicar la logica de "candidato + ventana de 3s + etiqueta real".
from analyze_fall_features import build_dataset
from fall_features import FEATURE_NAMES, POST_WINDOW_S

MODEL_PATH = "tools/fall_model.joblib"


def main():
    args = sys.argv[1:]
    if not args:
        print("Uso: python tools/train_fall_model.py <carpeta_recordings...>")
        sys.exit(1)

    folders = []
    for a in args:
        folders.extend(glob.glob(a))
    folders = [f for f in folders if f]

    print(f"Sesiones usadas para el modelo final: {folders}")
    df = build_dataset(folders)
    print(f"Total ejemplos: {len(df)} ({(df['label']==1).sum()} caidas, {(df['label']==0).sum()} normal)")

    if len(df) < 8 or df["label"].nunique() < 2:
        print("Muy pocos ejemplos o falta una clase - no se entrena nada.")
        sys.exit(1)

    X = df[FEATURE_NAMES].to_numpy()
    y = df["label"].to_numpy()

    clf = RandomForestClassifier(n_estimators=200, max_depth=4, random_state=0)
    clf.fit(X, y)

    joblib.dump({
        "model": clf,
        "feature_names": FEATURE_NAMES,
        "post_window_s": POST_WINDOW_S,
        "trained_on_sessions": folders,
        "n_examples": len(df),
        "n_caidas": int((df["label"] == 1).sum()),
    }, MODEL_PATH)

    print(f"\nModelo guardado en {MODEL_PATH}")
    print("*** Entrenado con TODOS los datos disponibles (no LOO) - para saber que tan bien")
    print("generaliza de verdad, revisa el resultado de tools/analyze_fall_features.py, no")
    print("la precision de este modelo sobre sus propios datos de entrenamiento (esa siempre")
    print("se ve mejor de lo que es en la realidad).")


if __name__ == "__main__":
    main()
