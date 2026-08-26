"""
Entrena una version MAS CHICA del RandomForest (menos arboles que el
modelo "grande" de tools/fall_model.joblib) y la exporta como codigo C++
para correr DENTRO del ESP32 (deteccion en el dispositivo, sin depender
del PC).

Por que menos arboles: con 200 arboles/depth=4 el C generado pesa
~570KB de fuente (innecesario - un barrido con Leave-One-Out mostro que
30 arboles/depth=4 da practicamente la misma exactitud honesta, 92.7%
vs 93.0%, con 1/6 del tamano). Ver el barrido completo en el historial
de la sesion si hace falta repetirlo.

Uso:
  python tools/export_model_to_c.py recordings/Casa_Junior_2026-08-23_121025 recordings/casa_junior_2026-08-24_102202 recordings/k_2026-08-23_125154 recordings/okop_2026-08-23_143131

Genera:
  main/fall_model.h   (clase C++ Eloquent::ML::Port::RandomForest, generada por micromlgen)
"""
import sys
import glob
from sklearn.ensemble import RandomForestClassifier
from micromlgen import port

from analyze_fall_features import build_dataset
from fall_features import FEATURE_NAMES

OUT_PATH = "main/fall_model.h"
N_ESTIMATORS = 30
MAX_DEPTH = 4


def main():
    args = sys.argv[1:]
    if not args:
        print("Uso: python tools/export_model_to_c.py <carpeta_recordings...>")
        sys.exit(1)

    folders = []
    for a in args:
        folders.extend(glob.glob(a))
    folders = [f for f in folders if f]

    print(f"Sesiones: {folders}")
    df = build_dataset(folders)
    print(f"Total ejemplos: {len(df)} ({(df['label']==1).sum()} caidas, {(df['label']==0).sum()} normal)")

    if len(df) < 8 or df["label"].nunique() < 2:
        print("Muy pocos ejemplos o falta una clase - no se exporta nada.")
        sys.exit(1)

    # Mismo orden de columnas que FEATURE_NAMES (build_dataset arma el
    # dict de features con esas mismas claves primero) - CRITICO que el
    # C generado reciba el vector en este mismo orden.
    X = df[FEATURE_NAMES].to_numpy()
    y = df["label"].to_numpy()

    clf = RandomForestClassifier(n_estimators=N_ESTIMATORS, max_depth=MAX_DEPTH, random_state=0)
    clf.fit(X, y)

    code = port(clf, classmap={0: "NORMAL", 1: "CAIDA"})

    header = f"""/* GENERADO por tools/export_model_to_c.py - NO EDITAR A MANO.
 * Reentrenar y regenerar corriendo ese script de nuevo si cambian los
 * datos de entrenamiento.
 *
 * Sesiones: {folders}
 * Ejemplos: {len(df)} ({int((df['label']==1).sum())} caidas, {int((df['label']==0).sum())} normal)
 * n_estimators={N_ESTIMATORS} max_depth={MAX_DEPTH}
 * Orden de features esperado en el vector de entrada (debe coincidir
 * EXACTO con tools/fall_features.py FEATURE_NAMES):
 *   {FEATURE_NAMES}
 */
"""

    with open(OUT_PATH, "w") as f:
        f.write(header)
        # micromlgen solo agrega <cstdarg> pero el codigo generado usa
        # uint8_t - sin esto, quien incluya fall_model.h sin <cstdint>
        # previo no compila ("uint8_t was not declared in this scope").
        f.write("#include <cstdint>\n")
        f.write(code)

    print(f"\nEscrito {OUT_PATH} ({len(code)} bytes de codigo C++)")


if __name__ == "__main__":
    main()
