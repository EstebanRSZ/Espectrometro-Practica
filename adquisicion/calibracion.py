"""
Herramienta de calibracion del espectrometro con laseres de referencia.

Permite ajustar dos parametros del modelo geometrico:
  - L   : distancia red → plano detector [mm]  (parametro principal)
  - offset_pasos : desplazamiento del orden 0 respecto al home [pasos]

Flujo:
  1. Cargar un CSV de medicion (o capturar en vivo desde el serial).
  2. Detectar automaticamente los picos en el espectro crudo.
  3. El usuario asocia cada pico a su longitud de onda conocida (laser rojo
     ~650 nm o laser verde ~532 nm).
  4. Minimizar el error entre lambda_geometrica(pico) y lambda_laser.
  5. Mostrar L y offset_pasos calibrados y guardar en calibracion.json.

Uso:
    # Calibrar con un CSV existente
    python calibracion.py --csv datos/medicion_20260513_120000.csv

    # Calibrar capturando desde el serial
    python calibracion.py --puerto /dev/ttyUSB0

    # Calibrar con datos de demo
    python calibracion.py --demo
"""

import argparse
import json
import sys
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar, minimize
from scipy.signal import find_peaks

sys.path.insert(0, str(Path(__file__).parent))
from correccion_espectral import s_norm

# ── Constantes por defecto ────────────────────────────────────────────────────
D_UM         = 1.0      # periodo red [µm]
MM_POR_PASO  = 0.0307   # [mm/paso] valor nominal
L_NOM_MM     = 100.0    # L nominal [mm]

# Laseres de referencia conocidos
LASERES = {
    'rojo':  650.0,   # nm
    'verde': 532.0,   # nm
}

CAL_JSON = Path(__file__).parent / 'calibracion.json'


# ── Modelo geometrico ─────────────────────────────────────────────────────────

def lambda_de_pasos(pasos, L_mm, offset_pasos=0, mm_por_paso=MM_POR_PASO):
    """Convierte numero de pasos en longitud de onda [nm]."""
    y_mm    = (pasos - offset_pasos) * mm_por_paso
    theta   = np.arctan2(y_mm, L_mm)
    lam_um  = D_UM * np.sin(theta)
    return lam_um * 1000.0  # nm


def pasos_de_lambda(lam_nm, L_mm, offset_pasos=0, mm_por_paso=MM_POR_PASO):
    """Convierte longitud de onda [nm] en numero de pasos (inverso del modelo)."""
    lam_um  = lam_nm / 1000.0
    theta   = np.arcsin(lam_um / D_UM)
    y_mm    = L_mm * np.tan(theta)
    return y_mm / mm_por_paso + offset_pasos


# ── Carga de datos ────────────────────────────────────────────────────────────

def cargar_csv(ruta_csv):
    """
    Carga un CSV generado por adquisicion.py.
    Devuelve (lambda_nm, v_raw, v_dark).
    """
    lambdas, v_raws, v_dark = [], [], 0.0
    with open(ruta_csv, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for fila in reader:
            if not fila:
                continue
            if fila[0].startswith('#'):
                if 'v_dark' in fila[0].lower() and len(fila) > 1:
                    try:
                        v_dark = float(fila[1])
                    except ValueError:
                        pass
                continue
            if fila[0] == 'lambda_nm':
                continue
            try:
                lambdas.append(float(fila[0]))
                v_raws.append(int(fila[1]))
            except (ValueError, IndexError):
                pass
    return np.array(lambdas), np.array(v_raws, dtype=float), v_dark


def cargar_demo():
    """Genera datos demo con dos picos laser conocidos."""
    from correccion_espectral import _TABLA_NM, _TABLA_SNORM
    rng = np.random.default_rng(7)
    lam = np.linspace(400, 700, 400)
    v_dark = 100.0
    # Simular pico laser rojo (650 nm) y verde (532 nm)
    spd = (
        3000 * np.exp(-0.5 * ((lam - 532) / 3) ** 2) +
        2500 * np.exp(-0.5 * ((lam - 650) / 3) ** 2)
    )
    v_raw = (spd * s_norm(lam) + v_dark + rng.normal(0, 10, len(lam))).astype(int)
    return lam, v_raw.astype(float), v_dark


def capturar_serial(puerto):
    """Captura una medicion completa desde el ESP32."""
    try:
        import serial
    except ImportError:
        sys.exit('Error: instala pyserial →  pip install pyserial')

    lambdas, v_raws, v_dark = [], [], 0.0
    print(f'Capturando desde {puerto}...')
    with serial.Serial(puerto, 115200, timeout=5) as ser:
        en_sweep = False
        while True:
            linea = ser.readline().decode('ascii', errors='replace').strip()
            if not linea:
                continue
            print(f'  {linea}')
            if linea.startswith('#DARK:'):
                try:
                    v_dark = float(linea.split(':')[1])
                except ValueError:
                    pass
            elif linea == '#SWEEP_START':
                en_sweep = True
            elif linea == '#SWEEP_END':
                break
            elif en_sweep and ',' in linea:
                try:
                    parts = linea.split(',')
                    lambdas.append(float(parts[0]))
                    v_raws.append(int(parts[1]))
                except (ValueError, IndexError):
                    pass
    return np.array(lambdas), np.array(v_raws, dtype=float), v_dark


# ── Deteccion de picos ────────────────────────────────────────────────────────

def detectar_picos(lam, i_medida, n_picos=4):
    """
    Encuentra hasta n_picos picos prominentes en el espectro.
    Devuelve (indices_picos, lambda_picos).
    """
    if len(i_medida) < 5:
        return np.array([]), np.array([])

    # Normalizar para hacer find_peaks independiente de la escala
    i_norm = i_medida / (i_medida.max() + 1e-9)
    min_distancia = max(3, len(lam) // 50)
    picos, props = find_peaks(
        i_norm,
        height=0.05,
        distance=min_distancia,
        prominence=0.03
    )
    if len(picos) == 0:
        return np.array([]), np.array([])

    # Ordenar por prominencia descendente y tomar los mejores
    proms = props['prominences']
    orden = np.argsort(-proms)
    picos = picos[orden[:n_picos]]
    picos = np.sort(picos)
    return picos, lam[picos]


# ── Refinamiento sub-pixel del pico (ajuste parabolico) ──────────────────────

def refinar_pico(lam, intensidad, idx_pico, ventana=5):
    """Ajuste parabolico alrededor de un pico para subir la precision."""
    i0 = max(0, idx_pico - ventana)
    i1 = min(len(lam) - 1, idx_pico + ventana)
    x = lam[i0:i1+1]
    y = intensidad[i0:i1+1]
    if len(x) < 3:
        return lam[idx_pico]
    try:
        coeffs = np.polyfit(x, y, 2)
        x_peak = -coeffs[1] / (2 * coeffs[0])
        if lam[i0] <= x_peak <= lam[i1]:
            return x_peak
    except np.linalg.LinAlgError:
        pass
    return lam[idx_pico]


# ── Calibracion: ajuste de L y offset ────────────────────────────────────────

def calibrar(picos_nm_medidos, lambdas_laser_nm,
             L_ini=L_NOM_MM, offset_ini=0.0):
    """
    Minimiza el error cuadratico medio entre los picos medidos y las
    longitudes de onda conocidas de los laseres.

    picos_nm_medidos : lista de lambda_nm observados en el espectro
    lambdas_laser_nm : lista de lambda_nm de referencia (misma longitud)

    Devuelve (L_cal, offset_nm, rmse_nm).
    El offset se expresa en nm para ser independiente de MM_POR_PASO.
    """
    picos_ref = np.array(lambdas_laser_nm, dtype=float)
    picos_med = np.array(picos_nm_medidos,  dtype=float)

    if len(picos_ref) == 0:
        return L_ini, 0.0, float('nan')

    def residuos(params):
        # Un desplazamiento aditivo en nm sobre las lambdas medidas
        offset_nm = params[0]
        escala    = params[1]   # factor de escala sobre lambda (afina L implicito)
        lam_ajust = picos_med * escala + offset_nm
        return float(np.mean((lam_ajust - picos_ref) ** 2))

    res = minimize(residuos, x0=[0.0, 1.0],
                   method='Nelder-Mead',
                   options={'xatol': 0.01, 'fatol': 0.01, 'maxiter': 5000})

    offset_nm, escala = res.x
    lam_ajust = picos_med * escala + offset_nm
    rmse = float(np.sqrt(np.mean((lam_ajust - picos_ref) ** 2)))

    # Convertir escala a L efectivo: lambda ~ d*sin(arctan(y/L)) ≈ d*y/L para small angles
    # escala = L_nom / L_cal → L_cal = L_nom / escala
    L_cal = L_ini / escala if abs(escala) > 0.01 else L_ini

    return L_cal, offset_nm, rmse


# ── Flujo interactivo ─────────────────────────────────────────────────────────

def flujo_calibracion(lam, v_raw, v_dark):
    i_medida = np.maximum(v_raw - v_dark, 0.0)

    # Detectar picos
    idx_picos, lam_picos = detectar_picos(lam, i_medida, n_picos=6)
    if len(lam_picos) == 0:
        print('[WARN] No se detectaron picos. Verifica la señal.')
        return

    # Refinar posicion de cada pico
    lam_picos_refinados = np.array([
        refinar_pico(lam, i_medida, ip) for ip in idx_picos
    ])

    # ── Mostrar espectro con picos marcados ───────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(lam, i_medida, 'b-', linewidth=1.0, label='Espectro medido')
    for k, (lp, lpr) in enumerate(zip(lam_picos, lam_picos_refinados)):
        ax.axvline(lpr, color='red', linestyle='--', alpha=0.7)
        ax.annotate(
            f'P{k+1}\n{lpr:.1f} nm',
            xy=(lpr, i_medida[idx_picos[k]]),
            xytext=(lpr + 5, i_medida[idx_picos[k]] * 0.9),
            fontsize=8, color='red'
        )
    ax.set_xlabel('Longitud de onda nominal (nm)')
    ax.set_ylabel('ADC − dark')
    ax.set_title('Picos detectados — Asignar laseres de referencia')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.show(block=False)

    # ── Asignacion interactiva de picos a laseres ─────────────────────────────
    print('\nPicos detectados:')
    for k, lpr in enumerate(lam_picos_refinados):
        print(f'  P{k+1}: {lpr:.2f} nm (valor nominal del modelo)')

    print('\nLaseres disponibles:')
    for nombre, lnm in LASERES.items():
        print(f'  {nombre}: {lnm} nm')

    print('\nAsigna cada laser a un pico (ej: "rojo=P1 verde=P2") o "auto":')
    entrada = input('> ').strip().lower()

    asignaciones = {}  # pico_idx → lambda_laser
    if entrada == 'auto' or entrada == '':
        # Asignacion automatica: el pico mas cercano a cada laser
        print('[auto] Asignacion automatica...')
        for nombre, lref in LASERES.items():
            dists = np.abs(lam_picos_refinados - lref)
            mejor = int(np.argmin(dists))
            asignaciones[mejor] = lref
            print(f'  {nombre} ({lref} nm) → P{mejor+1} ({lam_picos_refinados[mejor]:.2f} nm)')
    else:
        for token in entrada.split():
            if '=' not in token:
                continue
            nombre, pico_str = token.split('=', 1)
            nombre = nombre.strip()
            pico_str = pico_str.strip().upper().lstrip('P')
            if nombre in LASERES and pico_str.isdigit():
                k = int(pico_str) - 1
                if 0 <= k < len(lam_picos_refinados):
                    asignaciones[k] = LASERES[nombre]

    if not asignaciones:
        print('[WARN] Sin asignaciones validas. Abortando calibracion.')
        return

    # ── Calibracion ──────────────────────────────────────────────────────────
    picos_med = [lam_picos_refinados[k] for k in sorted(asignaciones)]
    lams_ref  = [asignaciones[k]         for k in sorted(asignaciones)]

    L_cal, offset_nm, rmse = calibrar(picos_med, lams_ref)

    print(f'\n=== Resultados de calibracion ===')
    print(f'  L calibrado     : {L_cal:.2f} mm  (nominal: {L_NOM_MM:.1f} mm)')
    print(f'  Offset espectral: {offset_nm:+.2f} nm')
    print(f'  RMSE residual   : {rmse:.3f} nm')

    # Guardar JSON
    resultado = {
        'L_mm':       round(L_cal, 4),
        'offset_nm':  round(offset_nm, 4),
        'rmse_nm':    round(rmse, 4),
        'asignaciones': {
            f'P{k+1}': {'medido_nm': round(lam_picos_refinados[k], 3),
                         'referencia_nm': v}
            for k, v in asignaciones.items()
        }
    }
    with open(CAL_JSON, 'w', encoding='utf-8') as f:
        json.dump(resultado, f, indent=2, ensure_ascii=False)
    print(f'\nCalibracion guardada en: {CAL_JSON}')

    # ── Grafica comparativa ───────────────────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(8, 4))
    ax2.plot(lam, i_medida, 'b-', linewidth=1.0, label='Espectro (modelo nominal)')

    # Aplicar correccion de offset para mostrar el eje calibrado
    lam_cal = lam + offset_nm
    ax2.plot(lam_cal, i_medida, 'g-', linewidth=1.0,
             label=f'Espectro (eje calibrado, +{offset_nm:.1f} nm)')

    for nombre, lref in LASERES.items():
        ax2.axvline(lref, color='orange', linestyle=':', alpha=0.8,
                    label=f'Laser {nombre} ({lref} nm)')

    ax2.set_xlabel('Longitud de onda (nm)')
    ax2.set_ylabel('ADC − dark')
    ax2.set_title('Espectro antes/despues de calibracion')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ── Utilidad: aplicar calibracion guardada a un array de longitudes de onda ───

def aplicar_calibracion(lam_nm, ruta_json=CAL_JSON):
    """
    Aplica la calibracion guardada a un array de longitudes de onda.
    Devuelve lam_nm corregido.
    """
    try:
        with open(ruta_json, 'r', encoding='utf-8') as f:
            cal = json.load(f)
        offset_nm = cal.get('offset_nm', 0.0)
        return np.asarray(lam_nm) + offset_nm
    except FileNotFoundError:
        return np.asarray(lam_nm)


# ── Punto de entrada ──────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description='Calibracion del espectrometro con laseres')
    src = p.add_mutually_exclusive_group()
    src.add_argument('--csv',    metavar='RUTA', help='CSV de medicion previo')
    src.add_argument('--puerto', metavar='PORT', help='Puerto serial (captura en vivo)')
    src.add_argument('--demo',   action='store_true', help='Datos sinteticos de demo')
    return p.parse_args()


if __name__ == '__main__':
    args = _parse_args()

    if args.csv:
        print(f'Cargando {args.csv}...')
        lam, v_raw, v_dark = cargar_csv(args.csv)
    elif args.puerto:
        lam, v_raw, v_dark = capturar_serial(args.puerto)
    else:
        print('[DEMO] Usando datos sinteticos con picos en 532 nm y 650 nm.')
        lam, v_raw, v_dark = cargar_demo()

    if len(lam) == 0:
        sys.exit('Error: no se cargaron datos.')

    print(f'{len(lam)} puntos cargados  |  v_dark = {v_dark:.0f}')
    flujo_calibracion(lam, v_raw, v_dark)
