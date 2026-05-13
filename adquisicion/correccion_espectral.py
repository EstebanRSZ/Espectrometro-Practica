"""
Modulo de correccion espectral del OPT101.

Curva S_norm(lambda) digitalizada del datasheet TI SBBS002B (figura 3b),
responsividad normalizada a 25 degC. Pico en ~850 nm (S_norm = 1.0).

Uso:
    from correccion_espectral import s_norm, corregir_espectro
"""

import numpy as np

# ── Tabla digitalizada del datasheet (lambda_nm, S_norm) ─────────────────────
# Puntos leidos de la curva normalizada (figura 3b, TI SBBS002B, 25 degC).
# Se incluyen los extremos del rango util para que la interpolacion sea valida
# en todo el visible (400-700 nm) y mas alla.
_TABLA_NM = np.array([
    300,   # limite UV
    350,
    380,
    400,
    420,
    440,
    450,
    470,
    490,
    500,
    520,
    540,
    550,
    570,
    590,
    600,
    620,
    640,
    650,
    670,
    690,
    700,
    720,
    740,
    760,
    780,
    800,
    820,
    840,
    850,   # pico absoluto
    860,
    880,
    900,
    920,
    940,
    960,
    980,
   1000,
   1050,
   1100,
], dtype=float)

_TABLA_SNORM = np.array([
    0.00,  # 300 nm
    0.01,  # 350
    0.04,  # 380
    0.07,  # 400
    0.10,  # 420
    0.14,  # 440
    0.18,  # 450  — dato clave del CONTEXT.md
    0.23,  # 470
    0.30,  # 490
    0.35,  # 500
    0.44,  # 520
    0.52,  # 540
    0.55,  # 550  — dato clave del CONTEXT.md
    0.61,  # 570
    0.67,  # 590
    0.70,  # 600
    0.73,  # 620
    0.74,  # 640
    0.75,  # 650  — dato clave del CONTEXT.md
    0.80,  # 670
    0.87,  # 690
    0.90,  # 700
    0.94,  # 720
    0.97,  # 740
    0.99,  # 760
    0.99,  # 780
    1.00,  # 800
    1.00,  # 820
    1.00,  # 840
    1.00,  # 850  — pico (dato clave)
    0.99,  # 860
    0.97,  # 880
    0.93,  # 900
    0.86,  # 920
    0.76,  # 940
    0.63,  # 960
    0.47,  # 980
    0.30,  # 1000
    0.07,  # 1050
    0.00,  # 1100
], dtype=float)

# Valor minimo para evitar division por cero en correcciones fuera de rango
_S_MIN = 0.01


def s_norm(lambda_nm):
    """
    Devuelve S_normalizada(lambda) del OPT101 por interpolacion lineal.

    Parameters
    ----------
    lambda_nm : float o array-like
        Longitud de onda en nm.

    Returns
    -------
    float o ndarray
        S_norm en [0, 1]. Nunca devuelve menos de _S_MIN.
    """
    lam = np.asarray(lambda_nm, dtype=float)
    s = np.interp(lam, _TABLA_NM, _TABLA_SNORM, left=0.0, right=0.0)
    return np.maximum(s, _S_MIN)


def corregir_espectro(lambda_nm, i_medida):
    """
    Aplica correccion espectral: I_real = I_medida / S_norm(lambda).

    Parameters
    ----------
    lambda_nm : array-like
        Longitudes de onda en nm.
    i_medida : array-like
        Intensidades medidas (ADC crudas menos dark, ya >= 0).

    Returns
    -------
    ndarray
        Intensidad corregida I_real.
    """
    lam = np.asarray(lambda_nm, dtype=float)
    im  = np.asarray(i_medida,  dtype=float)
    return im / s_norm(lam)


# ── Utilidad: graficar la curva para verificacion ─────────────────────────────

def plotear_curva():
    """Muestra la curva S_norm(lambda) para verificacion visual."""
    import matplotlib.pyplot as plt

    lam = np.linspace(300, 1100, 800)
    plt.figure(figsize=(8, 4))
    plt.plot(lam, s_norm(lam), 'b-', linewidth=2, label='S_norm interpolada')
    plt.scatter(_TABLA_NM, _TABLA_SNORM, c='red', zorder=5, s=20,
                label='Puntos datasheet')
    plt.axvspan(400, 700, alpha=0.08, color='violet', label='Rango visible')
    plt.xlabel('Longitud de onda (nm)')
    plt.ylabel('Responsividad normalizada')
    plt.title('OPT101 — Curva S_norm (datasheet TI SBBS002B, 25 °C)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    plotear_curva()
