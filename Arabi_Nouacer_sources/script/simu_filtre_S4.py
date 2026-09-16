# -*- coding: utf-8 -*-
"""
simu_filtre_S4.py — Validation temporelle du filtre IIR Butterworth S4.

Complément du diagramme de Bode (generate_bode_png.py) :
1. Vérifie que les coefficients codés en dur dans my_func.c (FILTRE_B/FILTRE_A)
   correspondent bien à scipy.signal.butter(2, 10/(120/2)).
2. Porte à l'identique la fonction C my_filtre_iir() (forme directe II
   transposée, arithmétique float 32 bits) et la fait tourner sur des
   sinusoïdes de test.
3. Compare l'atténuation MESURÉE sur l'implémentation C à la courbe
   THÉORIQUE |H(f)| → preuve que le code embarqué réalise le filtre conçu.

Sortie : reponse_filtre_S4.png (à insérer dans les dossiers, savoir-faire
R23 « caractériser le filtre obtenu » et R31-34 procédure/rapport de test).

Usage : python simu_filtre_S4.py
"""

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------
# 1. Coefficients : valeurs EXACTES de my_func.c (lignes FILTRE_B / FILTRE_A)
# ----------------------------------------------------------------------------
FS = 120.0   # Hz, mode SPORT
FC = 10.0    # Hz, coupure -3 dB

B_C = np.array([0.04948996, 0.09897991, 0.04948996], dtype=np.float32)
A_C = np.array([1.00000000, -1.27963242, 0.47759225], dtype=np.float32)

b_ref, a_ref = signal.butter(2, FC / (FS / 2.0))
err_b = np.max(np.abs(B_C.astype(np.float64) - b_ref))
err_a = np.max(np.abs(A_C.astype(np.float64) - a_ref))
print(f"scipy butter(2, {FC}/{FS/2}) : b = {b_ref}")
print(f"                              a = {a_ref}")
print(f"Écart max coefficients C vs scipy : b {err_b:.2e}, a {err_a:.2e}")
assert err_b < 1e-7 and err_a < 1e-7, "Coefficients my_func.c non conformes !"

# ----------------------------------------------------------------------------
# 2. Portage à l'identique de my_filtre_iir() (DF2T, float 32 bits comme en C)
# ----------------------------------------------------------------------------
def my_filtre_iir_c(x, w):
    """Réplique fidèle de my_func.c::my_filtre_iir (états w[0], w[1])."""
    x = np.float32(x)
    y = np.float32(B_C[0] * x + w[0])
    w[0] = np.float32(B_C[1] * x - A_C[1] * y + w[1])
    w[1] = np.float32(B_C[2] * x - A_C[2] * y)
    return y

def filtrer(sig):
    w = [np.float32(0.0), np.float32(0.0)]
    return np.array([my_filtre_iir_c(x, w) for x in sig], dtype=np.float32)

# ----------------------------------------------------------------------------
# 3. Mesure d'atténuation sur sinusoïdes pures (régime établi)
# ----------------------------------------------------------------------------
def attenuation_mesuree(f_hz, n_per=40):
    n = int(round(n_per * FS / f_hz))
    t = np.arange(n) / FS
    x = np.sin(2 * np.pi * f_hz * t).astype(np.float32)
    y = filtrer(x)
    n_skip = n // 2                       # on écarte le transitoire
    gain = np.max(np.abs(y[n_skip:])) / np.max(np.abs(x[n_skip:]))
    return 20 * np.log10(gain)

freqs_test = np.array([1, 2, 3, 5, 8, 10, 15, 20, 30, 40, 50])
att_mes = np.array([attenuation_mesuree(f) for f in freqs_test])

w, h = signal.freqz(b_ref, a_ref, worN=4096, fs=FS)
h_db = 20 * np.log10(np.maximum(np.abs(h), 1e-12))

print("\n f (Hz) | mesuré C (dB) | théorie (dB)")
for f, a in zip(freqs_test, att_mes):
    a_th = h_db[np.argmin(np.abs(w - f))]
    print(f"  {f:5.0f} | {a:13.2f} | {a_th:11.2f}")

# ----------------------------------------------------------------------------
# 4. Signal d'illustration : geste sportif 2 Hz + vibration carres de ski 30 Hz
# ----------------------------------------------------------------------------
DUREE = 2.0
t = np.arange(int(DUREE * FS)) / FS
utile = 0.8 * np.sin(2 * np.pi * 2.0 * t)            # accélération utile (2 Hz)
vib = 0.5 * np.sin(2 * np.pi * 30.0 * t)             # vibration mécanique (30 Hz)
rng = np.random.default_rng(42)
bruit = 0.05 * rng.standard_normal(len(t))           # bruit capteur
x_tot = (utile + vib + bruit).astype(np.float32)
y_tot = filtrer(x_tot)

# ----------------------------------------------------------------------------
# 5. Figure
# ----------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 6.5))
fig.suptitle("Validation temporelle du filtre IIR Butterworth — implémentation C de my_filtre_iir()",
             fontsize=11, fontweight="bold")

ax1.plot(t, x_tot, color="#b0b0b0", lw=0.8,
         label="Entrée brute : utile 2 Hz + vibration 30 Hz + bruit")
ax1.plot(t, y_tot, color="#1f77b4", lw=1.8,
         label="Sortie filtrée (my_filtre_iir, float 32 bits)")
ax1.plot(t, utile, color="#d62728", lw=1.0, ls="--",
         label="Composante utile seule (référence 2 Hz)")
ax1.set_xlabel("Temps (s)")
ax1.set_ylabel("Accélération (g)")
ax1.set_title(f"Signal composite échantillonné à Fs = {FS:.0f} Hz (mode SPORT)", fontsize=9)
ax1.grid(True, alpha=0.3)
ax1.legend(fontsize=7, loc="upper right")

ax2.plot(w, h_db, color="#1f77b4", lw=1.5, label="Théorie scipy |H(f)| (butter ordre 2)")
ax2.plot(freqs_test, att_mes, "o", color="#d62728", ms=6, mfc="none", mew=1.6,
         label="Mesure sur le portage exact du code C")
ax2.axvline(FC, color="red", ls="--", lw=0.9, alpha=0.6)
ax2.axhline(-3.0, color="gray", ls=":", lw=0.9)
ax2.annotate("fc = 10 Hz (−3 dB)", xy=(FC, -3), xytext=(12, 2), fontsize=8, color="red")
a30 = att_mes[list(freqs_test).index(30)]
a50 = att_mes[list(freqs_test).index(50)]
ax2.annotate(f"{a30:.1f} dB à 30 Hz", xy=(30, a30), xytext=(31, a30 + 6), fontsize=8,
             arrowprops=dict(arrowstyle="->", lw=0.8))
ax2.annotate(f"{a50:.1f} dB à 50 Hz", xy=(50, a50), xytext=(40, a50 + 9), fontsize=8,
             arrowprops=dict(arrowstyle="->", lw=0.8))
ax2.set_xlim(0, 60)
ax2.set_ylim(-60, 6)
ax2.set_xlabel("Fréquence (Hz)")
ax2.set_ylabel("Gain (dB)")
ax2.set_title("Atténuation mesurée (implémentation C) vs réponse théorique", fontsize=9)
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=7, loc="lower left")

fig.tight_layout()
fig.savefig("reponse_filtre_S4.png", dpi=150)
print("\nFigure écrite : reponse_filtre_S4.png")
