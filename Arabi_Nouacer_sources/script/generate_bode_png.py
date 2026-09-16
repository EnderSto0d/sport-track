"""
generate_bode_png.py
---------------------
Génération automatique du diagramme de Bode du filtre numérique IIR
utilisé dans SportTrack S4 (post-traitement des données MPU6050).

Caractéristiques du filtre :
    - Type       : Butterworth passe-bas
    - Ordre      : 2
    - Fs         : 120 Hz  (fréquence d'échantillonnage en mode SPORT)
    - fc         : 10 Hz   (fréquence de coupure à -3 dB)
    - Méthode    : scipy.signal.butter (transformée bilinéaire)

Justification de fc = 10 Hz :
    - Les accélérations sportives utiles (foulée, virage, impact ski)
      sont concentrées en dessous de 3 Hz.
    - Les vibrations parasites (chocs mécaniques sur les carres de ski,
      bruit de fixation, bruit haute fréquence capteur) sont au-delà de
      10 Hz.
    - Ce filtre est COMPLEMENTAIRE du filtre analogique RC passe-bas
      50 Hz déjà présent en amont sur l'ADXL335 (anti-repliement).

Usage :
    python generate_bode_png.py
    -> Génère script/bode_filtre_S4.png
    -> Affiche les coefficients b et a formatés pour copier-coller en C.
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Backend non interactif (pas de GUI)
import matplotlib.pyplot as plt
from scipy import signal

# =========================================================================
# Paramètres filtre
# =========================================================================
FS      = 120.0   # Fréquence d'échantillonnage en Hz (mode SPORT)
FC      = 10.0    # Fréquence de coupure en Hz
ORDER   = 2       # Ordre du filtre Butterworth
BTYPE   = "low"   # Passe-bas

# =========================================================================
# Calcul des coefficients (transformée bilinéaire, forme b/a classique)
# =========================================================================
b, a = signal.butter(ORDER, FC, btype=BTYPE, fs=FS)

# =========================================================================
# Réponse en fréquence (haute résolution pour Bode)
# =========================================================================
w, h = signal.freqz(b, a, worN=8192, fs=FS)

mag_db = 20.0 * np.log10(np.abs(h) + 1e-12)
phase_deg = np.degrees(np.unwrap(np.angle(h)))

# Recherche de la fréquence réelle à -3 dB (contrôle qualité)
idx_m3 = int(np.argmin(np.abs(mag_db - (-3.0))))
fc_mesuree = w[idx_m3]

# =========================================================================
# Tracé Bode : module + phase
# =========================================================================
fig, (ax_mag, ax_phase) = plt.subplots(
    2, 1, figsize=(9, 6.5), sharex=True, dpi=150
)
fig.patch.set_facecolor("white")

# --- Module ---
ax_mag.semilogx(w, mag_db, color="#1f4e79", linewidth=2.0,
                label="Module |H(f)|")
ax_mag.axvline(FC, color="#c0392b", linestyle="--", linewidth=1.4,
               label=f"fc = {FC:.1f} Hz  (−3 dB)")
ax_mag.axhline(-3.0, color="#808080", linestyle=":", linewidth=1.0)
ax_mag.set_ylabel("Module (dB)")
ax_mag.set_title(
    f"Diagramme de Bode — Filtre Butterworth passe-bas "
    f"(ordre {ORDER}, Fs = {FS:.0f} Hz, fc = {FC:.0f} Hz)",
    fontweight="bold"
)
ax_mag.grid(True, which="major", ls="-",  alpha=0.7)
ax_mag.grid(True, which="minor", ls=":",  alpha=0.4)
ax_mag.set_ylim([-80, 5])
ax_mag.legend(loc="lower left")

# --- Phase ---
ax_phase.semilogx(w, phase_deg, color="#117a3d", linewidth=2.0,
                  label="Phase ∠H(f)")
ax_phase.axvline(FC, color="#c0392b", linestyle="--", linewidth=1.4)
ax_phase.set_xlabel("Fréquence (Hz) — échelle logarithmique")
ax_phase.set_ylabel("Phase (°)")
ax_phase.grid(True, which="major", ls="-",  alpha=0.7)
ax_phase.grid(True, which="minor", ls=":",  alpha=0.4)
ax_phase.legend(loc="lower left")

# Limites de l'axe X : on commence à 0.1 Hz pour lisibilité
ax_phase.set_xlim([0.1, FS / 2.0])

fig.tight_layout()

# =========================================================================
# Sauvegarde PNG
# =========================================================================
out_dir = os.path.dirname(os.path.abspath(__file__))
out_path = os.path.join(out_dir, "bode_filtre_S4.png")
fig.savefig(out_path, dpi=150, facecolor="white")
plt.close(fig)

# =========================================================================
# Affichage des coefficients formatés pour copier-coller en C
# =========================================================================
def _format_c_array(name, values):
    inner = ", ".join(f"{v: .8f}f" for v in values)
    return f"static const float {name}[{len(values)}] = {{ {inner} }};"

print("=" * 72)
print("Filtre Butterworth passe-bas")
print(f"  Ordre       : {ORDER}")
print(f"  Fs          : {FS:.1f} Hz")
print(f"  fc          : {FC:.1f} Hz")
print(f"  fc mesurée à -3 dB (contrôle) : {fc_mesuree:.3f} Hz")
print("=" * 72)
print("Coefficients NumPy (forme directe, y(n) = ...):")
print(f"  b = {b}")
print(f"  a = {a}")
print()
print("Copier-coller dans my_func.c :")
print("/* ----- Filtre IIR Butterworth passe-bas -----")
print(f"   Ordre : {ORDER}  |  Fs = {FS:.0f} Hz  |  fc = {FC:.0f} Hz")
print( "   Calcul : scripts/generate_bode_png.py (scipy.signal.butter)")
print( "   Forme : directe II transposée, états w[] persistants")
print( "   -------------------------------------------- */")
print(_format_c_array("FILTRE_B", b))
print(_format_c_array("FILTRE_A", a))
print()
print(f"[OK] PNG généré : {out_path}")
