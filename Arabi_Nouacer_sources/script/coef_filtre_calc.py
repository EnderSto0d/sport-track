import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class FilterDesignerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Outil de Conception de Filtres Numériques")
        self.root.geometry("1100x700")
        self.root.minsize(900, 600)
        
        # Configuration du style
        style = ttk.Style()
        style.theme_use('clam')
        
        # --- Variables de contrôle Tkinter ---
        self.filter_type_var = tk.StringVar(value="Passe-bas")
        self.fs_var = tk.StringVar(value="44100")
        self.order_var = tk.StringVar(value="4")
        self.fc1_var = tk.StringVar(value="1000")
        self.fc2_var = tk.StringVar(value="5000")
        
        # --- Construction de l'interface ---
        self.create_widgets()
        
        # Initialisation de l'état de l'interface et premier tracé
        self.update_ui_state()
        self.design_filter()

    def create_widgets(self):
        # Cadre principal divisé en deux : Contrôles (gauche) et Graphique (droite)
        main_paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned_window.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # --- CADRE DE GAUCHE : Contrôles et Coefficients ---
        left_frame = ttk.Frame(main_paned_window, width=350)
        main_paned_window.add(left_frame, weight=0)
        
        # 1. Section Paramètres
        params_frame = ttk.LabelFrame(left_frame, text="Paramètres du Filtre")
        params_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Type de filtre
        ttk.Label(params_frame, text="Type de filtre :").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        type_cb = ttk.Combobox(params_frame, textvariable=self.filter_type_var, state="readonly", width=18)
        type_cb['values'] = ("Passe-bas", "Passe-haut", "Passe-bande", "Coupe-bande")
        type_cb.grid(row=0, column=1, padx=5, pady=5)
        type_cb.bind("<<ComboboxSelected>>", self.update_ui_state)
        
        # Fréquence d'échantillonnage
        ttk.Label(params_frame, text="Fréq. d'échantillonnage (Hz) :").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(params_frame, textvariable=self.fs_var, width=20).grid(row=1, column=1, padx=5, pady=5)
        
        # Ordre du filtre
        ttk.Label(params_frame, text="Ordre du filtre :").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(params_frame, textvariable=self.order_var, width=20).grid(row=2, column=1, padx=5, pady=5)
        
        # Fréquence de coupure 1
        ttk.Label(params_frame, text="Fréq. de coupure 1 (Hz) :").grid(row=3, column=0, sticky=tk.W, padx=5, pady=5)
        ttk.Entry(params_frame, textvariable=self.fc1_var, width=20).grid(row=3, column=1, padx=5, pady=5)
        
        # Fréquence de coupure 2 (pour passe-bande et coupe-bande)
        ttk.Label(params_frame, text="Fréq. de coupure 2 (Hz) :").grid(row=4, column=0, sticky=tk.W, padx=5, pady=5)
        self.fc2_entry = ttk.Entry(params_frame, textvariable=self.fc2_var, width=20)
        self.fc2_entry.grid(row=4, column=1, padx=5, pady=5)
        
        # Bouton de calcul
        calc_btn = ttk.Button(left_frame, text="Calculer et Tracer", command=self.design_filter)
        calc_btn.pack(fill=tk.X, padx=5, pady=15)
        
        # 2. Section Coefficients
        coeff_frame = ttk.LabelFrame(left_frame, text="Coefficients du Filtre (b, a)")
        coeff_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Zone de texte avec défilement pour afficher les tableaux Numpy
        self.coeff_text = tk.Text(coeff_frame, wrap=tk.WORD, width=40, height=15, font=("Consolas", 10))
        scrollbar = ttk.Scrollbar(coeff_frame, command=self.coeff_text.yview)
        self.coeff_text.configure(yscrollcommand=scrollbar.set)
        
        self.coeff_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5,0), pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0,5), pady=5)
        
        # --- CADRE DE DROITE : Tracé Matplotlib ---
        right_frame = ttk.Frame(main_paned_window)
        main_paned_window.add(right_frame, weight=1)
        
        self.figure, self.ax = plt.subplots(figsize=(6, 5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Ajout de la barre d'outils Matplotlib (zoom, sauvegarde, etc.)
        toolbar = NavigationToolbar2Tk(self.canvas, right_frame)
        toolbar.update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def update_ui_state(self, event=None):
        """Active ou désactive la seconde fréquence de coupure selon le type de filtre."""
        f_type = self.filter_type_var.get()
        if f_type in ["Passe-bande", "Coupe-bande"]:
            self.fc2_entry.config(state="normal")
        else:
            self.fc2_entry.config(state="disabled")

    def design_filter(self):
        """Récupère les entrées, valide, calcule les coefficients et met à jour l'interface."""
        # 1. Validation des entrées de l'utilisateur
        try:
            fs = float(self.fs_var.get())
            order = int(self.order_var.get())
            fc1 = float(self.fc1_var.get())
            
            if fs <= 0 or order <= 0 or fc1 <= 0:
                raise ValueError("Les valeurs doivent être strictement positives.")
                
            nyquist = 0.5 * fs
            if fc1 >= nyquist:
                raise ValueError(f"La fréquence de coupure doit être inférieure à la fréquence de Nyquist ({nyquist} Hz).")
                
            f_type = self.filter_type_var.get()
            
            # Gestion des fréquences selon le type de filtre
            if f_type in ["Passe-bande", "Coupe-bande"]:
                fc2 = float(self.fc2_var.get())
                if fc2 <= fc1:
                    raise ValueError("La fréquence de coupure 2 doit être supérieure à la fréquence de coupure 1.")
                if fc2 >= nyquist:
                    raise ValueError(f"La fréquence de coupure 2 doit être inférieure à la fréquence de Nyquist ({nyquist} Hz).")
                Wn = [fc1 / nyquist, fc2 / nyquist]
                btype = 'bandpass' if f_type == "Passe-bande" else 'bandstop'
            else:
                Wn = fc1 / nyquist
                btype = 'lowpass' if f_type == "Passe-bas" else 'highpass'
                
        except ValueError as e:
            messagebox.showerror("Erreur de Saisie", str(e))
            return
            
        # 2. Calcul des coefficients du filtre (Butterworth)
        try:
            b, a = signal.butter(order, Wn, btype=btype)
        except Exception as e:
            messagebox.showerror("Erreur Mathématique", f"Impossible de concevoir le filtre : {e}")
            return
            
        # 3. Affichage des coefficients
        self.display_coefficients(b, a)
        
        # 4. Tracé de la réponse en fréquence
        self.plot_response(b, a, fs, f_type)

    def display_coefficients(self, b, a):
        """Affiche les tableaux numpy et génère l'équation de récurrence y(n)."""
        self.coeff_text.config(state="normal")
        self.coeff_text.delete("1.0", tk.END)
        
        np.set_printoptions(precision=6, suppress=True)
        
        # 1. Affichage brut des tableaux
        self.coeff_text.insert(tk.END, "--- Coefficients NumPy ---\n")
        self.coeff_text.insert(tk.END, f"b (Numérateur)   : {b}\n")
        self.coeff_text.insert(tk.END, f"a (Dénominateur) : {a}\n\n")
        
        # 2. Construction de la formule y(n)
        self.coeff_text.insert(tk.END, "--- Équation de Récurrence ---\n")
        eq_str = "y(n) ="
        
        # Ajout des termes x(n) liés aux coefficients 'b'
        for i, val in enumerate(b):
            if abs(val) < 1e-10: continue  # Ignorer les zéros
            
            sign = " + " if val >= 0 else " - "
            # Ne pas mettre de "+" devant le tout premier terme s'il est positif
            if i == 0 and val >= 0: 
                sign = " "
                
            index = "n" if i == 0 else f"n-{i}"
            eq_str += f"{sign}{abs(val):.6g} * x({index})"
            
        # Ajout des termes y(n) liés aux coefficients 'a'
        # On commence à 1 car a[0] représente y(n), et on inverse le signe
        for i in range(1, len(a)):
            val = -a[i] # Inversion du signe pour passer de l'autre côté du "="
            if abs(val) < 1e-10: continue
            
            sign = " + " if val >= 0 else " - "
            eq_str += f"{sign}{abs(val):.6g} * y(n-{i})"
            
        # Affichage du résultat final dans la zone de texte
        self.coeff_text.insert(tk.END, f"{eq_str}\n")
        
        self.coeff_text.config(state="disabled")

    def plot_response(self, b, a, fs, f_type):
        """Calcule et trace la magnitude avec un axe des X en échelle logarithmique."""
        # Calcul de la réponse en fréquence
        w, h = signal.freqz(b, a, worN=8000)
        
        # Conversion de la pulsation normalisée vers la fréquence (Hz)
        freqs = w * fs / (2 * np.pi)
        
        # Conversion de la magnitude en décibels (dB)
        mag_db = 20 * np.log10(np.abs(h) + 1e-10)
        
        # Mise à jour du tracé Matplotlib
        self.ax.clear()
        
        # Utilisation de semilogx pour l'échelle logarithmique sur l'axe des abscisses
        self.ax.semilogx(freqs, mag_db, color='#1f77b4', linewidth=2)
        
        self.ax.set_title(f"Réponse en Fréquence - Filtre {f_type} (Ordre {int(self.order_var.get())})", fontweight='bold')
        self.ax.set_xlabel("Fréquence (Hz) - Échelle Logarithmique")
        self.ax.set_ylabel("Magnitude (dB)")
        
        # Grille majeure (lignes continues) et mineure (lignes pointillées pour le log)
        self.ax.grid(True, which="major", ls="-", alpha=0.8)
        self.ax.grid(True, which="minor", ls=":", alpha=0.5)
        
        # Limites visuelles : On commence à 0.01 Hz pour éviter log(0)
        self.ax.set_xlim([0.01, fs / 2])
        self.ax.set_ylim([-100, 5])
        
        self.figure.tight_layout()
        self.canvas.draw()

if __name__ == "__main__":
    # Lancement de l'application Tkinter
    root = tk.Tk()
    app = FilterDesignerApp(root)
    root.mainloop()