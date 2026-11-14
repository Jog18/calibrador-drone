## Interfaz 2 - Plataforma de pruebas multirrotor (ARMTOP)
## Basada en interface.py de Mario Aguilera Ruiz (CICESE 2024)
##
## Protocolo serie (compatible con FIN_ESP_PC.ino / FIN_ESP_DRON.ino actualizados):
##   PC -> ESP32 : "arm,throttle,roll_d,pitch_d,P,I,D,yaw_cmd\n"
##   ESP32 -> PC : "roll,pitch,yaw\n"   (yaw opcional: con 2 valores sigue funcionando)
##
## Mejoras respecto a interface.py:
##   - Tema oscuro moderno y diseño organizado en paneles
##   - Lectura serie en hilo separado con cola (la GUI nunca se congela ni
##     se toca desde otro hilo) y gráfica actualizada a tasa fija
##   - Conectar / desconectar, refrescar puertos y elegir baudrate
##   - Botón ARMAR/DESARMAR con color, confirmación y PARO DE EMERGENCIA
##     (tecla Esc o barra espaciadora)
##   - Lecturas numéricas de roll/pitch en vivo y error respecto a la referencia
##   - Dos gráficas (roll y pitch) con su referencia, pausa y limpieza
##   - Registro CSV opcional, un archivo por sesión con nombre con fecha
##   - Envío periódico de comandos (keep-alive) y sin errores si no hay conexión
##   - Modo SIMULADOR para probar la interfaz sin hardware
##   - Yaw: lectura en vivo, gráfica y comando de giro (yaw_cmd)
##   - Reenvío de roll/pitch/yaw por UDP al visor 3D de la 2da PC

import csv
import os
import math
import queue
import random
import socket
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ----------------------------------------------------------------------------
# Configuración
# ----------------------------------------------------------------------------
BAUDRATES = ["9600", "57600", "115200", "230400"]
DEFAULT_BAUD = "115200"
SIM_PORT = "SIMULADOR"
SAMPLES = 200            # muestras visibles en la gráfica
PLOT_PERIOD_MS = 50      # refresco de la gráfica (~20 fps)
KEEPALIVE_MS = 200       # reenvío periódico del comando actual
THROTTLE_MAX = 60
ANGLE_MAX = 50
PID_DEFAULTS = (22, 20, 30)
PID_MAX = 200
YAW_CMD_MAX = 50         # rango del comando de yaw (-50..50, como roll/pitch)

# Envío de los ángulos a la 2da PC (visor 3D) por UDP. Pon "" para desactivar.
PC2_IP = ""              # vacío = desactivado (el visor usa la 3ra ESP). Para UDP: IP de la 2da PC
PC2_PORT = 5005

COLORS = {
    "bg": "#1e1f26",
    "panel": "#282a36",
    "fg": "#e6e6e6",
    "muted": "#9aa0b4",
    "accent": "#4fc3f7",
    "ok": "#50fa7b",
    "warn": "#f1fa8c",
    "danger": "#ff5555",
    "roll": "#4fc3f7",
    "pitch": "#ff79c6",
    "yaw": "#bd93f9",
    "grid": "#3a3d4d",
}


# ----------------------------------------------------------------------------
# Comunicación serie
# ----------------------------------------------------------------------------
class SerialLink:
    """Maneja el puerto serie (o el simulador) en un hilo de lectura."""

    def __init__(self, rx_queue):
        self.rx_queue = rx_queue
        self.ser = None
        self.simulated = False
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._sim_cmd = (0, 0, 0, 0, 0)  # arm, thr, rolld, pitchd, yawd

    @property
    def connected(self):
        return self._running

    def open(self, port, baud):
        self.close()
        if port == SIM_PORT:
            self.simulated = True
        else:
            self.simulated = False
            self.ser = serial.Serial(port, int(baud), timeout=0.2)
        self._running = True
        target = self._sim_loop if self.simulated else self._read_loop
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def write(self, arm, thr, rolld, pitchd, p, i, d, yawd=0):
        if not self._running:
            return False
        if self.simulated:
            self._sim_cmd = (arm, thr, rolld, pitchd, yawd)
            return True
        msg = f"{arm},{thr},{rolld},{pitchd},{p},{i},{d},{yawd}\n"
        try:
            with self._lock:
                self.ser.write(msg.encode())
            return True
        except Exception as e:
            self.rx_queue.put(("error", str(e)))
            self._running = False
            return False

    def _read_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.rx_queue.put(("error", str(e)))
                self._running = False
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                roll, pitch = float(parts[0]), float(parts[1])
            except ValueError:
                continue  # p.ej. "ESPNow Init Success"
            try:
                yaw = float(parts[2]) if len(parts) > 2 else None
            except ValueError:
                yaw = None
            self.rx_queue.put(("data", roll, pitch, yaw))

    def _sim_loop(self):
        """Simula un dron de primer orden que sigue la referencia."""
        roll = pitch = yaw = 0.0
        t0 = time.time()
        while self._running:
            arm, thr, rolld, pitchd, yawd = self._sim_cmd
            t = time.time() - t0
            if arm:
                roll += 0.15 * (rolld - roll) + random.gauss(0, 0.6) + 0.8 * math.sin(3 * t)
                pitch += 0.15 * (pitchd - pitch) + random.gauss(0, 0.6) + 0.8 * math.cos(2 * t)
                yaw = (yaw + 0.035 * yawd + 180) % 360 - 180
            else:
                roll *= 0.9
                pitch *= 0.9
            self.rx_queue.put(("data", round(roll), round(pitch), round(yaw)))
            time.sleep(0.07)


# ----------------------------------------------------------------------------
# Interfaz gráfica
# ----------------------------------------------------------------------------
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Plataforma de pruebas multirrotor — Interfaz 2")
        self.root.configure(bg=COLORS["bg"])
        self.root.minsize(1100, 760)

        self.rx_queue = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if PC2_IP else None

        self.armed = False
        self.paused = False
        self.rx_count = 0
        self.rx_count_last = 0
        self.rate_t = time.time()

        self.csv_file = None
        self.csv_writer = None

        self.t_roll = deque(maxlen=SAMPLES)
        self.t_pitch = deque(maxlen=SAMPLES)
        self.t_rolld = deque(maxlen=SAMPLES)
        self.t_pitchd = deque(maxlen=SAMPLES)
        self.t_yaw = deque(maxlen=SAMPLES)

        self._setup_style()
        self._build_layout()
        self._bind_keys()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(PLOT_PERIOD_MS, self._poll)
        self.root.after(KEEPALIVE_MS, self._keepalive)
        self.root.after(1000, self._update_rate)

    # ------------------------------------------------------------------ estilo
    def _setup_style(self):
        s = ttk.Style(self.root)
        s.theme_use("clam")
        c = COLORS
        s.configure(".", background=c["bg"], foreground=c["fg"], fieldbackground=c["panel"],
                    font=("Segoe UI", 10))
        s.configure("Panel.TFrame", background=c["panel"])
        s.configure("Panel.TLabelframe", background=c["panel"], bordercolor=c["grid"], relief="solid")
        s.configure("Panel.TLabelframe.Label", background=c["panel"], foreground=c["accent"],
                    font=("Segoe UI", 11, "bold"))
        s.configure("Panel.TLabel", background=c["panel"], foreground=c["fg"])
        s.configure("Muted.TLabel", background=c["panel"], foreground=c["muted"], font=("Segoe UI", 9))
        s.configure("Value.TLabel", background=c["panel"], foreground=c["fg"],
                    font=("Consolas", 12, "bold"), width=6, anchor="e")
        s.configure("Big.TLabel", background=c["panel"], font=("Consolas", 22, "bold"))
        s.configure("Status.TLabel", background=c["bg"], foreground=c["muted"], font=("Segoe UI", 9))
        s.configure("TButton", background=c["grid"], foreground=c["fg"], borderwidth=0, padding=6)
        s.map("TButton", background=[("active", "#44475a"), ("disabled", c["panel"])],
              foreground=[("disabled", c["muted"])])
        s.configure("Accent.TButton", background=c["accent"], foreground="#000")
        s.map("Accent.TButton", background=[("active", "#81d4fa")])
        s.configure("Arm.TButton", background=c["ok"], foreground="#000",
                    font=("Segoe UI", 12, "bold"), padding=5)
        s.map("Arm.TButton", background=[("active", "#8affa5"), ("disabled", c["grid"])])
        s.configure("Disarm.TButton", background=c["warn"], foreground="#000",
                    font=("Segoe UI", 12, "bold"), padding=5)
        s.map("Disarm.TButton", background=[("active", "#f8ffb8")])
        s.configure("Stop.TButton", background=c["danger"], foreground="#fff",
                    font=("Segoe UI", 12, "bold"), padding=5)
        s.map("Stop.TButton", background=[("active", "#ff7b7b")])
        s.configure("TCombobox", fieldbackground=c["bg"], background=c["grid"], foreground=c["fg"],
                    arrowcolor=c["fg"])
        s.map("TCombobox", fieldbackground=[("readonly", c["bg"])], foreground=[("readonly", c["fg"])])
        s.configure("TSpinbox", fieldbackground=c["bg"], foreground=c["fg"], arrowcolor=c["fg"],
                    background=c["grid"])
        s.configure("Horizontal.TScale", background=c["panel"], troughcolor=c["bg"])
        s.configure("TCheckbutton", background=c["panel"], foreground=c["fg"])
        s.map("TCheckbutton", background=[("active", c["panel"])])
        self.root.option_add("*TCombobox*Listbox.background", c["panel"])
        self.root.option_add("*TCombobox*Listbox.foreground", c["fg"])

    # ----------------------------------------------------------------- layout
    def _build_layout(self):
        self._build_statusbar()  # primero, para que nunca quede oculta
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="ns", padx=(0, 10))
        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_connection(left)
        self._build_control(left)
        self._build_pid(left)
        self._build_logging(left)
        self._build_readouts(right)
        self._build_plot(right)

    def _panel(self, parent, title):
        f = ttk.LabelFrame(parent, text=f" {title} ", style="Panel.TLabelframe", padding=8)
        f.pack(fill="x", pady=(0, 6))
        return f

    def _build_connection(self, parent):
        f = self._panel(parent, "Conexión")
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="Puerto", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.combo_ports = ttk.Combobox(f, state="readonly", width=14)
        self.combo_ports.grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(f, text="⟳", width=3, command=self.refresh_ports).grid(row=0, column=2)

        ttk.Label(f, text="Baudios", style="Panel.TLabel").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.combo_baud = ttk.Combobox(f, state="readonly", width=14, values=BAUDRATES)
        self.combo_baud.set(DEFAULT_BAUD)
        self.combo_baud.grid(row=1, column=1, sticky="ew", padx=5, pady=(6, 0))

        self.btn_connect = ttk.Button(f, text="Conectar", style="Accent.TButton", command=self.toggle_connection)
        self.btn_connect.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 4))

        self.lbl_conn = tk.Label(f, text="● Desconectado", bg=COLORS["panel"], fg=COLORS["danger"],
                                 font=("Segoe UI", 10, "bold"))
        self.lbl_conn.grid(row=3, column=0, columnspan=3, sticky="w")

    def _build_control(self, parent):
        f = self._panel(parent, "Control del dron")
        f.columnconfigure(1, weight=1)

        self.btn_arm = ttk.Button(f, text="ARMAR", style="Arm.TButton", command=self.toggle_arm)
        self.btn_arm.grid(row=0, column=0, columnspan=3, sticky="ew")
        ttk.Button(f, text="PARO DE EMERGENCIA  (Esc)", style="Stop.TButton",
                   command=self.emergency_stop).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 4))

        self.var_thr = tk.IntVar(value=0)
        self.var_roll = tk.IntVar(value=0)
        self.var_pitch = tk.IntVar(value=0)
        self.var_yaw = tk.IntVar(value=0)
        self._slider(f, 2, "Throttle (%)", self.var_thr, 0, THROTTLE_MAX)
        self._slider(f, 4, "Roll ref (°)", self.var_roll, -ANGLE_MAX, ANGLE_MAX, center=True)
        self._slider(f, 6, "Pitch ref (°)", self.var_pitch, -ANGLE_MAX, ANGLE_MAX, center=True)
        self._slider(f, 8, "Yaw cmd (giro)", self.var_yaw, -YAW_CMD_MAX, YAW_CMD_MAX, center=True)

    def _slider(self, parent, row, text, var, lo, hi, center=False):
        ttk.Label(parent, text=text, style="Panel.TLabel").grid(row=row, column=0, columnspan=2, sticky="w",
                                                                  pady=(4, 0))
        val = ttk.Label(parent, text="0", style="Value.TLabel")
        val.grid(row=row, column=2, sticky="e")

        def on_move(_=None):
            v = int(round(float(scale.get())))
            if var.get() != v:
                var.set(v)
            val.config(text=str(v))
            self.send_values()

        scale = ttk.Scale(parent, from_=lo, to=hi, orient="horizontal", length=220, command=on_move)
        scale.grid(row=row + 1, column=0, columnspan=2, sticky="ew")
        var.trace_add("write", lambda *_: (scale.set(var.get()), val.config(text=str(var.get()))))
        if center:
            ttk.Button(parent, text="0", width=3, command=lambda: (var.set(0), self.send_values())).grid(
                row=row + 1, column=2, sticky="e")

    def _build_pid(self, parent):
        f = self._panel(parent, "Sintonización PID (Roll)")
        self.var_p = tk.IntVar(value=PID_DEFAULTS[0])
        self.var_i = tk.IntVar(value=PID_DEFAULTS[1])
        self.var_d = tk.IntVar(value=PID_DEFAULTS[2])
        for col, (name, var) in enumerate((("P", self.var_p), ("I", self.var_i), ("D", self.var_d))):
            ttk.Label(f, text=name, style="Panel.TLabel", font=("Segoe UI", 10, "bold")).grid(
                row=0, column=col, padx=6)
            sp = ttk.Spinbox(f, from_=0, to=PID_MAX, width=6, textvariable=var, justify="center",
                             command=self.send_values)
            sp.grid(row=1, column=col, padx=6, pady=(2, 0))
            sp.bind("<Return>", lambda e: self.send_values())
            sp.bind("<FocusOut>", lambda e: self.send_values())
        ttk.Button(f, text="↺ Reset", command=self.reset_pid).grid(row=1, column=3, padx=(4, 0))

    def _build_logging(self, parent):
        f = self._panel(parent, "Registro de datos")
        self.var_log = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Guardar en CSV", variable=self.var_log,
                        command=self.toggle_logging).grid(row=0, column=0, sticky="w")
        ttk.Button(f, text="Carpeta…", command=self.choose_log_dir).grid(row=0, column=1, sticky="e", padx=(8, 0))
        self.log_dir = "."
        self.lbl_log = ttk.Label(f, text="Sin registro", style="Muted.TLabel", wraplength=230)
        self.lbl_log.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def _build_readouts(self, parent):
        f = ttk.Frame(parent, style="Panel.TFrame", padding=10)
        f.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        self.readouts = {}
        items = (("roll", "Roll", COLORS["roll"]), ("pitch", "Pitch", COLORS["pitch"]), ("yaw", "Yaw", COLORS["yaw"]),
                 ("eroll", "Error roll", COLORS["muted"]), ("epitch", "Error pitch", COLORS["muted"]))
        for i, (key, title, color) in enumerate(items):
            f.columnconfigure(i, weight=1)
            box = ttk.Frame(f, style="Panel.TFrame")
            box.grid(row=0, column=i, sticky="ew")
            ttk.Label(box, text=title, style="Muted.TLabel").pack()
            lbl = ttk.Label(box, text="—", style="Big.TLabel", foreground=color)
            lbl.pack()
            self.readouts[key] = lbl

        bar = ttk.Frame(f, style="Panel.TFrame")
        bar.grid(row=0, column=len(items), sticky="e", padx=(10, 0))
        self.btn_pause = ttk.Button(bar, text="⏸ Pausar", command=self.toggle_pause)
        self.btn_pause.pack(fill="x")
        ttk.Button(bar, text="🗑 Limpiar", command=self.clear_plot).pack(fill="x", pady=(4, 0))

    def _build_plot(self, parent):
        c = COLORS
        self.fig = Figure(figsize=(8, 4), dpi=100, facecolor=c["panel"])
        self.ax_roll = self.fig.add_subplot(311)
        self.ax_pitch = self.fig.add_subplot(312, sharex=self.ax_roll)
        self.ax_yaw = self.fig.add_subplot(313, sharex=self.ax_roll)
        for ax, title in ((self.ax_roll, "Roll"), (self.ax_pitch, "Pitch"), (self.ax_yaw, "Yaw")):
            ax.set_facecolor(c["bg"])
            ax.set_xlim(0, SAMPLES)
            ax.set_ylim(-60, 60)
            ax.set_ylabel(f"{title} (°)", color=c["fg"])
            ax.tick_params(colors=c["muted"])
            ax.grid(True, color=c["grid"], linewidth=0.6)
            for sp in ax.spines.values():
                sp.set_color(c["grid"])
        self.ax_yaw.set_ylim(-190, 190)
        self.ax_yaw.set_xlabel("Muestras", color=c["fg"])

        self.l_roll, = self.ax_roll.plot([], [], lw=2, color=c["roll"], label="Roll")
        self.l_rolld, = self.ax_roll.plot([], [], lw=1.5, ls="--", color=c["warn"], label="Roll ref")
        self.l_pitch, = self.ax_pitch.plot([], [], lw=2, color=c["pitch"], label="Pitch")
        self.l_pitchd, = self.ax_pitch.plot([], [], lw=1.5, ls="--", color=c["warn"], label="Pitch ref")
        self.l_yaw, = self.ax_yaw.plot([], [], lw=2, color=c["yaw"], label="Yaw")
        for ax in (self.ax_roll, self.ax_pitch, self.ax_yaw):
            leg = ax.legend(loc="upper left", facecolor=c["panel"], edgecolor=c["grid"], fontsize=8)
            for t in leg.get_texts():
                t.set_color(c["fg"])
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

    def _build_statusbar(self):
        bar = ttk.Frame(self.root, padding=(10, 2))
        bar.pack(fill="x", side="bottom")
        self.lbl_status = ttk.Label(bar, text="Listo", style="Status.TLabel")
        self.lbl_status.pack(side="left")
        self.lbl_rate = ttk.Label(bar, text="0 Hz · 0 paquetes", style="Status.TLabel")
        self.lbl_rate.pack(side="right")
        ttk.Label(bar, text="Atajos: Esc/Espacio = paro · Ctrl+A = armar/desarmar   ",
                  style="Status.TLabel").pack(side="right")

    def _bind_keys(self):
        self.root.bind("<Escape>", lambda e: self.emergency_stop())
        self.root.bind("<space>", lambda e: self.emergency_stop())
        self.root.bind("<Control-a>", lambda e: self.toggle_arm())

    # -------------------------------------------------------------- acciones
    def set_status(self, text):
        self.lbl_status.config(text=f"{datetime.now():%H:%M:%S}  {text}")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_ports["values"] = ports + [SIM_PORT]
        if self.combo_ports.get() not in self.combo_ports["values"]:
            self.combo_ports.set(ports[0] if ports else "")
        self.set_status(f"{len(ports)} puerto(s) encontrado(s)")

    def toggle_connection(self):
        if self.link.connected:
            self.disconnect()
            return
        port = self.combo_ports.get()
        if not port:
            messagebox.showwarning("Aviso", "Selecciona un puerto")
            return
        try:
            self.link.open(port, self.combo_baud.get())
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir {port}:\n{e}")
            return
        self.lbl_conn.config(text=f"● Conectado a {port}", fg=COLORS["ok"])
        self.btn_connect.config(text="Desconectar")
        self.combo_ports.config(state="disabled")
        self.combo_baud.config(state="disabled")
        self.set_status(f"Conectado a {port}")
        self.send_values()

    def disconnect(self, reason=None):
        if self.armed:
            self.emergency_stop()
        self.link.close()
        self.lbl_conn.config(text="● Desconectado", fg=COLORS["danger"])
        self.btn_connect.config(text="Conectar")
        self.combo_ports.config(state="readonly")
        self.combo_baud.config(state="readonly")
        self.set_status(reason or "Desconectado")

    def current_command(self):
        def safe(var, default=0):
            try:
                return int(var.get())
            except (tk.TclError, ValueError):
                return default
        p = max(0, min(PID_MAX, safe(self.var_p, PID_DEFAULTS[0])))
        i = max(0, min(PID_MAX, safe(self.var_i, PID_DEFAULTS[1])))
        d = max(0, min(PID_MAX, safe(self.var_d, PID_DEFAULTS[2])))
        return (1 if self.armed else 0, safe(self.var_thr), safe(self.var_roll), safe(self.var_pitch), p, i, d,
                max(-YAW_CMD_MAX, min(YAW_CMD_MAX, safe(self.var_yaw))))

    def send_values(self):
        self.link.write(*self.current_command())

    def toggle_arm(self):
        if not self.armed:
            if not self.link.connected:
                messagebox.showwarning("Aviso", "Conéctate antes de armar el dron")
                return
            if self.var_thr.get() > 0:
                messagebox.showwarning("Aviso", "Baja el throttle a 0 antes de armar")
                return
            if not messagebox.askyesno("Armar", "¿Armar los motores?\nAsegúrate de que la zona esté despejada."):
                return
            self.armed = True
            self.btn_arm.config(text="DESARMAR", style="Disarm.TButton")
            self.set_status("Dron ARMADO")
            self.send_values()
        else:
            self._disarm("Dron desarmado")

    def emergency_stop(self):
        self._disarm("¡PARO DE EMERGENCIA!")

    def _disarm(self, msg):
        self.armed = False
        self.var_thr.set(0)
        self.var_roll.set(0)
        self.var_pitch.set(0)
        self.var_yaw.set(0)
        self.btn_arm.config(text="ARMAR", style="Arm.TButton")
        self.send_values()
        self.set_status(msg)

    def reset_pid(self):
        self.var_p.set(PID_DEFAULTS[0])
        self.var_i.set(PID_DEFAULTS[1])
        self.var_d.set(PID_DEFAULTS[2])
        self.send_values()

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="▶ Reanudar" if self.paused else "⏸ Pausar")

    def clear_plot(self):
        for dq in (self.t_roll, self.t_pitch, self.t_rolld, self.t_pitchd, self.t_yaw):
            dq.clear()
        self._redraw()

    # ---------------------------------------------------------------- CSV
    def choose_log_dir(self):
        d = filedialog.askdirectory(initialdir=self.log_dir)
        if d:
            self.log_dir = d
            if self.var_log.get():
                self._close_log()
                self._open_log()

    def toggle_logging(self):
        if self.var_log.get():
            self._open_log()
        else:
            self._close_log()
            self.lbl_log.config(text="Sin registro")

    def _open_log(self):
        name = os.path.join(self.log_dir, f"vuelo_{datetime.now():%Y%m%d_%H%M%S}.csv")
        try:
            self.csv_file = open(name, "w", newline="")
        except OSError as e:
            self.var_log.set(False)
            messagebox.showerror("Error", f"No se pudo crear el archivo:\n{e}")
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "rolld", "pitchd", "arm", "throttle", "P", "I", "D", "yaw", "yawd"])
        self.lbl_log.config(text=f"Guardando: {os.path.basename(name)}")
        self.set_status("Registro CSV iniciado")

    def _close_log(self):
        if self.csv_file:
            self.csv_file.close()
            self.set_status("Registro CSV cerrado")
        self.csv_file = None
        self.csv_writer = None

    # ------------------------------------------------------- ciclos periódicos
    def _poll(self):
        got_data = False
        while True:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if item[0] == "error":
                self.disconnect(f"Error de comunicación: {item[1]}")
                continue
            _, roll, pitch, yaw = item
            arm, thr, rolld, pitchd, p, i, d, yawd = self.current_command()
            self.rx_count += 1
            got_data = True
            if not self.paused:
                self.t_roll.append(roll)
                self.t_pitch.append(pitch)
                self.t_rolld.append(rolld)
                self.t_pitchd.append(pitchd)
                self.t_yaw.append(yaw if yaw is not None else float("nan"))
            if self.csv_writer:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                self.csv_writer.writerow([ts, roll, pitch, rolld, pitchd, arm, thr, p, i, d, yaw, yawd])
            self._update_readouts(roll, pitch, yaw, rolld, pitchd)
            if self.udp:  # reenvío al visor 3D de la 2da PC
                msg = f"{roll},{pitch}" + (f",{yaw}" if yaw is not None else "")
                try:
                    self.udp.sendto(msg.encode(), (PC2_IP, PC2_PORT))
                except OSError:
                    pass
        if got_data and not self.paused:
            self._redraw()
        self.root.after(PLOT_PERIOD_MS, self._poll)

    def _update_readouts(self, roll, pitch, yaw, rolld, pitchd):
        self.readouts["roll"].config(text=f"{roll:+.0f}°")
        self.readouts["pitch"].config(text=f"{pitch:+.0f}°")
        self.readouts["yaw"].config(text="—" if yaw is None else f"{yaw:+.0f}°")
        for key, err in (("eroll", rolld - roll), ("epitch", pitchd - pitch)):
            color = COLORS["ok"] if abs(err) < 3 else COLORS["warn"] if abs(err) < 10 else COLORS["danger"]
            self.readouts[key].config(text=f"{err:+.0f}°", foreground=color)

    def _redraw(self):
        x = range(len(self.t_roll))
        self.l_roll.set_data(x, self.t_roll)
        self.l_rolld.set_data(x, self.t_rolld)
        self.l_pitch.set_data(x, self.t_pitch)
        self.l_pitchd.set_data(x, self.t_pitchd)
        self.l_yaw.set_data(x, self.t_yaw)
        self.canvas.draw_idle()

    def _keepalive(self):
        if self.link.connected:
            self.send_values()
        self.root.after(KEEPALIVE_MS, self._keepalive)

    def _update_rate(self):
        now = time.time()
        rate = (self.rx_count - self.rx_count_last) / (now - self.rate_t)
        self.rx_count_last, self.rate_t = self.rx_count, now
        self.lbl_rate.config(text=f"{rate:.0f} Hz · {self.rx_count} paquetes")
        self.root.after(1000, self._update_rate)

    def on_close(self):
        if self.link.connected:
            self._disarm("Cerrando")
        self.link.close()
        self._close_log()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
