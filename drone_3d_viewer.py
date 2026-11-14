#!/usr/bin/env python3
# Visor 3D del cuadricoptero (2da PC)
# Recibe "roll,pitch[,yaw]" en grados por UDP (enviado por interface.py) o por puerto Serial.
# Solo usa tkinter (incluido en Python). pyserial es opcional, solo para el modo Serial.
import math, socket, threading, time
import tkinter as tk
from tkinter import ttk
try:
    import serial, serial.tools.list_ports
except ImportError:
    serial = None

UDP_PORT = 5005
BG, PANEL, FG, MUTED = "#0f141b", "#18202b", "#e6edf3", "#7d8a99"
COLORS = ("#ff6b6b", "#4dabf7", "#51cf66")   # roll, pitch, yaw
RANGES = (45, 45, 180)                        # escala de las barras (grados)

S = {"tgt": [0.0, 0.0, 0.0], "last": 0.0, "n": 0, "gen": 0, "mode": "Demo", "err": ""}
cur, off = [0.0] * 3, [0.0] * 3               # angulos mostrados / offsets de "poner a cero"
view = {"az": -0.6, "el": 0.45, "zoom": 1.0}

# ---------------- recepcion de datos ----------------
def parse(line):
    try:
        v = [float(x) for x in line.strip().split(",")]
    except ValueError:
        return
    if len(v) >= 2:
        S["tgt"] = [v[0], v[1], v[2] if len(v) > 2 else S["tgt"][2]]
        S["last"] = time.time()
        S["n"] += 1

def udp_loop(gen):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", UDP_PORT)); sock.settimeout(0.3)
    except OSError as e:
        S["err"] = f"UDP: {e.strerror}"; return
    while S["gen"] == gen:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        parse(data.decode(errors="ignore"))
    sock.close()

def serial_loop(port, gen):
    try:
        ser = serial.Serial(port, 115200, timeout=0.3)
    except Exception as e:
        S["err"] = f"Serial: {e}"; return
    while S["gen"] == gen:
        try:
            parse(ser.readline().decode(errors="ignore"))
        except Exception:
            break
    ser.close()

# ---------------- modelo 3D ----------------
# Ejes del dron: X al frente, Y a la izquierda, Z arriba.
def rot(roll, pitch, yaw):          # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]]

V = [(sx*.3, sy*.2, sz*.07) for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)]
FACES = [([0, 1, 3, 2], "#d9480f"),   # frente
         ([4, 5, 7, 6], "#495866"),   # atras
         ([0, 1, 5, 4], "#56626f"),   # lado izq
         ([2, 3, 7, 6], "#56626f"),   # lado der
         ([0, 2, 6, 4], "#7b8794"),   # arriba
         ([1, 3, 7, 5], "#3a444f")]   # abajo
TOP = 4
ARROW = [(.5, 0, .075), (.12, .13, .075), (.12, -.13, .075)]
MOTORS = [(.8, .8), (.8, -.8), (-.8, .8), (-.8, -.8)]
CIRC = [(math.cos(a * math.pi / 9), math.sin(a * math.pi / 9)) for a in range(18)]

def draw():
    cv.delete("all")
    W, H = cv.winfo_width(), cv.winfo_height()
    cx, cy = W / 2, H / 2 + 20
    sc = min(W, H) * 0.3 * view["zoom"]
    ca, sa = math.cos(view["az"]), math.sin(view["az"])
    ce, se = math.cos(view["el"]), math.sin(view["el"])

    def pj(p):                        # mundo -> pantalla (x, y, profundidad)
        x, y, z = p
        x, y = ca*x - sa*y, sa*x + ca*y
        up, dep = z*ce + x*se, x*ce - z*se
        k = sc * 6 / (6 + dep)
        return cx - y*k, cy - up*k, dep

    R = rot(cur[0], -cur[1], -cur[2])
    tf = lambda p: tuple(sum(R[i][j] * p[j] for j in range(3)) for i in range(3))
    P = lambda pts: [pj(tf(p)) for p in pts]
    flat = lambda q: [c for t in q for c in t[:2]]

    for i in range(-2, 3):            # piso de referencia (no gira con el dron)
        for a, b in (((i, -2, -.7), (i, 2, -.7)), ((-2, i, -.7), (2, i, -.7))):
            p, q = pj(a), pj(b)
            cv.create_line(p[0], p[1], q[0], q[1], fill="#243040")

    items = []
    def add(q, *draws):
        items.append((sum(t[2] for t in q) / len(q), draws))

    for mx, my in MOTORS:
        q = P([(0, 0, 0), (mx, my, 0)])
        add(q, ("l", flat(q), dict(fill="#aab4bf", width=7, capstyle="round")))
        col = "#ff8c42" if mx > 0 else "#4dabf7"
        pq = P([(mx + .45*c, my + .45*s, .1) for c, s in CIRC])
        hq = P([(mx + .09*c, my + .09*s, .12) for c, s in CIRC])
        add(pq, ("p", flat(pq), dict(fill=col, outline=col, stipple="gray50")),
                ("p", flat(hq), dict(fill="#222a33", outline="")))
    for i, (idx, col) in enumerate(FACES):
        q = P([V[j] for j in idx])
        d = [("p", flat(q), dict(fill=col, outline="#0b0f14"))]
        if i == TOP:
            d.append(("p", flat(P(ARROW)), dict(fill="#ffd43b", outline="")))
        add(q, *d)
    for _, draws in sorted(items, key=lambda t: -t[0]):   # los mas lejanos primero
        for kind, xy, kw in draws:
            (cv.create_polygon if kind == "p" else cv.create_line)(*xy, **kw)

    n = pj(tf((.95, 0, .1)))
    cv.create_text(n[0], n[1] - 14, text="FRENTE", fill=MUTED, font=("Segoe UI", 9))
    cv.create_text(10, H - 10, anchor="sw", fill=MUTED, font=("Segoe UI", 9),
                   text="Arrastra: rotar vista  |  Rueda: zoom")

# ---------------- interfaz ----------------
root = tk.Tk()
root.title("Visor 3D - Cuadricoptero")
root.configure(bg=BG)
root.geometry("1050x650")
root.columnconfigure(0, weight=1); root.rowconfigure(0, weight=1)

cv = tk.Canvas(root, bg=BG, highlightthickness=0)
cv.grid(row=0, column=0, sticky="nsew", padx=(10, 0), pady=10)
side = tk.Frame(root, bg=BG)
side.grid(row=0, column=1, sticky="ns", padx=10, pady=10)

vals, bars = [], []
for name, col in zip(("ROLL", "PITCH", "YAW"), COLORS):
    f = tk.Frame(side, bg=PANEL, padx=14, pady=8); f.pack(fill="x", pady=4)
    tk.Label(f, text=name, bg=PANEL, fg=col, font=("Segoe UI", 10, "bold")).pack(anchor="w")
    v = tk.Label(f, text="+0.0°", bg=PANEL, fg=FG, font=("Consolas", 28, "bold")); v.pack(anchor="w")
    b = tk.Canvas(f, width=250, height=10, bg=PANEL, highlightthickness=0); b.pack(anchor="w", pady=(2, 0))
    vals.append(v); bars.append(b)

def bar(c, v, rng, col):
    c.delete("all")
    m, x = 125, 125 + max(-1, min(1, v / rng)) * 125
    c.create_rectangle(0, 0, 250, 10, fill="#0f141b", outline="")
    c.create_rectangle(min(m, x), 0, max(m, x), 10, fill=col, outline="")
    c.create_line(m, 0, m, 10, fill=FG)

ctl = tk.Frame(side, bg=PANEL, padx=14, pady=10); ctl.pack(fill="x", pady=4)
tk.Label(ctl, text="FUENTE DE DATOS", bg=PANEL, fg=MUTED, font=("Segoe UI", 9, "bold")).pack(anchor="w")
src = ttk.Combobox(ctl, values=["Demo", "UDP (desde interface.py)", "Serial"], state="readonly", width=28)
src.set("Demo"); src.pack(anchor="w", pady=3)
port = ttk.Combobox(ctl, width=28); port.pack(anchor="w", pady=3)
hint = tk.Label(ctl, bg=PANEL, fg=MUTED, font=("Segoe UI", 9)); hint.pack(anchor="w")
row = tk.Frame(ctl, bg=PANEL); row.pack(anchor="w", pady=4)

def on_src(_=None):
    m = src.get()
    if m == "Serial":
        ports = [p.device for p in serial.tools.list_ports.comports()] if serial else []
        port["values"] = ports; port.set(ports[0] if ports else "")
        hint.config(text="Elige el puerto y pulsa Iniciar" if serial else "Falta: pip install pyserial")
    elif m.startswith("UDP"):
        hint.config(text=f"Escucha en el puerto UDP {UDP_PORT}")
    else:
        hint.config(text="Datos simulados para probar el modelo")

def start():
    S["gen"] += 1; S["err"] = ""; S["last"] = 0; m = S["mode"] = src.get()
    if m.startswith("UDP"):
        threading.Thread(target=udp_loop, args=(S["gen"],), daemon=True).start()
    elif m == "Serial":
        if serial is None: S["err"] = "Falta pyserial"
        else: threading.Thread(target=serial_loop, args=(port.get(), S["gen"]), daemon=True).start()

def stop():
    S["gen"] += 1; S["mode"] = ""

ttk.Button(row, text="Iniciar", command=start).pack(side="left", padx=(0, 6))
ttk.Button(row, text="Detener", command=stop).pack(side="left")
src.bind("<<ComboboxSelected>>", on_src)

adj = tk.Frame(side, bg=PANEL, padx=14, pady=10); adj.pack(fill="x", pady=4)
tk.Label(adj, text="AJUSTES", bg=PANEL, fg=MUTED, font=("Segoe UI", 9, "bold")).pack(anchor="w")
inv = [tk.BooleanVar() for _ in range(3)]
for t, var in zip(("Invertir roll", "Invertir pitch", "Invertir yaw"), inv):
    tk.Checkbutton(adj, text=t, variable=var, bg=PANEL, fg=FG, selectcolor="#2b3646",
                   activebackground=PANEL, activeforeground=FG).pack(anchor="w")
row2 = tk.Frame(adj, bg=PANEL); row2.pack(anchor="w", pady=4)
ttk.Button(row2, text="Poner a cero", command=lambda: off.__setitem__(slice(None), S["tgt"])).pack(side="left", padx=(0, 6))
ttk.Button(row2, text="Quitar cero", command=lambda: off.__setitem__(slice(None), [0, 0, 0])).pack(side="left")

status = tk.Label(side, bg=BG, fg=MUTED, anchor="w", font=("Segoe UI", 10)); status.pack(fill="x", pady=6)

# vista con el mouse
drag = {}
def press(e): drag["x"], drag["y"] = e.x, e.y
def move(e):
    view["az"] += (e.x - drag["x"]) * .01
    view["el"] = max(.05, min(1.5, view["el"] + (e.y - drag["y"]) * .01))
    drag["x"], drag["y"] = e.x, e.y
cv.bind("<ButtonPress-1>", press); cv.bind("<B1-Motion>", move)
cv.bind("<MouseWheel>", lambda e: view.__setitem__("zoom", max(.4, min(2.5, view["zoom"] * (1.1 if e.delta > 0 else .9)))))

hz = {"t": time.time(), "n": 0, "v": 0.0}
def tick():
    now = time.time()
    if S["mode"] == "Demo":
        S["tgt"] = [28 * math.sin(now * .9), 20 * math.sin(now * .6 + 1), (now * 25 + 180) % 360 - 180]
        S["last"] = now
    for i in range(3):                # suavizado + inversion + offset
        t = (-1 if inv[i].get() else 1) * (S["tgt"][i] - off[i])
        d = t - cur[i]
        if i == 2: d = (d + 180) % 360 - 180
        cur[i] += .35 * d
    cur[2] = (cur[2] + 180) % 360 - 180
    for i in range(3):
        vals[i].config(text=f"{cur[i]:+.1f}°"); bar(bars[i], cur[i], RANGES[i], COLORS[i])
    if now - hz["t"] >= 1:
        hz["v"] = (S["n"] - hz["n"]) / (now - hz["t"]); hz["n"] = S["n"]; hz["t"] = now
    if S["mode"] == "Demo": s, c = "● Demo (datos simulados)", "#ffd43b"
    elif S["err"]: s, c = "● " + S["err"], "#ff6b6b"
    elif S["mode"] and now - S["last"] < 1: s, c = f"● Recibiendo · {hz['v']:.0f} Hz", "#51cf66"
    elif S["mode"]: s, c = "● Esperando datos…", MUTED
    else: s, c = "● Detenido", MUTED
    status.config(text=s, fg=c)
    draw()
    root.after(33, tick)

root.protocol("WM_DELETE_WINDOW", lambda: (stop(), root.destroy()))
on_src()
root.after(100, tick)
root.mainloop()
