"""
Simulador de búsqueda para robot de rescate.

Compara BFS, Dijkstra y A* sobre una cuadrícula con escombros y fugas de gas.

Requisitos: Python 3.8 o superior. Usa tkinter, que viene incluido con Python
en Windows y macOS. En Linux: sudo apt install python3-tk

Ejecución: python simulador.py
"""
from __future__ import annotations

import heapq
import itertools
import math
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk

# ---------- Configuración ----------
COLUMNAS, FILAS, TAM = 20, 13, 40
INICIO = (1, 6)              # entrada del edificio (columna, fila)
OBJETIVO = (18, 6)           # posición del superviviente
COSTO_GAS = 5                # costo extra por entrar a una celda con gas
CUADROS_EXPLORACION = 60     # cuadros aproximados de la animación de exploración
VELOCIDADES = {"Pausada": 170, "Normal": 90, "Rápida": 35}  # milisegundos por cuadro

LIBRE, ESCOMBRO, GAS = 0, 1, 2
NOMBRE_TIPO = {LIBRE: "libre", ESCOMBRO: "escombro", GAS: "gas"}

ALGORITMOS = [
    ("A* · costo + heurística", "astar"),
    ("Dijkstra · menor costo", "dijkstra"),
    ("BFS · menor número de pasos", "bfs"),
]
NOMBRES = {"bfs": "BFS", "dijkstra": "Dijkstra", "astar": "A*"}
DESCRIPCIONES = {
    "bfs": "Busca la ruta con menos pasos. No considera el gas.",
    "dijkstra": "Busca la ruta de menor costo: cada paso cuesta 1 y cada celda con gas suma 5.",
    "astar": "Igual que Dijkstra, pero guiado por una estimación de distancia al superviviente.",
}

# Bloques de escombros: (columna inicial, fila inicial, columna final, fila final)
BLOQUES = [
    (4, 0, 4, 4), (4, 7, 4, 12), (8, 2, 8, 9), (12, 0, 12, 5),
    (12, 8, 12, 12), (16, 3, 16, 12), (2, 2, 2, 3), (6, 10, 7, 10),
    (10, 11, 11, 11), (14, 1, 15, 1),
]
# Fugas de gas: (columna, fila)
FUGAS = [
    (3, 6), (5, 5), (5, 6), (7, 3), (7, 4), (9, 9), (10, 9),
    (11, 7), (13, 6), (14, 7), (15, 5), (17, 6), (17, 7),
]

# Colores
FONDO, PANEL, TEXTO = "#08111b", "#0f1c2b", "#e6eef6"
SUAVE, CIAN, VERDE, ROJO, NARANJA = "#8fa3b8", "#43d9ce", "#83e3a0", "#ff6f80", "#ffb454"
COLOR_LIBRE, COLOR_ESCOMBRO, COLOR_GAS = "#112236", "#2c3b4c", "#3a2a22"


# ---------- Modelo y algoritmos ----------
def mapa_original() -> list[list[int]]:
    """Construye la matriz del mapa: mapa[fila][columna]."""
    m = [[LIBRE] * COLUMNAS for _ in range(FILAS)]
    for x1, y1, x2, y2 in BLOQUES:
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                m[y][x] = ESCOMBRO
    for x, y in FUGAS:
        if m[y][x] == LIBRE:
            m[y][x] = GAS
    return m


def vecinos(mapa: list[list[int]], pos: tuple[int, int]):
    """Celdas transitables vecinas, en orden derecha, abajo, izquierda, arriba."""
    x, y = pos
    for nx, ny in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
        if 0 <= nx < COLUMNAS and 0 <= ny < FILAS and mapa[ny][nx] != ESCOMBRO:
            yield (nx, ny)


@dataclass
class Resultado:
    algoritmo: str
    ruta: list[tuple[int, int]]
    orden: list[tuple[int, int]]   # nodos expandidos, en el orden en que se revisaron
    llego: bool
    pasos: int
    gas: int                       # celdas con gas que cruza la ruta
    costo: int | None
    ms: float                      # tiempo de cálculo en milisegundos


def buscar(mapa: list[list[int]], algoritmo: str) -> Resultado:
    """
    Ejecuta BFS, Dijkstra o A*.
    BFS: costo 1 por paso, ignora el gas.
    Dijkstra: costo 1 por paso + COSTO_GAS por entrar a una celda con gas.
    A*: igual que Dijkstra, pero prioriza según costo + distancia Manhattan estimada.
    """
    inicio_tiempo = time.perf_counter()
    usa_gas = algoritmo != "bfs"

    def estimar(p):
        return abs(p[0] - OBJETIVO[0]) + abs(p[1] - OBJETIVO[1])

    costo = {INICIO: 0}
    previo: dict[tuple[int, int], tuple[int, int]] = {}
    cerrados: set[tuple[int, int]] = set()
    orden: list[tuple[int, int]] = []
    desempate = itertools.count()   # mantiene el orden de llegada en empates
    frontera = [(estimar(INICIO) if algoritmo == "astar" else 0, next(desempate), 0, INICIO)]
    llego = False

    while frontera:
        _, _, g, pos = heapq.heappop(frontera)
        if pos in cerrados:
            continue
        cerrados.add(pos)
        orden.append(pos)
        if pos == OBJETIVO:
            llego = True
            break

        for v in vecinos(mapa, pos):
            entra_gas = usa_gas and mapa[v[1]][v[0]] == GAS
            nuevo = g + 1 + (COSTO_GAS if entra_gas else 0)
            if nuevo < costo.get(v, math.inf):
                costo[v] = nuevo
                previo[v] = pos
                f = nuevo + (estimar(v) if algoritmo == "astar" else 0)
                heapq.heappush(frontera, (f, next(desempate), nuevo, v))

    ruta: list[tuple[int, int]] = []
    if llego:
        p = OBJETIVO
        while True:
            ruta.append(p)
            if p == INICIO:
                break
            p = previo[p]
        ruta.reverse()

    return Resultado(
        algoritmo=algoritmo,
        ruta=ruta,
        orden=orden,
        llego=llego,
        pasos=len(ruta) - 1 if ruta else 0,
        gas=sum(1 for x, y in ruta if mapa[y][x] == GAS),
        costo=costo[OBJETIVO] if llego else None,
        ms=(time.perf_counter() - inicio_tiempo) * 1000,
    )


def mezclar(c1: str, c2: str, t: float) -> str:
    """Mezcla dos colores hexadecimales. t=0 es c1 y t=1 es c2."""
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    t = max(0.0, min(1.0, t))
    return "#%02x%02x%02x" % tuple(round(a[i] + (b[i] - a[i]) * t) for i in range(3))


# ---------- Interfaz ----------
class App:
    def __init__(self, raiz: tk.Tk):
        self.raiz = raiz
        raiz.title("Robot de rescate · simulador de rutas")
        raiz.configure(bg=FONDO)
        raiz.resizable(False, False)

        self.mapa = mapa_original()
        self.fase = "listo"            # listo | explorando | ruta | fin
        self.job = None                # id de la animación programada con after()
        self.ultimo: Resultado | None = None
        self.orden: list[tuple[int, int]] = []
        self.ruta: list[tuple[int, int]] = []
        self.visitas: dict[tuple[int, int], int] = {}
        self.idx_exp = 0
        self.idx_ruta = 0
        self.paso = 1
        self.robot = INICIO

        self._construir()
        self.reiniciar()
        self.registrar("Sistema listo. Mapa de 20 × 13 celdas cargado.")
        self.registrar("Entrada en E, superviviente en S. Los escombros bloquean; el gas añade costo.")

    # ----- construcción de la ventana -----
    def _construir(self):
        estilo = ttk.Style()
        estilo.theme_use("clam")
        estilo.configure("Treeview", background="#0b1825", foreground=TEXTO,
                         fieldbackground="#0b1825", rowheight=24, borderwidth=0)
        estilo.configure("Treeview.Heading", background="#17293b", foreground=SUAVE, relief="flat")
        estilo.map("Treeview", background=[("selected", "#17293b")])
        estilo.configure("TCombobox", fieldbackground="#0a1724", background="#17293b",
                         foreground=TEXTO, arrowcolor=CIAN)

        principal = tk.Frame(self.raiz, bg=FONDO)
        principal.pack(padx=18, pady=18)

        # Columna izquierda: mapa
        izquierda = tk.Frame(principal, bg=FONDO)
        izquierda.grid(row=0, column=0, sticky="n")
        tk.Label(izquierda, text="Edificio colapsado", font=("Helvetica", 15, "bold"),
                 bg=FONDO, fg=TEXTO).pack(anchor="w")
        tk.Label(izquierda, text="El robot debe llegar desde la entrada (E) hasta el superviviente (S).",
                 font=("Helvetica", 10), bg=FONDO, fg=SUAVE).pack(anchor="w")
        self.estado = tk.StringVar(value="Listo")
        tk.Label(izquierda, textvariable=self.estado, font=("Helvetica", 10, "bold"),
                 bg=FONDO, fg=VERDE).pack(anchor="w", pady=(4, 6))

        self.lienzo = tk.Canvas(izquierda, width=COLUMNAS * TAM, height=FILAS * TAM,
                                bg="#091521", highlightthickness=1, highlightbackground="#263b50")
        self.lienzo.pack()
        self.lienzo.bind("<Button-1>", self.clic_mapa)

        tk.Label(izquierda, font=("Helvetica", 10), bg=FONDO, fg=SUAVE,
                 text="■ Robot   ■ Superviviente   ■ Escombros   ■ Gas   ■ Explorado   ■ Ruta final"
                 ).pack(anchor="w", pady=(8, 0))
        tk.Label(izquierda, font=("Helvetica", 10), bg=FONDO, fg=SUAVE,
                 text="Clic en una celda para editarla: libre → escombro → gas → libre."
                 ).pack(anchor="w")

        # Columna derecha: controles, indicadores, tabla y bitácora
        derecha = tk.Frame(principal, bg=FONDO)
        derecha.grid(row=0, column=1, padx=(18, 0), sticky="n")

        # Planificación
        plan = self._tarjeta(derecha, "Planificación", "Elige cómo buscar la ruta y pulsa iniciar.")
        tk.Label(plan, text="ALGORITMO", font=("Helvetica", 9, "bold"), bg=PANEL, fg=SUAVE).pack(anchor="w")
        self.combo_algo = ttk.Combobox(plan, values=[etiqueta for etiqueta, _ in ALGORITMOS],
                                       state="readonly", width=38)
        self.combo_algo.current(0)
        self.combo_algo.pack(anchor="w", pady=(2, 4))
        self.combo_algo.bind("<<ComboboxSelected>>", self.cambiar_algoritmo)
        self.descripcion = tk.StringVar(value=DESCRIPCIONES[self._clave_algoritmo()])
        tk.Label(plan, textvariable=self.descripcion, wraplength=340, justify="left",
                 font=("Helvetica", 10), bg=PANEL, fg=SUAVE).pack(anchor="w", pady=(0, 8))

        tk.Label(plan, text="VELOCIDAD", font=("Helvetica", 9, "bold"), bg=PANEL, fg=SUAVE).pack(anchor="w")
        self.combo_vel = ttk.Combobox(plan, values=list(VELOCIDADES), state="readonly", width=38)
        self.combo_vel.set("Normal")
        self.combo_vel.pack(anchor="w", pady=(2, 10))

        botones = tk.Frame(plan, bg=PANEL)
        botones.pack(fill="x")
        botones.columnconfigure((0, 1), weight=1)
        self.boton = self._boton(botones, "▶ Iniciar", self.alternar, "#0a7b7d", 0, 0, columnspan=2)
        self._boton(botones, "↺ Reiniciar", self.reiniciar_manual, "#17293b", 1, 0)
        self._boton(botones, "▦ Comparar", self.comparar, "#10292d", 1, 1)
        self._boton(botones, "Restablecer mapa original", self.restablecer, "#17293b", 2, 0, columnspan=2)

        # Indicadores
        ind = self._tarjeta(derecha, "Indicadores", "Resultados de la última búsqueda.")
        self.indicadores = {k: tk.StringVar(value="—") for k in ("pasos", "explorados", "gas", "costo", "tiempo")}
        rejilla = tk.Frame(ind, bg=PANEL)
        rejilla.pack(fill="x")
        etiquetas = [("PASOS DE LA RUTA", "pasos"), ("NODOS EXPLORADOS", "explorados"),
                     ("CELDAS CON GAS", "gas"), ("COSTO TOTAL", "costo")]
        for i, (titulo, clave) in enumerate(etiquetas):
            caja = tk.Frame(rejilla, bg="#0b1825", padx=10, pady=6, highlightthickness=1,
                            highlightbackground="#203449")
            caja.grid(row=i // 2, column=i % 2, padx=4, pady=4, sticky="nsew")
            tk.Label(caja, text=titulo, font=("Helvetica", 8, "bold"), bg="#0b1825", fg=SUAVE).pack(anchor="w")
            tk.Label(caja, textvariable=self.indicadores[clave], font=("Helvetica", 16, "bold"),
                     bg="#0b1825", fg=TEXTO).pack(anchor="w")
        caja = tk.Frame(rejilla, bg="#0b1825", padx=10, pady=6, highlightthickness=1,
                        highlightbackground="#203449")
        caja.grid(row=2, column=0, columnspan=2, padx=4, pady=4, sticky="nsew")
        tk.Label(caja, text="TIEMPO DE CÁLCULO", font=("Helvetica", 8, "bold"), bg="#0b1825", fg=SUAVE).pack(anchor="w")
        tk.Label(caja, textvariable=self.indicadores["tiempo"], font=("Helvetica", 16, "bold"),
                 bg="#0b1825", fg=TEXTO).pack(anchor="w")

        self.resultado = tk.StringVar(value="Elige un algoritmo y pulsa Iniciar.")
        tk.Label(ind, textvariable=self.resultado, wraplength=360, justify="left", anchor="w",
                 font=("Helvetica", 10), bg="#0b1825", fg=SUAVE, padx=10, pady=8).pack(fill="x", pady=(8, 0))

        # Tabla de comparación
        self.tabla = ttk.Treeview(ind, columns=("alg", "pasos", "expl", "gas", "costo", "tiempo"),
                                  show="headings", height=3)
        for col, texto_col, ancho in [("alg", "Algoritmo", 90), ("pasos", "Pasos", 55), ("expl", "Explorados", 85),
                                      ("gas", "Gas", 50), ("costo", "Costo", 60), ("tiempo", "Tiempo", 80)]:
            self.tabla.heading(col, text=texto_col)
            self.tabla.column(col, width=ancho, anchor="center")
        self.tabla.tag_configure("rec", foreground=VERDE)
        self.tabla.pack(fill="x", pady=(8, 0))
        self.nota = tk.Label(ind, text="", wraplength=360, justify="left", font=("Helvetica", 9),
                             bg=PANEL, fg=SUAVE)
        self.nota.pack(anchor="w", pady=(4, 0))

        # Bitácora
        bit = self._tarjeta(derecha, "Bitácora del robot", None)
        self.bitacora = tk.Text(bit, height=7, width=44, state="disabled", bg="#0b1825", fg=SUAVE,
                                relief="flat", font=("Courier", 10), padx=8, pady=6, wrap="word")
        self.bitacora.pack(fill="x")

    def _tarjeta(self, padre, titulo, subtitulo):
        marco = tk.Frame(padre, bg=PANEL, padx=14, pady=12, highlightthickness=1,
                         highlightbackground="#22354a")
        marco.pack(fill="x", pady=(0, 12))
        tk.Label(marco, text=titulo, font=("Helvetica", 12, "bold"), bg=PANEL, fg=TEXTO).pack(anchor="w")
        if subtitulo:
            tk.Label(marco, text=subtitulo, font=("Helvetica", 9), bg=PANEL, fg=SUAVE).pack(anchor="w", pady=(0, 8))
        return marco

    def _boton(self, padre, texto, comando, color, fila, columna, columnspan=1):
        boton = tk.Button(padre, text=texto, command=comando, bg=color, fg=TEXTO,
                          activebackground="#1f3a52", activeforeground=TEXTO,
                          relief="flat", font=("Helvetica", 10, "bold"), pady=7, cursor="hand2")
        boton.grid(row=fila, column=columna, columnspan=columnspan, sticky="ew", padx=3, pady=3)
        return boton

    # ----- utilidades -----
    def _clave_algoritmo(self) -> str:
        return dict(ALGORITMOS)[self.combo_algo.get()]

    def _ms(self) -> int:
        return VELOCIDADES[self.combo_vel.get()]

    def registrar(self, mensaje: str):
        self.bitacora.configure(state="normal")
        self.bitacora.insert("end", f"› {mensaje}\n")
        self.bitacora.see("end")
        self.bitacora.configure(state="disabled")

    def _detener_animacion(self):
        if self.job is not None:
            self.raiz.after_cancel(self.job)
            self.job = None

    def mostrar_indicadores(self, r: Resultado):
        self.indicadores["pasos"].set(str(r.pasos) if r.llego else "—")
        self.indicadores["explorados"].set(str(len(r.orden)))
        self.indicadores["gas"].set(str(r.gas))
        self.indicadores["costo"].set(str(r.costo) if r.llego else "—")
        self.indicadores["tiempo"].set(f"{r.ms:.2f} ms")
        if r.llego:
            self.resultado.set(f"{NOMBRES[r.algoritmo]} encontró una ruta de {r.pasos} pasos "
                               f"con {r.gas} celda(s) de gas.")
        else:
            self.resultado.set(f"{NOMBRES[r.algoritmo]} no encontró ruta: "
                               "los escombros bloquean el paso al superviviente.")

    # ----- control de la simulación -----
    def reiniciar(self):
        self._detener_animacion()
        self.fase = "listo"
        self.ultimo = None
        self.orden, self.ruta, self.visitas = [], [], {}
        self.idx_exp = 0
        self.idx_ruta = 0
        self.robot = INICIO
        for var in self.indicadores.values():
            var.set("—")
        self.resultado.set("Elige un algoritmo y pulsa Iniciar.")
        self.tabla.delete(*self.tabla.get_children())
        self.nota.configure(text="")
        self.boton.configure(text="▶ Iniciar")
        self.estado.set("Listo")
        self.dibujar()

    def reiniciar_manual(self):
        self.reiniciar()
        self.registrar("Simulación reiniciada.")

    def restablecer(self):
        self.mapa = mapa_original()
        self.reiniciar()
        self.registrar("Mapa restablecido al original.")

    def cambiar_algoritmo(self, _evento=None):
        self.descripcion.set(DESCRIPCIONES[self._clave_algoritmo()])
        self.reiniciar()

    def nueva_busqueda(self):
        clave = self._clave_algoritmo()
        r = buscar(self.mapa, clave)
        self.ultimo = r
        self.orden, self.ruta = r.orden, r.ruta
        self.visitas = {pos: i for i, pos in enumerate(r.orden)}
        self.idx_exp = 0
        self.idx_ruta = 0
        self.robot = INICIO
        self.paso = max(1, math.ceil(len(r.orden) / CUADROS_EXPLORACION))
        self.mostrar_indicadores(r)

        if not r.llego:
            self.idx_exp = len(r.orden)
            self.fase = "fin"
            self.estado.set("Sin ruta")
            self.registrar(f"{NOMBRES[clave]}: no hay ruta hacia el superviviente.")
            self.dibujar()
            return

        self.fase = "explorando"
        self.registrar(f"{NOMBRES[clave]} inicia la búsqueda.")
        self.dibujar()

    def alternar(self):
        if self.job is not None:
            self.pausar()
            return
        if self.fase in ("listo", "fin"):
            self.nueva_busqueda()
            if self.fase == "fin":
                return
        self.boton.configure(text="❚❚ Pausar")
        self.estado.set("Explorando" if self.fase == "explorando" else "Siguiendo la ruta")
        self.job = self.raiz.after(self._ms(), self._tick)

    def pausar(self):
        self._detener_animacion()
        self.boton.configure(text="▶ Reanudar")
        self.estado.set("En pausa")

    def _tick(self):
        """Un cuadro de la animación: primero la exploración, después el recorrido."""
        self.job = None
        if self.fase == "explorando":
            self.idx_exp = min(len(self.orden), self.idx_exp + self.paso)
            if self.idx_exp >= len(self.orden):
                self.fase = "ruta"
                self.registrar(f"Exploración completa: {len(self.orden)} nodos revisados.")
                self.estado.set("Siguiendo la ruta")
        elif self.fase == "ruta":
            if self.idx_ruta < len(self.ruta) - 1:
                self.idx_ruta += 1
                self.robot = self.ruta[self.idx_ruta]
            else:
                self._terminar()
                self.dibujar()
                return

        self.dibujar()
        self.job = self.raiz.after(self._ms(), self._tick)

    def _terminar(self):
        self.fase = "fin"
        self.boton.configure(text="▶ Iniciar")
        self.estado.set("Superviviente localizado")
        self.registrar(f"El robot llegó al superviviente en {self.ultimo.pasos} pasos.")

    def comparar(self):
        self.reiniciar()
        resultados = [buscar(self.mapa, clave) for _, clave in ALGORITMOS]

        def costo_total(r: Resultado) -> float:
            return r.pasos + COSTO_GAS * r.gas if r.llego else math.inf

        recomendado = min(resultados, key=lambda r: (costo_total(r), len(r.orden)))
        hay_ruta = recomendado.llego

        self.tabla.delete(*self.tabla.get_children())
        for r in resultados:
            es_rec = hay_ruta and r is recomendado
            self.tabla.insert("", "end", tags=("rec",) if es_rec else (), values=(
                NOMBRES[r.algoritmo] + (" ★" if es_rec else ""),
                r.pasos if r.llego else "—",
                len(r.orden),
                r.gas,
                r.costo if r.llego else "—",
                f"{r.ms:.2f} ms",
            ))
        self.nota.configure(text=f"★ Recomendado por menor costo total (pasos + {COSTO_GAS} × celdas con gas). "
                                 "En empate gana el que explora menos nodos.")
        self.registrar(f"Comparación lista. Recomendado: {NOMBRES[recomendado.algoritmo]}."
                       if hay_ruta else "Comparación lista. Ningún algoritmo encontró ruta.")

    def clic_mapa(self, evento):
        x, y = evento.x // TAM, evento.y // TAM
        if not (0 <= x < COLUMNAS and 0 <= y < FILAS) or (x, y) in (INICIO, OBJETIVO):
            return
        self.reiniciar()
        self.mapa[y][x] = (self.mapa[y][x] + 1) % 3
        self.registrar(f"Celda ({x}, {y}) cambiada a {NOMBRE_TIPO[self.mapa[y][x]]}.")
        self.dibujar()

    # ----- dibujo -----
    def dibujar(self):
        c = self.lienzo
        c.delete("all")
        total = max(1, len(self.orden))

        for y in range(FILAS):
            for x in range(COLUMNAS):
                px, py = x * TAM, y * TAM
                tipo = self.mapa[y][x]
                color = {ESCOMBRO: COLOR_ESCOMBRO, GAS: COLOR_GAS}.get(tipo, COLOR_LIBRE)
                idx = self.visitas.get((x, y))
                if tipo == LIBRE and idx is not None and idx < self.idx_exp:
                    # Las celdas revisadas primero se ven más intensas
                    color = mezclar(COLOR_LIBRE, CIAN, 0.45 - 0.3 * idx / total)
                c.create_rectangle(px + 1, py + 1, px + TAM - 1, py + TAM - 1,
                                   fill=color, outline="#1d3042")
                cx, cy = px + TAM / 2, py + TAM / 2
                if tipo == ESCOMBRO:
                    c.create_text(cx, cy, text="✕", fill="#8b9bad", font=("Helvetica", 15, "bold"))
                elif tipo == GAS:
                    c.create_text(cx, cy, text="GAS", fill="#ffc98a", font=("Helvetica", 9, "bold"))

        # Ruta final: se dibuja conforme el robot avanza
        if self.fase in ("ruta", "fin") and self.ruta:
            puntos = []
            for x, y in self.ruta[: self.idx_ruta + 1]:
                puntos += [x * TAM + TAM / 2, y * TAM + TAM / 2]
            if len(puntos) >= 4:
                c.create_line(*puntos, fill=ROJO, width=4, joinstyle="round", capstyle="round")

        # Entrada (E) y superviviente (S)
        ex, ey = INICIO[0] * TAM, INICIO[1] * TAM
        c.create_rectangle(ex + 8, ey + 8, ex + TAM - 8, ey + TAM - 8, outline=CIAN, width=2)
        c.create_text(ex + TAM / 2, ey + TAM / 2, text="E", fill=CIAN, font=("Helvetica", 11, "bold"))
        sx, sy = OBJETIVO[0] * TAM, OBJETIVO[1] * TAM
        c.create_rectangle(sx + 7, sy + 7, sx + TAM - 7, sy + TAM - 7, fill=VERDE, outline="")
        c.create_text(sx + TAM / 2, sy + TAM / 2, text="S", fill="#082415", font=("Helvetica", 13, "bold"))

        # Robot
        rx, ry = self.robot[0] * TAM + TAM / 2, self.robot[1] * TAM + TAM / 2
        c.create_oval(rx - 13, ry - 13, rx + 13, ry + 13, fill=CIAN, outline="#2fb8ad", width=2)
        c.create_text(rx, ry, text="R", fill="#06201e", font=("Helvetica", 12, "bold"))


if __name__ == "__main__":
    raiz = tk.Tk()
    App(raiz)
    raiz.mainloop()
