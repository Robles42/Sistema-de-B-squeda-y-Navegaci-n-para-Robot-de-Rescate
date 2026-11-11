"""
Simulador 3D de búsqueda para robot de rescate (vista isométrica).

Compara BFS, Dijkstra y A* sobre un edificio con escombros y fugas de gas.
Incluye cámara controlada con el mouse y un editor de mapa.

Requisitos: Python 3.8 o superior y pygame.
    pip install pygame

Ejecución:
    python simulador_3d.py

Cámara (siempre disponible):
    Arrastrar clic derecho    girar e inclinar la cámara
    Arrastrar clic central    mover el mapa
    Rueda  o  + / -           acercar / alejar
    Q / E                     girar 90 grados (animado)
    V                         restablecer la cámara

Simulación:
    Clic en una celda         editar celda (libre -> escombro -> gas -> libre)
    Arrastrar clic izquierdo  también gira la cámara
    Espacio                   iniciar / pausar / reanudar
    R reiniciar · C comparar · O con / sin obstáculos

Editor de mapa (Tab para entrar / salir):
    Clic izquierdo            dibuja con la herramienta activa (Mayús = borrar)
    1 / 2 / 3                 material: libre / escombro / gas
    B L F G                   pincel / línea / relleno / gotero
    Ctrl+Z / Ctrl+Y           deshacer / rehacer
    Ctrl+S / Ctrl+L           guardar / cargar mapa_rescate.json
"""
from __future__ import annotations

import heapq
import itertools
import json
import math
import os
import random
import time
from dataclasses import dataclass

import pygame

# ---------- Configuración del mapa ----------
COLUMNAS, FILAS = 20, 13
INICIO = (1, 6)            # entrada del edificio (columna, fila)
OBJETIVO = (18, 6)         # superviviente
COSTO_GAS = 5              # costo extra por entrar a una celda con gas

LIBRE, ESCOMBRO, GAS = 0, 1, 2
NOMBRE_CELDA = {LIBRE: "libre", ESCOMBRO: "escombro", GAS: "gas"}

BLOQUES = [
    (4, 0, 4, 4), (4, 7, 4, 12), (8, 2, 8, 9), (12, 0, 12, 5),
    (12, 8, 12, 12), (16, 3, 16, 12), (2, 2, 2, 3), (6, 10, 7, 10),
    (10, 11, 11, 11), (14, 1, 15, 1),
]
FUGAS = [
    (3, 6), (5, 5), (5, 6), (7, 3), (7, 4), (9, 9), (10, 9),
    (11, 7), (13, 6), (14, 7), (15, 5), (17, 6), (17, 7),
]

ALGORITMOS = ["astar", "dijkstra", "bfs"]
NOMBRES = {"bfs": "BFS", "dijkstra": "Dijkstra", "astar": "A*"}
DESCRIPCIONES = {
    "bfs": "Ruta con menos pasos. Ignora el gas.",
    "dijkstra": "Ruta de menor costo: paso = 1, gas = 5.",
    "astar": "Como Dijkstra, guiado por distancia al superviviente.",
}

# (nombre, celdas que recorre el robot por segundo, nodos explorados por segundo)
VELOCIDADES = [("Pausada", 3.0, 40.0), ("Normal", 6.0, 90.0), ("Rápida", 14.0, 240.0)]

# ---------- Editor ----------
HERRAMIENTAS = {
    "pincel": "Pincel", "linea": "Línea", "rect": "Rect.", "contorno": "Contorno",
    "relleno": "Relleno", "entrada": "Entrada", "meta": "Meta", "gotero": "Gotero",
}
GROSORES = [1, 2, 3]
DENSIDADES = [0.15, 0.25, 0.35]
LIMITE_HISTORIAL = 100
ARCHIVO_MAPA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapa_rescate.json")

# ---------- Geometría de la vista ----------
ANCHO, ALTO = 1240, 760
MAPA_W = 900
PANEL_X = MAPA_W
TW, TH, HZ = 52, 26, 30    # ancho y alto de una celda en pantalla, y altura por unidad
CX, CY = (COLUMNAS - 1) / 2, (FILAS - 1) / 2


# ---------- Colores ----------
def rgb(h: str) -> tuple:
    return tuple(int(h[i:i + 2], 16) for i in (1, 3, 5))

FONDO = rgb("#060d16")
PANEL = rgb("#0c1828")
BORDE = rgb("#22354a")
TEXTO = rgb("#e6eef6")
SUAVE = rgb("#8fa3b8")
CIAN = rgb("#43d9ce")
VERDE = rgb("#83e3a0")
ROJO = rgb("#ff6f80")
NARANJA = rgb("#ffb454")
GRIS_AZUL = rgb("#9fb3c8")
PISO = rgb("#14273d")
PISO_ALT = rgb("#112237")
PISO_RUINA = rgb("#1d2836")
PISO_GAS = rgb("#3a2a1f")
PARED = rgb("#5a6b7e")

COLOR_MATERIAL = {LIBRE: ROJO, ESCOMBRO: GRIS_AZUL, GAS: NARANJA}   # vista previa del editor


def mezcla(a, b, t):
    t = max(0.0, min(1.0, t))
    return tuple(round(a[i] + (b[i] - a[i]) * t) for i in range(3))


def aclara(c, f):
    return tuple(max(0, min(255, round(v * f))) for v in c)


# ---------- Modelo y algoritmos ----------
def mapa_original(con_obstaculos=True):
    """Mapa inicial. Sin obstáculos: mapa completamente libre (sin escombros ni gas)."""
    m = [[LIBRE] * COLUMNAS for _ in range(FILAS)]
    if not con_obstaculos:
        return m
    for x1, y1, x2, y2 in BLOQUES:
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                m[y][x] = ESCOMBRO
    for x, y in FUGAS:
        if m[y][x] == LIBRE:
            m[y][x] = GAS
    return m


def vecinos(mapa, pos):
    x, y = pos
    for nx, ny in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
        if 0 <= nx < COLUMNAS and 0 <= ny < FILAS and mapa[ny][nx] != ESCOMBRO:
            yield (nx, ny)


@dataclass
class Resultado:
    algoritmo: str
    ruta: list
    orden: list
    llego: bool
    pasos: int
    gas: int
    costo: object
    ms: float


def buscar(mapa, algoritmo, inicio=INICIO, objetivo=OBJETIVO):
    inicio_tiempo = time.perf_counter()
    usa_gas = algoritmo != "bfs"

    def estimar(p):
        return abs(p[0] - objetivo[0]) + abs(p[1] - objetivo[1])

    costo = {inicio: 0}
    previo = {}
    cerrados = set()
    orden = []
    desempate = itertools.count()
    frontera = [(estimar(inicio) if algoritmo == "astar" else 0, next(desempate), 0, inicio)]
    llego = False

    while frontera:
        _, _, g, pos = heapq.heappop(frontera)
        if pos in cerrados:
            continue
        cerrados.add(pos)
        orden.append(pos)
        if pos == objetivo:
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

    ruta = []
    if llego:
        p = objetivo
        while True:
            ruta.append(p)
            if p == inicio:
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
        costo=costo[objetivo] if llego else None,
        ms=(time.perf_counter() - inicio_tiempo) * 1000,
    )


# ---------- Utilidades de edición ----------
def linea_celdas(a, b):
    """Celdas de una línea entre a y b (Bresenham)."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    celdas = []
    while True:
        celdas.append((x0, y0))
        if (x0, y0) == (x1, y1):
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return celdas


def celdas_forma(herramienta, a, b):
    """Celdas de una línea, un rectángulo relleno o un contorno entre a y b."""
    if herramienta == "linea":
        return linea_celdas(a, b)
    x1, x2 = sorted((a[0], b[0]))
    y1, y2 = sorted((a[1], b[1]))
    celdas = []
    for y in range(y1, y2 + 1):
        for x in range(x1, x2 + 1):
            if herramienta == "rect" or x in (x1, x2) or y in (y1, y2):
                celdas.append((x, y))
    return celdas


# ---------- Cámara isométrica ----------
class Vista:
    def __init__(self, ox, oy):
        self.ox0, self.oy0 = ox, oy
        self.ox, self.oy = ox, oy
        self.angulo = 0.0          # giro actual en radianes
        self.angulo_obj = 0.0      # giro al que se dirige (animación de Q/E)
        self.inclinacion = 1.0     # 0.35 (casi de lado) .. 1.4 (casi desde arriba)
        self.zoom = 1.0

    def reiniciar(self):
        self.ox, self.oy = self.ox0, self.oy0
        self.angulo_obj = 0.0
        self.inclinacion = 1.0
        self.zoom = 1.0

    def rotar(self, dx, dy):
        c, s = math.cos(self.angulo), math.sin(self.angulo)
        return dx * c - dy * s, dx * s + dy * c

    def proyectar(self, x, y, z=0.0):
        """Convierte coordenadas de celda (esquinas en enteros) a píxeles."""
        dx, dy = self.rotar(x - CX, y - CY)
        s = self.zoom
        altura = HZ * (1.5 - 0.5 * self.inclinacion)
        return (self.ox + (dx - dy) * TW / 2 * s,
                self.oy + (dx + dy) * TH / 2 * s * self.inclinacion - z * altura * s)

    def desproyectar(self, px, py):
        """Inverso de proyectar al nivel del suelo: píxeles -> coordenadas de celda."""
        s = self.zoom
        a = (px - self.ox) / (TW / 2 * s)
        b = (py - self.oy) / (TH / 2 * s * self.inclinacion)
        dx, dy = (a + b) / 2, (b - a) / 2
        c, sn = math.cos(-self.angulo), math.sin(-self.angulo)
        return dx * c - dy * sn + CX, dx * sn + dy * c + CY

    def profundidad(self, x, y):
        """Valor para ordenar: lo que tiene más profundidad se dibuja después."""
        dx, dy = self.rotar(x - CX, y - CY)
        return dx + dy


# Caras laterales de un bloque: (normal, esquina A, esquina B), relativas a la celda
CARAS = [((1, 0), (1, 0), (1, 1)), ((0, 1), (1, 1), (0, 1)),
         ((-1, 0), (0, 1), (0, 0)), ((0, -1), (0, 0), (1, 0))]


# ---------- Botones ----------
class Boton:
    def __init__(self, rect, texto, accion, color=None, activo=None):
        self.rect = pygame.Rect(rect)
        self.texto = texto          # texto fijo o función que devuelve el texto
        self.accion = accion
        self.color = color or rgb("#17293b")
        self.activo = activo        # función opcional: True si el botón está seleccionado

    def dibujar(self, s, fuente):
        act = bool(self.activo and self.activo())
        color = mezcla(self.color, CIAN, 0.3) if act else self.color
        pygame.draw.rect(s, color, self.rect, border_radius=8)
        pygame.draw.rect(s, CIAN if act else BORDE, self.rect,
                         width=2 if act else 1, border_radius=8)
        texto = self.texto() if callable(self.texto) else self.texto
        etiqueta = fuente.render(texto, True, TEXTO)
        s.blit(etiqueta, etiqueta.get_rect(center=self.rect.center))

    def contiene(self, pos):
        return self.rect.collidepoint(pos)


# ---------- Aplicación ----------
class App:
    def __init__(self, pantalla):
        self.pantalla = pantalla
        self.fuente = pygame.font.SysFont("arial", 15)
        self.fuente_chica = pygame.font.SysFont("arial", 13)
        self.fuente_titulo = pygame.font.SysFont("arial", 22, bold=True)
        self.fuente_grande = pygame.font.SysFont("arial", 20, bold=True)
        self.vista = Vista(MAPA_W / 2, 350)
        self.con_obstaculos = True
        self.mapa = mapa_original(self.con_obstaculos)
        self.inicio = INICIO
        self.objetivo = OBJETIVO
        self.tiempo = 0.0
        self.arrastre = None           # arrastre de cámara con el mouse

        # editor
        self.modo_edicion = False
        self.herramienta = "pincel"
        self.material = ESCOMBRO
        self.idx_grosor = 0
        self.idx_dens = 1
        self.sim_h = False
        self.sim_v = False
        self.hover = None              # celda bajo el mouse
        self.trazo = None              # trazo de edición en curso
        self.deshacer_pila = []
        self.rehacer_pila = []

        # simulación
        self.idx_alg = 0
        self.idx_vel = 1
        self.corriendo = False
        self.fase = "listo"            # listo | explorando | ruta | fin
        self.ultimo = None
        self.orden, self.ruta = [], []
        self.visitas = {}
        self.idx_exp = 0.0
        self.idx_ruta = 0
        self.progreso = 0.0
        self.comparacion = []
        self.estado = "Listo"
        self.bitacora = []
        self.resultado = "Elige un algoritmo y pulsa Iniciar."

        self.botones = self._crear_botones()
        self.btn_editor = Boton((PANEL_X + 20, 62, 300, 32), self._texto_editor,
                                self.alternar_editor, rgb("#2a2350"),
                                activo=lambda: self.modo_edicion)
        self.botones_edicion = self._crear_botones_edicion()
        self.registrar("Sistema listo. Mapa de 20 × 13 celdas cargado.")
        self.registrar("Entrada en E, superviviente en S.")

    # ----- interfaz -----
    def _texto_editor(self):
        return "Salir del editor (Tab)" if self.modo_edicion else "Editor de mapa (Tab)"

    def _crear_botones(self):
        x1, x2, w = PANEL_X + 20, PANEL_X + 175, 145
        ancho_total = 300
        return [
            Boton((x1, 136, ancho_total, 32), "", self.ciclar_algoritmo, rgb("#15324a")),  # 0
            Boton((x1, 172, ancho_total, 32), "", self.ciclar_velocidad, rgb("#15324a")),  # 1
            Boton((x1, 208, w, 32), "Iniciar", self.alternar, rgb("#0a7b7d")),             # 2
            Boton((x2, 208, w, 32), "Reiniciar", self.reiniciar_manual),                   # 3
            Boton((x1, 244, w, 32), "Comparar", self.comparar, rgb("#10292d")),            # 4
            Boton((x2, 244, w, 32), "Restablecer", self.restablecer),                      # 5
            Boton((x1, 280, w, 32), "Girar (Q)", lambda: self.girar(-1)),                  # 6
            Boton((x2, 280, w, 32), "Girar (E)", lambda: self.girar(1)),                   # 7
            Boton((x1, 316, w, 32), "Alejar (-)", lambda: self.zoom(-0.08)),               # 8
            Boton((x2, 316, w, 32), "Acercar (+)", lambda: self.zoom(0.08)),               # 9
            Boton((x1, 100, ancho_total, 32), "", self.alternar_modo, rgb("#3a2a1a")),     # 10
        ]

    def _crear_botones_edicion(self):
        x0 = PANEL_X + 20
        lista = []

        def add(x, y, w, h, texto, accion, activo=None):
            lista.append(Boton((x, y, w, h), texto, accion, None, activo))

        # herramientas: 4 columnas x 2 filas
        for i, (clave, nombre) in enumerate(HERRAMIENTAS.items()):
            add(x0 + (i % 4) * 77, 122 + (i // 4) * 34, 69, 28, nombre,
                lambda c=clave: self.elegir_herramienta(c),
                lambda c=clave: self.herramienta == c)
        # materiales
        for i, (valor, nombre) in enumerate([(LIBRE, "1 · Libre"), (ESCOMBRO, "2 · Escombro"),
                                             (GAS, "3 · Gas")]):
            add(x0 + i * 102, 210, 96, 30, nombre,
                lambda v=valor: self.elegir_material(v),
                lambda v=valor: self.material == v)
        # opciones
        add(x0, 270, 147, 28, "Simetría H", self.alternar_sim_h, lambda: self.sim_h)
        add(x0 + 153, 270, 147, 28, "Simetría V", self.alternar_sim_v, lambda: self.sim_v)
        add(x0, 304, 147, 28, lambda: f"Grosor: {GROSORES[self.idx_grosor]}",
            self.ciclar_grosor)
        add(x0 + 153, 304, 147, 28, lambda: f"Densidad: {int(DENSIDADES[self.idx_dens] * 100)}%",
            self.ciclar_densidad)
        # acciones
        add(x0, 364, 147, 28, "Deshacer", self.deshacer)
        add(x0 + 153, 364, 147, 28, "Rehacer", self.rehacer)
        add(x0, 398, 147, 28, "Limpiar todo", self.limpiar)
        add(x0 + 153, 398, 147, 28, "Borde", self.poner_borde)
        add(x0, 432, 147, 28, "Aleatorio", self.aleatorio)
        add(x0 + 153, 432, 147, 28, "Validar ruta", self.validar)
        add(x0, 466, 147, 28, "Guardar", self.guardar_mapa)
        add(x0 + 153, 466, 147, 28, "Cargar", self.cargar_mapa)
        return lista

    def botones_activos(self):
        return [self.btn_editor] + (self.botones_edicion if self.modo_edicion else self.botones)

    def registrar(self, mensaje):
        self.bitacora.append(mensaje)
        self.bitacora = self.bitacora[-5:]

    def _texto(self, s, texto, fuente, color, pos, centro=False, derecha=False):
        superficie = fuente.render(texto, True, color)
        rect = superficie.get_rect()
        if centro:
            rect.center = pos
        elif derecha:
            rect.topright = pos
        else:
            rect.topleft = pos
        s.blit(superficie, rect)

    def _lineas(self, texto, ancho):
        palabras, lineas, actual = texto.split(), [], ""
        for p in palabras:
            prueba = (actual + " " + p).strip()
            if self.fuente.size(prueba)[0] <= ancho:
                actual = prueba
            else:
                lineas.append(actual)
                actual = p
        if actual:
            lineas.append(actual)
        return lineas

    # ----- acciones de control -----
    def ciclar_algoritmo(self):
        self.idx_alg = (self.idx_alg + 1) % len(ALGORITMOS)
        self.reiniciar()
        self.registrar(f"Algoritmo: {NOMBRES[ALGORITMOS[self.idx_alg]]}.")

    def ciclar_velocidad(self):
        self.idx_vel = (self.idx_vel + 1) % len(VELOCIDADES)

    def girar(self, sentido):
        self.vista.angulo_obj += sentido * math.pi / 2

    def zoom(self, delta):
        self.vista.zoom = max(0.4, min(2.0, self.vista.zoom + delta))

    def reiniciar(self):
        self.corriendo = False
        self.fase = "listo"
        self.ultimo = None
        self.orden, self.ruta, self.visitas = [], [], {}
        self.idx_exp = 0.0
        self.idx_ruta = 0
        self.progreso = 0.0
        self.comparacion = []
        self.estado = "Listo"
        self.resultado = "Elige un algoritmo y pulsa Iniciar."
        self.botones[2].texto = "Iniciar"

    def reiniciar_manual(self):
        self.reiniciar()
        self.registrar("Simulación reiniciada.")

    def restablecer(self):
        snap = self._estado()
        self.mapa = mapa_original(self.con_obstaculos)
        self.inicio, self.objetivo = INICIO, OBJETIVO
        self.reiniciar()
        self._confirmar(snap)
        self.registrar("Mapa restablecido al original.")

    def alternar_modo(self):
        snap = self._estado()
        self.con_obstaculos = not self.con_obstaculos
        self.mapa = mapa_original(self.con_obstaculos)
        self.inicio, self.objetivo = INICIO, OBJETIVO
        self.reiniciar()
        self._confirmar(snap)
        self.registrar("Modo sin obstáculos: sin escombros ni gas."
                       if not self.con_obstaculos else "Modo con obstáculos.")

    def nueva_busqueda(self):
        clave = ALGORITMOS[self.idx_alg]
        r = buscar(self.mapa, clave, self.inicio, self.objetivo)
        self.ultimo = r
        self.orden, self.ruta = r.orden, r.ruta
        self.visitas = {pos: i for i, pos in enumerate(r.orden)}
        self.idx_exp = 0.0
        self.idx_ruta = 0
        self.progreso = 0.0
        if r.llego:
            self.resultado = (f"{NOMBRES[clave]} encontró una ruta de {r.pasos} pasos "
                              f"con {r.gas} celda(s) de gas.")
        else:
            self.resultado = f"{NOMBRES[clave]} no encontró ruta: los escombros bloquean el paso."
        if not r.llego:
            self.idx_exp = float(len(r.orden))
            self.fase = "fin"
            self.estado = "Sin ruta"
            self.registrar(f"{NOMBRES[clave]}: no hay ruta hacia el superviviente.")
            return
        self.fase = "explorando"
        self.registrar(f"{NOMBRES[clave]} inicia la búsqueda.")

    def alternar(self):
        if self.corriendo:
            self.corriendo = False
            self.estado = "En pausa"
            self.botones[2].texto = "Reanudar"
            return
        if self.fase in ("listo", "fin"):
            self.nueva_busqueda()
            if self.fase == "fin":
                return
        self.corriendo = True
        self.estado = "Explorando" if self.fase == "explorando" else "Siguiendo la ruta"
        self.botones[2].texto = "Pausar"

    def _terminar(self):
        self.idx_ruta = len(self.ruta) - 1
        self.progreso = 0.0
        self.corriendo = False
        self.fase = "fin"
        self.estado = "Superviviente localizado"
        self.botones[2].texto = "Iniciar"
        self.registrar(f"El robot llegó al superviviente en {self.ultimo.pasos} pasos.")

    def comparar(self):
        self.reiniciar()
        resultados = [buscar(self.mapa, c, self.inicio, self.objetivo) for c in ALGORITMOS]

        def costo_total(r):
            return r.pasos + COSTO_GAS * r.gas if r.llego else math.inf

        recomendado = min(resultados, key=lambda r: (costo_total(r), len(r.orden)))
        self.comparacion = [
            {"nombre": NOMBRES[r.algoritmo], "pasos": r.pasos if r.llego else "—",
             "expl": len(r.orden), "gas": r.gas,
             "costo": costo_total(r) if r.llego else "—",
             "rec": recomendado.llego and r is recomendado}
            for r in resultados
        ]
        self.registrar(f"Comparación lista. Recomendado: {NOMBRES[recomendado.algoritmo]}."
                       if recomendado.llego else "Comparación lista. Ningún algoritmo llegó.")

    # ----- selección de celdas con el mouse -----
    def celda_en(self, px, py):
        """Celda del suelo bajo el píxel (px, py), o None si queda fuera del mapa."""
        fx, fy = self.vista.desproyectar(px, py)
        x, y = math.floor(fx), math.floor(fy)
        if 0 <= x < COLUMNAS and 0 <= y < FILAS:
            return (x, y)
        return None

    def editar_celda(self, px, py):
        """Modo simulación: un clic cicla la celda libre -> escombro -> gas."""
        c = self.celda_en(px, py)
        if c is None or c in (self.inicio, self.objetivo):
            return
        x, y = c
        snap = self._estado()
        self.mapa[y][x] = (self.mapa[y][x] + 1) % 3
        self._confirmar(snap)
        self.registrar(f"Celda ({x}, {y}) cambiada a {NOMBRE_CELDA[self.mapa[y][x]]}.")

    # ----- historial (deshacer / rehacer) -----
    def _estado(self):
        return ([fila[:] for fila in self.mapa], self.inicio, self.objetivo)

    def _restaurar(self, estado):
        mapa, self.inicio, self.objetivo = estado
        self.mapa = [fila[:] for fila in mapa]
        self.reiniciar()

    def _confirmar(self, snap):
        """Guarda snap en el historial si el mapa cambió desde entonces."""
        if self._estado() != snap:
            self.deshacer_pila.append(snap)
            del self.deshacer_pila[:-LIMITE_HISTORIAL]
            self.rehacer_pila.clear()
            self.reiniciar()

    def deshacer(self):
        if not self.deshacer_pila:
            self.registrar("Nada que deshacer.")
            return
        self.rehacer_pila.append(self._estado())
        self._restaurar(self.deshacer_pila.pop())
        self.registrar("Deshacer.")

    def rehacer(self):
        if not self.rehacer_pila:
            self.registrar("Nada que rehacer.")
            return
        self.deshacer_pila.append(self._estado())
        self._restaurar(self.rehacer_pila.pop())
        self.registrar("Rehacer.")

    # ----- editor de mapa -----
    def alternar_editor(self):
        self.modo_edicion = not self.modo_edicion
        self.trazo = None
        self.reiniciar()
        self.registrar("Editor de mapa activado." if self.modo_edicion
                       else "Editor de mapa cerrado.")

    def elegir_herramienta(self, clave):
        self.herramienta = clave

    def elegir_material(self, valor):
        self.material = valor
        if self.herramienta in ("entrada", "meta", "gotero"):
            self.herramienta = "pincel"

    def alternar_sim_h(self):
        self.sim_h = not self.sim_h

    def alternar_sim_v(self):
        self.sim_v = not self.sim_v

    def ciclar_grosor(self):
        self.idx_grosor = (self.idx_grosor + 1) % len(GROSORES)

    def ciclar_densidad(self):
        self.idx_dens = (self.idx_dens + 1) % len(DENSIDADES)

    def celdas_afectadas(self, base, usar_grosor, simetria=True):
        """Expande celdas base con el grosor del pincel y la simetría; quita entrada/meta."""
        g = GROSORES[self.idx_grosor] if usar_grosor else 1
        desde = -(g // 2)
        res = set()
        for x, y in base:
            for ddx in range(desde, desde + g):
                for ddy in range(desde, desde + g):
                    res.add((x + ddx, y + ddy))
        if simetria and (self.sim_h or self.sim_v):
            extra = set()
            for x, y in res:
                if self.sim_h:
                    extra.add((COLUMNAS - 1 - x, y))
                if self.sim_v:
                    extra.add((x, FILAS - 1 - y))
                if self.sim_h and self.sim_v:
                    extra.add((COLUMNAS - 1 - x, FILAS - 1 - y))
            res |= extra
        protegidas = (self.inicio, self.objetivo)
        return {c for c in res
                if 0 <= c[0] < COLUMNAS and 0 <= c[1] < FILAS and c not in protegidas}

    def _pintar(self, celdas, valor):
        for x, y in celdas:
            self.mapa[y][x] = valor

    def _huella(self, herramienta, base):
        """Celdas que se verían afectadas por la herramienta (para la vista previa)."""
        if herramienta in ("pincel", "linea"):
            return self.celdas_afectadas(base, True)
        if herramienta in ("rect", "contorno", "relleno"):
            return self.celdas_afectadas(base, False)
        return {c for c in base if 0 <= c[0] < COLUMNAS and 0 <= c[1] < FILAS}

    def rellenar(self, inicio, valor):
        x0, y0 = inicio
        original = self.mapa[y0][x0]
        if original == valor:
            return
        pila, vistos = [inicio], {inicio}
        while pila:
            x, y = pila.pop()
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if (0 <= nx < COLUMNAS and 0 <= ny < FILAS and (nx, ny) not in vistos
                        and self.mapa[ny][nx] == original):
                    vistos.add((nx, ny))
                    pila.append((nx, ny))
        self._pintar(self.celdas_afectadas(vistos, False), valor)

    def iniciar_trazo(self, pos):
        c = self.celda_en(*pos)
        if c is None:
            return
        h = self.herramienta
        borrar = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
        valor = LIBRE if borrar else self.material
        snap = self._estado()
        x, y = c

        if h == "gotero":
            self.material = self.mapa[y][x]
            self.registrar(f"Gotero: material {NOMBRE_CELDA[self.material]}.")
            return
        if h == "relleno":
            self.rellenar(c, valor)
            self._confirmar(snap)
            return
        if h in ("entrada", "meta"):
            otro = self.objetivo if h == "entrada" else self.inicio
            if c == otro:
                return
            self.mapa[y][x] = LIBRE
            if h == "entrada":
                self.inicio = c
            else:
                self.objetivo = c
            self._confirmar(snap)
            self.registrar(f"{'Entrada' if h == 'entrada' else 'Superviviente'} movido a ({x}, {y}).")
            return

        self.trazo = {"herr": h, "ini": c, "ult": c, "valor": valor, "snap": snap}
        if h == "pincel":
            self._pintar(self.celdas_afectadas([c], True), valor)

    def mover_trazo(self, c):
        t = self.trazo
        if t is None or c is None or c == t["ult"]:
            return
        if t["herr"] == "pincel":
            self._pintar(self.celdas_afectadas(linea_celdas(t["ult"], c), True), t["valor"])
        t["ult"] = c

    def terminar_trazo(self, pos):
        t = self.trazo
        if t is None:
            return
        if pos[0] < PANEL_X:
            self.mover_trazo(self.celda_en(*pos))
        h = t["herr"]
        if h in ("linea", "rect", "contorno"):
            base = celdas_forma(h, t["ini"], t["ult"])
            self._pintar(self.celdas_afectadas(base, h == "linea"), t["valor"])
        self._confirmar(t["snap"])
        self.trazo = None

    def vista_previa(self):
        """(celdas, color) que se resaltan sobre el mapa en modo editor."""
        h = self.herramienta
        borrar = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
        valor = LIBRE if borrar else self.material
        if h == "entrada":
            color = CIAN
        elif h == "meta":
            color = VERDE
        elif h == "gotero":
            color = TEXTO
        else:
            color = COLOR_MATERIAL[valor]
        if self.trazo and h in ("linea", "rect", "contorno"):
            base = celdas_forma(h, self.trazo["ini"], self.trazo["ult"])
        elif self.hover:
            base = [self.hover]
        else:
            return set(), color
        if self.trazo:
            color = COLOR_MATERIAL[self.trazo["valor"]]
        return self._huella(h, base), color

    def limpiar(self):
        snap = self._estado()
        self.mapa = [[LIBRE] * COLUMNAS for _ in range(FILAS)]
        self._confirmar(snap)
        self.registrar("Mapa limpiado.")

    def poner_borde(self):
        snap = self._estado()
        borde = {(x, y) for x in range(COLUMNAS) for y in range(FILAS)
                 if x in (0, COLUMNAS - 1) or y in (0, FILAS - 1)}
        self._pintar(self.celdas_afectadas(borde, False, simetria=False), self.material)
        self._confirmar(snap)
        self.registrar(f"Borde del mapa: {NOMBRE_CELDA[self.material]}.")

    def aleatorio(self):
        snap = self._estado()
        d = DENSIDADES[self.idx_dens]
        garantizado = False
        for _ in range(40):
            m = [[LIBRE] * COLUMNAS for _ in range(FILAS)]
            for y in range(FILAS):
                for x in range(COLUMNAS):
                    if (x, y) in (self.inicio, self.objetivo):
                        continue
                    r = random.random()
                    if r < d:
                        m[y][x] = ESCOMBRO
                    elif r < d + 0.06:
                        m[y][x] = GAS
            if buscar(m, "bfs", self.inicio, self.objetivo).llego:
                garantizado = True
                break
        self.mapa = m
        self._confirmar(snap)
        self.registrar("Mapa aleatorio con ruta posible." if garantizado
                       else "Mapa aleatorio (sin ruta garantizada).")

    def validar(self):
        r = buscar(self.mapa, "bfs", self.inicio, self.objetivo)
        if r.llego:
            self.registrar(f"Hay ruta: {r.pasos} pasos mínimos hasta el superviviente.")
        else:
            self.registrar("Sin ruta: los escombros bloquean el paso.")

    def guardar_mapa(self):
        datos = {"columnas": COLUMNAS, "filas": FILAS, "mapa": self.mapa,
                 "inicio": list(self.inicio), "objetivo": list(self.objetivo)}
        try:
            with open(ARCHIVO_MAPA, "w", encoding="utf-8") as f:
                json.dump(datos, f)
            self.registrar("Mapa guardado en mapa_rescate.json.")
        except OSError as e:
            self.registrar(f"No se pudo guardar: {e.strerror or e}")

    def cargar_mapa(self):
        try:
            with open(ARCHIVO_MAPA, encoding="utf-8") as f:
                d = json.load(f)
            mapa = d["mapa"]
            ini, obj = tuple(d["inicio"]), tuple(d["objetivo"])
            ok = (d["columnas"] == COLUMNAS and d["filas"] == FILAS and len(mapa) == FILAS
                  and all(len(fila) == COLUMNAS and all(v in (0, 1, 2) for v in fila)
                          for fila in mapa)
                  and 0 <= ini[0] < COLUMNAS and 0 <= ini[1] < FILAS
                  and 0 <= obj[0] < COLUMNAS and 0 <= obj[1] < FILAS and ini != obj)
            if not ok:
                raise ValueError("formato no válido")
        except FileNotFoundError:
            self.registrar("No hay mapa guardado todavía.")
            return
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.registrar(f"No se pudo cargar: {e}")
            return
        snap = self._estado()
        self.mapa = [fila[:] for fila in mapa]
        self.inicio, self.objetivo = ini, obj
        self.mapa[ini[1]][ini[0]] = LIBRE
        self.mapa[obj[1]][obj[0]] = LIBRE
        self._confirmar(snap)
        self.registrar("Mapa cargado.")

    # ----- eventos -----
    def evento(self, ev):
        v = self.vista
        if ev.type == pygame.MOUSEBUTTONDOWN:
            if ev.pos[0] >= PANEL_X:
                if ev.button == 1:
                    for b in self.botones_activos():
                        if b.contiene(ev.pos):
                            b.accion()
                            break
            elif ev.button == 1 and self.modo_edicion:
                self.iniciar_trazo(ev.pos)
            elif ev.button in (1, 2, 3):
                self.arrastre = {"btn": ev.button, "ini": ev.pos,
                                 "ult": ev.pos, "movido": False}
        elif ev.type == pygame.MOUSEMOTION and self.arrastre:
            a = self.arrastre
            dx, dy = ev.pos[0] - a["ult"][0], ev.pos[1] - a["ult"][1]
            a["ult"] = ev.pos
            if not a["movido"] and math.hypot(ev.pos[0] - a["ini"][0],
                                              ev.pos[1] - a["ini"][1]) > 5:
                a["movido"] = True
            if a["movido"]:
                if a["btn"] == 2:                      # clic central: mover
                    v.ox += dx
                    v.oy += dy
                else:                                  # girar e inclinar
                    v.angulo += dx * 0.01
                    v.angulo_obj = v.angulo
                    v.inclinacion = max(0.35, min(1.4, v.inclinacion - dy * 0.005))
        elif ev.type == pygame.MOUSEBUTTONUP:
            if ev.button == 1 and self.trazo:
                self.terminar_trazo(ev.pos)
            elif self.arrastre and ev.button == self.arrastre["btn"]:
                if ev.button == 1 and not self.arrastre["movido"]:
                    self.editar_celda(*ev.pos)         # clic simple: editar celda
                self.arrastre = None
        elif ev.type == pygame.MOUSEWHEEL:
            self.zoom(0.06 * ev.y)
        elif ev.type == pygame.KEYDOWN:
            self._tecla(ev)

    def _tecla(self, ev):
        k = ev.key
        v = self.vista
        if ev.mod & (pygame.KMOD_CTRL | pygame.KMOD_META):      # atajos con Ctrl
            if self.modo_edicion:
                if k == pygame.K_z:
                    self.rehacer() if ev.mod & pygame.KMOD_SHIFT else self.deshacer()
                elif k == pygame.K_y:
                    self.rehacer()
                elif k == pygame.K_s:
                    self.guardar_mapa()
                elif k == pygame.K_l:
                    self.cargar_mapa()
            return
        # teclas comunes
        if k == pygame.K_TAB:
            self.alternar_editor()
        elif k == pygame.K_q:
            self.girar(-1)
        elif k == pygame.K_e:
            self.girar(1)
        elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.zoom(0.08)
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.zoom(-0.08)
        elif k == pygame.K_v:
            v.reiniciar()
        elif self.modo_edicion:
            if k in (pygame.K_1, pygame.K_KP1):
                self.elegir_material(LIBRE)
            elif k in (pygame.K_2, pygame.K_KP2):
                self.elegir_material(ESCOMBRO)
            elif k in (pygame.K_3, pygame.K_KP3):
                self.elegir_material(GAS)
            elif k == pygame.K_b:
                self.elegir_herramienta("pincel")
            elif k == pygame.K_l:
                self.elegir_herramienta("linea")
            elif k == pygame.K_f:
                self.elegir_herramienta("relleno")
            elif k == pygame.K_g:
                self.elegir_herramienta("gotero")
        else:
            if k == pygame.K_SPACE:
                self.alternar()
            elif k == pygame.K_o:
                self.alternar_modo()
            elif k == pygame.K_r:
                self.reiniciar_manual()
            elif k == pygame.K_c:
                self.comparar()

    # ----- actualización por cuadro -----
    def posicion_robot(self):
        if self.fase in ("ruta", "fin") and self.ruta:
            i = min(self.idx_ruta, len(self.ruta) - 1)
            if i >= len(self.ruta) - 1:
                return self.ruta[-1]
            a, b = self.ruta[i], self.ruta[i + 1]
            t = self.progreso
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        return self.inicio

    def actualizar(self, dt):
        self.tiempo += dt
        v = self.vista
        v.angulo += (v.angulo_obj - v.angulo) * min(1.0, dt * 10)

        if self.modo_edicion:
            mx, my = pygame.mouse.get_pos()
            self.hover = self.celda_en(mx, my) if mx < PANEL_X else None
            if self.trazo:
                self.mover_trazo(self.hover)

        _, celdas_s, nodos_s = VELOCIDADES[self.idx_vel]
        if not self.corriendo:
            return
        if self.fase == "explorando":
            self.idx_exp += nodos_s * dt
            if self.idx_exp >= len(self.orden):
                self.idx_exp = float(len(self.orden))
                self.fase = "ruta"
                self.estado = "Siguiendo la ruta"
                self.registrar(f"Exploración completa: {len(self.orden)} nodos revisados.")
        elif self.fase == "ruta":
            self.progreso += celdas_s * dt
            while self.progreso >= 1 and self.idx_ruta < len(self.ruta) - 1:
                self.progreso -= 1
                self.idx_ruta += 1
            if self.idx_ruta >= len(self.ruta) - 1:
                self._terminar()

    # ----- dibujo del mapa 3D -----
    def _diamante(self, x, y):
        return [self.vista.proyectar(x, y), self.vista.proyectar(x + 1, y),
                self.vista.proyectar(x + 1, y + 1), self.vista.proyectar(x, y + 1)]

    def _glow(self, s, centro, radio, color, alfa):
        if radio < 1:
            return
        capa = pygame.Surface((radio * 2, radio * 2), pygame.SRCALPHA)
        pygame.draw.circle(capa, (*color, max(0, min(255, int(alfa)))), (radio, radio), radio)
        s.blit(capa, (centro[0] - radio, centro[1] - radio))

    def _resaltar(self, s, celdas, color, alfa=90):
        if not celdas:
            return
        capa = pygame.Surface((MAPA_W, ALTO), pygame.SRCALPHA)
        for x, y in celdas:
            pts = self._diamante(x, y)
            pygame.draw.polygon(capa, (*color, alfa), pts)
            pygame.draw.polygon(capa, (*color, 230), pts, 2)
        s.blit(capa, (0, 0))

    def _caja(self, s, x, y, h, color):
        base = (x, y)
        for (nx, ny), (ax, ay), (bx, by) in CARAS:
            rnx, rny = self.vista.rotar(nx, ny)
            if rnx + rny <= 0:          # cara orientada hacia el lado contrario al espectador
                continue
            cara = [
                self.vista.proyectar(base[0] + ax, base[1] + ay, 0),
                self.vista.proyectar(base[0] + bx, base[1] + by, 0),
                self.vista.proyectar(base[0] + bx, base[1] + by, h),
                self.vista.proyectar(base[0] + ax, base[1] + ay, h),
            ]
            factor = 0.82 if abs(rnx) > abs(rny) else 0.6
            pygame.draw.polygon(s, aclara(color, factor), cara)
            pygame.draw.polygon(s, aclara(color, factor * 0.7), cara, 1)
        arriba = [self.vista.proyectar(x, y, h), self.vista.proyectar(x + 1, y, h),
                  self.vista.proyectar(x + 1, y + 1, h), self.vista.proyectar(x, y + 1, h)]
        pygame.draw.polygon(s, aclara(color, 1.25), arriba)
        pygame.draw.polygon(s, aclara(color, 1.5), arriba, 1)

    def dibujar_mapa(self, s):
        s.fill(FONDO)
        pygame.draw.rect(s, rgb("#0a1522"), (0, 0, MAPA_W, ALTO))
        total = max(1, len(self.orden))
        ahora = self.tiempo

        # Suelo
        for y in range(FILAS):
            for x in range(COLUMNAS):
                tipo = self.mapa[y][x]
                base = PISO if (x + y) % 2 == 0 else PISO_ALT
                if tipo == ESCOMBRO:
                    base = PISO_RUINA
                elif tipo == GAS:
                    base = PISO_GAS
                idx = self.visitas.get((x, y))
                if tipo == LIBRE and idx is not None and idx < int(self.idx_exp):
                    base = mezcla(base, CIAN, 0.4 - 0.25 * idx / total)
                pts = self._diamante(x, y)
                pygame.draw.polygon(s, base, pts)
                pygame.draw.polygon(s, rgb("#1a2c40"), pts, 1)

                if tipo == GAS:
                    cx, cy = self.vista.proyectar(x + 0.5, y + 0.5, 0)
                    z_esc = self.vista.zoom
                    pulso = 0.5 + 0.5 * math.sin(ahora * 2.2 + x * 0.7 + y)
                    self._glow(s, (cx, cy), int(30 * z_esc + 6 * pulso), NARANJA, 40 + 40 * pulso)
                    for k in range(3):   # partículas de humo que suben
                        fase = (ahora * 0.5 + k / 3 + (x * 13 + y * 7) % 7 / 7) % 1.0
                        px, py = self.vista.proyectar(x + 0.5, y + 0.5, fase * 1.8)
                        self._glow(s, (px, py), int(6 * z_esc + 2), rgb("#ffb878"), 110 * (1 - fase))

        # Ruta final (sobre el suelo, bajo las paredes)
        if self.fase in ("ruta", "fin") and self.ruta:
            puntos = [self.vista.proyectar(x + 0.5, y + 0.5, 0.25)
                      for x, y in self.ruta[: self.idx_ruta + 1]]
            if len(puntos) >= 2:
                pygame.draw.lines(s, rgb("#5a1f2b"), False, puntos, 9)
                pygame.draw.lines(s, ROJO, False, puntos, 4)

        # Entrada
        ex, ey = self.vista.proyectar(self.inicio[0] + 0.5, self.inicio[1] + 0.5, 0.05)
        pygame.draw.polygon(s, CIAN, self._diamante(*self.inicio), 2)
        self._texto(s, "E", self.fuente_chica, CIAN, (ex, ey), centro=True)

        # Objetos ordenados por profundidad: paredes, superviviente y robot
        objetos = []
        for y in range(FILAS):
            for x in range(COLUMNAS):
                if self.mapa[y][x] == ESCOMBRO:
                    objetos.append((self.vista.profundidad(x + 0.5, y + 0.5), "pared", (x, y)))
        objetos.append((self.vista.profundidad(self.objetivo[0] + 0.5, self.objetivo[1] + 0.5),
                        "meta", self.objetivo))
        rx, ry = self.posicion_robot()
        objetos.append((self.vista.profundidad(rx + 0.5, ry + 0.5) + 0.05, "robot", (rx, ry)))
        objetos.sort(key=lambda o: o[0])

        for _, tipo, dato in objetos:
            if tipo == "pared":
                x, y = dato
                h = 1.4 + ((x * 7 + y * 13) % 5) * 0.22
                self._caja(s, x, y, h, PARED)
            elif tipo == "meta":
                x, y = dato
                pulso = 0.5 + 0.5 * math.sin(ahora * 3)
                self._caja(s, x + 0.2, y + 0.2, 1.3, rgb("#2a7a52"))
                cx, cy = self.vista.proyectar(x + 0.5, y + 0.5, 1.8)
                self._glow(s, (cx, cy), int(16 + 6 * pulso), VERDE, 70)
                self._texto(s, "S", self.fuente_grande, VERDE, (cx, cy - 10), centro=True)
                bx, by = self.vista.proyectar(x + 0.5, y + 0.5, 0)
                tope = self.vista.proyectar(x + 0.5, y + 0.5, 6)
                pygame.draw.line(s, rgb("#2f8f62"), (bx, by), tope, 2)
            elif tipo == "robot":
                self._dibujar_robot(s, dato[0] + 0.5, dato[1] + 0.5, ahora)

        # Vista previa del editor
        if self.modo_edicion:
            celdas, color = self.vista_previa()
            self._resaltar(s, celdas, color)

        # Indicaciones
        self._texto(s, "Edificio colapsado", self.fuente_titulo, TEXTO, (20, 16))
        if self.modo_edicion:
            self._texto(s, f"Editor · {HERRAMIENTAS[self.herramienta]} · "
                           f"{NOMBRE_CELDA[self.material]}", self.fuente, NARANJA, (20, 46))
            ayuda = "Izq: dibujar (Mayús = borrar) · Der: girar · Central: mover · Rueda: zoom · Tab: salir"
        else:
            self._texto(s, self.estado, self.fuente, VERDE, (20, 46))
            ayuda = "Clic: editar · Arrastrar: girar · Clic central: mover · Rueda: zoom · V: centrar"
        self._texto(s, ayuda, self.fuente_chica, SUAVE, (20, ALTO - 28))

    def _dibujar_robot(self, s, rx, ry, ahora):
        z = self.vista.zoom
        sx, sy = self.vista.proyectar(rx, ry, 0)
        pygame.draw.ellipse(s, rgb("#05090f"),
                            (sx - 16 * z, sy - 7 * z, 32 * z, 14 * z))
        cx, cy = self.vista.proyectar(rx, ry, 0.8 + 0.08 * math.sin(ahora * 4))
        r = max(4, int(13 * z))
        self._glow(s, (cx, cy), r + 8, CIAN, 60)
        pygame.draw.circle(s, aclara(CIAN, 0.7), (cx, cy), r)
        pygame.draw.circle(s, CIAN, (cx - r * 0.2, cy - r * 0.2), int(r * 0.75))
        pygame.draw.circle(s, rgb("#d9fff9"), (cx - r * 0.35, cy - r * 0.35), max(2, int(r * 0.25)))

    # ----- panel -----
    def dibujar_panel(self, s):
        pygame.draw.rect(s, PANEL, (PANEL_X, 0, ANCHO - PANEL_X, ALTO))
        pygame.draw.line(s, BORDE, (PANEL_X, 0), (PANEL_X, ALTO), 2)
        self._texto(s, "Robot de rescate", self.fuente_titulo, TEXTO, (PANEL_X + 20, 14))
        self._texto(s, "Simulador 3D · BFS, Dijkstra y A*", self.fuente_chica, SUAVE, (PANEL_X + 20, 46))

        # Textos dentro de los botones de selección
        self.botones[0].texto = f"Algoritmo: {NOMBRES[ALGORITMOS[self.idx_alg]]}"
        self.botones[1].texto = f"Velocidad: {VELOCIDADES[self.idx_vel][0]}"
        self.botones[10].texto = "Modo: " + ("Con obstáculos" if self.con_obstaculos else "Sin obstáculos")
        for b in self.botones_activos():
            b.dibujar(s, self.fuente)

        if self.modo_edicion:
            self._panel_edicion(s)
        else:
            self._panel_simulacion(s)

        # Bitácora
        y = 680
        pygame.draw.line(s, BORDE, (PANEL_X + 20, y - 8), (ANCHO - 20, y - 8), 1)
        self._texto(s, "Bitácora", self.fuente, TEXTO, (PANEL_X + 20, y))
        y += 24
        for linea in self.bitacora[-3:]:
            self._texto(s, "› " + linea, self.fuente_chica, SUAVE, (PANEL_X + 20, y))
            y += 17

    def _panel_simulacion(self, s):
        self._texto(s, DESCRIPCIONES[ALGORITMOS[self.idx_alg]], self.fuente_chica, SUAVE,
                    (PANEL_X + 20, 352))

        # Indicadores
        r = self.ultimo
        filas = [
            ("Pasos de la ruta", str(r.pasos) if r and r.llego else "—"),
            ("Nodos explorados", str(len(self.orden)) if r else "—"),
            ("Celdas con gas", str(r.gas) if r else "—"),
            ("Costo total", str(r.costo) if r and r.llego else "—"),
            ("Tiempo de cálculo", f"{r.ms:.2f} ms" if r else "—"),
        ]
        y = 378
        for etiqueta, valor in filas:
            self._texto(s, etiqueta, self.fuente, SUAVE, (PANEL_X + 20, y))
            self._texto(s, valor, self.fuente, TEXTO, (ANCHO - 20, y), derecha=True)
            y += 20
        y += 6
        for linea in self._lineas(self.resultado, 300)[:3]:
            self._texto(s, linea, self.fuente_chica, SUAVE, (PANEL_X + 20, y))
            y += 18

        # Comparación
        y = 548
        if self.comparacion:
            self._texto(s, "Comparación", self.fuente, TEXTO, (PANEL_X + 20, y))
            y += 24
            cabecera = ["Alg.", "Pasos", "Expl.", "Gas", "Costo"]
            xs = [PANEL_X + 20, PANEL_X + 90, PANEL_X + 160, PANEL_X + 230, PANEL_X + 290]
            for c, x in zip(cabecera, xs):
                self._texto(s, c, self.fuente_chica, SUAVE, (x, y))
            y += 20
            for fila in self.comparacion:
                color = VERDE if fila["rec"] else TEXTO
                valores = [fila["nombre"] + (" ★" if fila["rec"] else ""),
                           str(fila["pasos"]), str(fila["expl"]), str(fila["gas"]), str(fila["costo"])]
                for v, x in zip(valores, xs):
                    self._texto(s, v, self.fuente_chica, color, (x, y))
                y += 19
            self._texto(s, "★ menor costo total (pasos + 5 × gas)", self.fuente_chica, SUAVE,
                        (PANEL_X + 20, y))

    def _panel_edicion(self, s):
        x = PANEL_X + 20
        for texto, y in (("Herramienta", 102), ("Material", 192),
                         ("Opciones", 252), ("Acciones", 346)):
            self._texto(s, texto, self.fuente_chica, SUAVE, (x, y))

        # Información
        y = 510
        if self.hover:
            hx, hy = self.hover
            info = f"Celda ({hx}, {hy}) · {NOMBRE_CELDA[self.mapa[hy][hx]]}"
            if self.hover == self.inicio:
                info += " · entrada"
            elif self.hover == self.objetivo:
                info += " · superviviente"
        else:
            info = "Celda: —"
        self._texto(s, info, self.fuente, TEXTO, (x, y))
        y += 22
        total = COLUMNAS * FILAS
        esc = sum(f.count(ESCOMBRO) for f in self.mapa)
        gas = sum(f.count(GAS) for f in self.mapa)
        self._texto(s, f"Escombros: {esc} · Gas: {gas} · Libres: {total - esc - gas}",
                    self.fuente, SUAVE, (x, y))
        y += 28
        ayudas = ["Izq: dibujar (Mayús = borrar)",
                  "Der: girar · Central: mover · Rueda: zoom",
                  "1/2/3 material · B L F G herramientas",
                  "Ctrl+Z / Ctrl+Y deshacer y rehacer",
                  "Ctrl+S / Ctrl+L guardar y cargar"]
        for linea in ayudas:
            self._texto(s, linea, self.fuente_chica, SUAVE, (x, y))
            y += 18


def main():
    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO, ALTO))
    pygame.display.set_caption("Robot de rescate · simulador 3D")
    app = App(pantalla)
    reloj = pygame.time.Clock()
    ejecutando = True
    while ejecutando:
        dt = min(0.05, reloj.tick(60) / 1000)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                ejecutando = False
            else:
                app.evento(ev)
        app.actualizar(dt)
        app.dibujar_mapa(pantalla)
        app.dibujar_panel(pantalla)
        pygame.display.flip()
    pygame.quit()


if __name__ == "__main__":
    main()