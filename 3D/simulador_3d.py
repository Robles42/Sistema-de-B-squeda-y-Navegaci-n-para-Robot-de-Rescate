"""
Simulador 3D de búsqueda para robot de rescate (vista isométrica).

Compara BFS, Dijkstra y A* sobre un edificio con escombros y fugas de gas.

Requisitos: Python 3.8 o superior y pygame.
    pip install pygame

Ejecución:
    python simulador_3d.py

Controles:
    Clic en el mapa   editar celda (libre -> escombro -> gas -> libre)
    Espacio           iniciar / pausar / reanudar
    Q / E             girar la cámara 90 grados
    + / -  o rueda    acercar / alejar
    R                 reiniciar
    C                 comparar algoritmos
"""
from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

import pygame

COLUMNAS, FILAS = 20, 13
INICIO = (1, 6)            
OBJETIVO = (18, 6)         
COSTO_GAS = 5             

LIBRE, ESCOMBRO, GAS = 0, 1, 2

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

VELOCIDADES = [("Pausada", 3.0, 40.0), ("Normal", 6.0, 90.0), ("Rápida", 14.0, 240.0)]

ANCHO, ALTO = 1240, 760
MAPA_W = 900
PANEL_X = MAPA_W
TW, TH, HZ = 52, 26, 30    # ancho y alto de una celda en pana,tall y altura por unidad
CX, CY = (COLUMNAS - 1) / 2, (FILAS - 1) / 2

#  Colores 
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
PISO = rgb("#14273d")
PISO_ALT = rgb("#112237")
PISO_RUINA = rgb("#1d2836")
PISO_GAS = rgb("#3a2a1f")
PARED = rgb("#5a6b7e")


def mezcla(a, b, t):
    t = max(0.0, min(1.0, t))
    return tuple(round(a[i] + (b[i] - a[i]) * t) for i in range(3))


def aclara(c, f):
    return tuple(max(0, min(255, round(v * f))) for v in c)


#  Modelo y algoritmos 
def mapa_original():
    m = [[LIBRE] * COLUMNAS for _ in range(FILAS)]
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


def buscar(mapa, algoritmo):
    inicio_tiempo = time.perf_counter()
    usa_gas = algoritmo != "bfs"

    def estimar(p):
        return abs(p[0] - OBJETIVO[0]) + abs(p[1] - OBJETIVO[1])

    costo = {INICIO: 0}
    previo = {}
    cerrados = set()
    orden = []
    desempate = itertools.count()
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

    ruta = []
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


#  Cámara isométrica 
class Vista:
    def __init__(self, ox, oy):
        self.ox, self.oy = ox, oy
        self.giro = 0       # 0..3, cada uno son 90 grados
        self.zoom = 1.0

    def rotar(self, dx, dy):
        k = self.giro
        if k == 1:
            return -dy, dx
        if k == 2:
            return -dx, -dy
        if k == 3:
            return dy, -dx
        return dx, dy

    def proyectar(self, x, y, z=0.0):
        """Convierte coordenadas de celda (esquinas en enteros) a píxeles."""
        dx, dy = self.rotar(x - CX, y - CY)
        s = self.zoom
        return (self.ox + (dx - dy) * TW / 2 * s,
                self.oy + (dx + dy) * TH / 2 * s - z * HZ * s)

    def profundidad(self, x, y):
        """Valor para ordenar: lo que tiene más profundidad se dibuja después."""
        dx, dy = self.rotar(x - CX, y - CY)
        return dx + dy


def punto_en_poligono(p, pts):
    signos = []
    for i in range(len(pts)):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % len(pts)]
        signos.append((bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax))
    return all(s >= 0 for s in signos) or all(s <= 0 for s in signos)


# Caras laterales de un bloque: (normal, esquina A, esquina B), relativas a la celda
CARAS = [((1, 0), (1, 0), (1, 1)), ((0, 1), (1, 1), (0, 1)),
         ((-1, 0), (0, 1), (0, 0)), ((0, -1), (0, 0), (1, 0))]


#  Botones 
class Boton:
    def __init__(self, rect, texto, accion, color=None):
        self.rect = pygame.Rect(rect)
        self.texto = texto
        self.accion = accion
        self.color = color or rgb("#17293b")

    def dibujar(self, s, fuente):
        pygame.draw.rect(s, self.color, self.rect, border_radius=8)
        pygame.draw.rect(s, BORDE, self.rect, width=1, border_radius=8)
        etiqueta = fuente.render(self.texto, True, TEXTO)
        s.blit(etiqueta, etiqueta.get_rect(center=self.rect.center))

    def contiene(self, pos):
        return self.rect.collidepoint(pos)


#  Aplicacion 
class App:
    def __init__(self, pantalla):
        self.pantalla = pantalla
        self.fuente = pygame.font.SysFont("arial", 15)
        self.fuente_chica = pygame.font.SysFont("arial", 13)
        self.fuente_titulo = pygame.font.SysFont("arial", 22, bold=True)
        self.fuente_grande = pygame.font.SysFont("arial", 20, bold=True)
        self.vista = Vista(MAPA_W / 2, 350)
        self.mapa = mapa_original()
        self.tiempo = 0.0

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
        self.registrar("Sistema listo. Mapa de 20 × 13 celdas cargado.")
        self.registrar("Entrada en E, superviviente en S.")

    #  interfaz 
    def _crear_botones(self):
        x1, x2, w = PANEL_X + 20, PANEL_X + 175, 145
        ancho_total = 300
        return [
            Boton((x1, 100, ancho_total, 34), "", self.ciclar_algoritmo, rgb("#15324a")),
            Boton((x1, 142, ancho_total, 34), "", self.ciclar_velocidad, rgb("#15324a")),
            Boton((x1, 184, w, 34), "Iniciar", self.alternar, rgb("#0a7b7d")),
            Boton((x2, 184, w, 34), "Reiniciar", self.reiniciar_manual),
            Boton((x1, 226, w, 34), "Comparar", self.comparar, rgb("#10292d")),
            Boton((x2, 226, w, 34), "Restablecer", self.restablecer),
            Boton((x1, 268, w, 34), "Girar (Q)", lambda: self.girar(-1)),
            Boton((x2, 268, w, 34), "Girar (E)", lambda: self.girar(1)),
            Boton((x1, 310, w, 34), "Alejar (-)", lambda: self.zoom(-0.08)),
            Boton((x2, 310, w, 34), "Acercar (+)", lambda: self.zoom(0.08)),
        ]

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

    #  acciones de control 
    def ciclar_algoritmo(self):
        self.idx_alg = (self.idx_alg + 1) % len(ALGORITMOS)
        self.reiniciar()
        self.registrar(f"Algoritmo: {NOMBRES[ALGORITMOS[self.idx_alg]]}.")

    def ciclar_velocidad(self):
        self.idx_vel = (self.idx_vel + 1) % len(VELOCIDADES)

    def girar(self, sentido):
        self.vista.giro = (self.vista.giro + sentido) % 4

    def zoom(self, delta):
        self.vista.zoom = max(0.6, min(1.05, self.vista.zoom + delta))

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
        self.mapa = mapa_original()
        self.reiniciar()
        self.registrar("Mapa restablecido al original.")

    def nueva_busqueda(self):
        clave = ALGORITMOS[self.idx_alg]
        r = buscar(self.mapa, clave)
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
        resultados = [buscar(self.mapa, c) for c in ALGORITMOS]

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

    def editar_celda(self, px, py):
        for y in range(FILAS):
            for x in range(COLUMNAS):
                pts = self._diamante(x, y)
                if punto_en_poligono((px, py), pts):
                    if (x, y) in (INICIO, OBJETIVO):
                        return
                    self.reiniciar()
                    self.mapa[y][x] = (self.mapa[y][x] + 1) % 3
                    nombre = {LIBRE: "libre", ESCOMBRO: "escombro", GAS: "gas"}[self.mapa[y][x]]
                    self.registrar(f"Celda ({x}, {y}) cambiada a {nombre}.")
                    return

    #  eventos 
    def evento(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if ev.pos[0] >= PANEL_X:
                for b in self.botones:
                    if b.contiene(ev.pos):
                        b.accion()
                        break
            else:
                self.editar_celda(*ev.pos)
        elif ev.type == pygame.MOUSEWHEEL:
            self.zoom(0.06 * ev.y)
        elif ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_SPACE:
                self.alternar()
            elif ev.key == pygame.K_q:
                self.girar(-1)
            elif ev.key == pygame.K_e:
                self.girar(1)
            elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.zoom(0.08)
            elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.zoom(-0.08)
            elif ev.key == pygame.K_r:
                self.reiniciar_manual()
            elif ev.key == pygame.K_c:
                self.comparar()

    #  actualización por cuadro 
    def posicion_robot(self):
        if self.fase in ("ruta", "fin") and self.ruta:
            i = min(self.idx_ruta, len(self.ruta) - 1)
            if i >= len(self.ruta) - 1:
                return self.ruta[-1]
            a, b = self.ruta[i], self.ruta[i + 1]
            t = self.progreso
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        return INICIO

    def actualizar(self, dt):
        self.tiempo += dt
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

    #  dibujo del mapa 3D 
    def _diamante(self, x, y):
        return [self.vista.proyectar(x, y), self.vista.proyectar(x + 1, y),
                self.vista.proyectar(x + 1, y + 1), self.vista.proyectar(x, y + 1)]

    def _glow(self, s, centro, radio, color, alfa):
        if radio < 1:
            return
        capa = pygame.Surface((radio * 2, radio * 2), pygame.SRCALPHA)
        pygame.draw.circle(capa, (*color, max(0, min(255, int(alfa)))), (radio, radio), radio)
        s.blit(capa, (centro[0] - radio, centro[1] - radio))

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
        ex, ey = self.vista.proyectar(INICIO[0] + 0.5, INICIO[1] + 0.5, 0.05)
        pygame.draw.polygon(s, CIAN, self._diamante(*INICIO), 2)
        self._texto(s, "E", self.fuente_chica, CIAN, (ex, ey), centro=True)

        # Objetos ordenados por profundidad: paredes, superviviente y robot
        objetos = []
        for y in range(FILAS):
            for x in range(COLUMNAS):
                if self.mapa[y][x] == ESCOMBRO:
                    objetos.append((self.vista.profundidad(x + 0.5, y + 0.5), "pared", (x, y)))
        objetos.append((self.vista.profundidad(OBJETIVO[0] + 0.5, OBJETIVO[1] + 0.5), "meta", OBJETIVO))
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

        # Indicaciones
        self._texto(s, "Edificio colapsado", self.fuente_titulo, TEXTO, (20, 16))
        self._texto(s, self.estado, self.fuente, VERDE, (20, 46))
        self._texto(s, "Clic: editar celda · Espacio: iniciar · Q/E: girar · +/−: zoom",
                    self.fuente_chica, SUAVE, (20, ALTO - 28))

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

    #  panel 
    def dibujar_panel(self, s):
        pygame.draw.rect(s, PANEL, (PANEL_X, 0, ANCHO - PANEL_X, ALTO))
        pygame.draw.line(s, BORDE, (PANEL_X, 0), (PANEL_X, ALTO), 2)
        self._texto(s, "Robot de rescate", self.fuente_titulo, TEXTO, (PANEL_X + 20, 14))
        self._texto(s, "Simulador 3D · BFS, Dijkstra y A*", self.fuente_chica, SUAVE, (PANEL_X + 20, 46))

        # Textos dentro de los botones de selección
        self.botones[0].texto = f"Algoritmo: {NOMBRES[ALGORITMOS[self.idx_alg]]}"
        self.botones[1].texto = f"Velocidad: {VELOCIDADES[self.idx_vel][0]}"
        for b in self.botones:
            b.dibujar(s, self.fuente)
        self._texto(s, DESCRIPCIONES[ALGORITMOS[self.idx_alg]], self.fuente_chica, SUAVE, (PANEL_X + 20, 348))

        # Indicadores
        r = self.ultimo
        filas = [
            ("Pasos de la ruta", str(r.pasos) if r and r.llego else "—"),
            ("Nodos explorados", str(len(self.orden)) if r else "—"),
            ("Celdas con gas", str(r.gas) if r else "—"),
            ("Costo total", str(r.costo) if r and r.llego else "—"),
            ("Tiempo de cálculo", f"{r.ms:.2f} ms" if r else "—"),
        ]
        y = 372
        for etiqueta, valor in filas:
            self._texto(s, etiqueta, self.fuente, SUAVE, (PANEL_X + 20, y))
            self._texto(s, valor, self.fuente, TEXTO, (ANCHO - 20, y), derecha=True)
            y += 22
        y += 6
        for linea in self._lineas(self.resultado, 300)[:3]:
            self._texto(s, linea, self.fuente_chica, SUAVE, (PANEL_X + 20, y))
            y += 18

        # Comparación
        y = 556
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
            self._texto(s, "★ menor costo total (pasos + 5 × gas)", self.fuente_chica, SUAVE, (PANEL_X + 20, y))

        # Bitácora
        y = 680
        pygame.draw.line(s, BORDE, (PANEL_X + 20, y - 8), (ANCHO - 20, y - 8), 1)
        self._texto(s, "Bitácora", self.fuente, TEXTO, (PANEL_X + 20, y))
        y += 24
        for linea in self.bitacora[-3:]:
            self._texto(s, "› " + linea, self.fuente_chica, SUAVE, (PANEL_X + 20, y))
            y += 17


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
