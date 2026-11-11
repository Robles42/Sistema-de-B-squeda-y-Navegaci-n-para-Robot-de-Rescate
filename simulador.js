// =====================================================================
// Simulador de búsqueda para robot de rescate
// Compara BFS, Dijkstra y A* en una cuadrícula con escombros y gas.
// =====================================================================

// ---------- Configuración ----------
const COLUMNAS = 20;
const FILAS = 13;
const TAM = 40;                  // tamaño de cada celda en píxeles
const INICIO = [1, 6];           // entrada del edificio [columna, fila]
const OBJETIVO = [18, 6];        // posición del superviviente
const COSTO_GAS = 5;             // costo extra por entrar a una celda con gas
const CUADROS_EXPLORACION = 60;  // cuántos cuadros dura la animación de exploración

const NOMBRES = { bfs: "BFS", dijkstra: "Dijkstra", astar: "A*" };
const DESCRIPCIONES = {
  bfs: "Busca la ruta con menos pasos. No considera el gas.",
  dijkstra: "Busca la ruta de menor costo: cada paso cuesta 1 y cada celda con gas suma 5.",
  astar: "Igual que Dijkstra, pero guiado por una estimación de distancia al superviviente."
};

// Tipos de celda
const LIBRE = 0, ESCOMBRO = 1, GAS = 2;
const NOMBRE_TIPO = ["libre", "escombro", "gas"];

// Bloques de escombros: [columna inicial, fila inicial, columna final, fila final]
const BLOQUES = [
  [4, 0, 4, 4], [4, 7, 4, 12], [8, 2, 8, 9], [12, 0, 12, 5],
  [12, 8, 12, 12], [16, 3, 16, 12], [2, 2, 2, 3], [6, 10, 7, 10],
  [10, 11, 11, 11], [14, 1, 15, 1]
];

// Fugas de gas: [columna, fila]
const FUGAS = [
  [3, 6], [5, 5], [5, 6], [7, 3], [7, 4], [9, 9], [10, 9],
  [11, 7], [13, 6], [14, 7], [15, 5], [17, 6], [17, 7]
];

// ---------- Utilidades ----------
const $ = (id) => document.getElementById(id);
const clave = ([x, y]) => `${x},${y}`;
const enRango = (x, y) => x >= 0 && x < COLUMNAS && y >= 0 && y < FILAS;

function mapaOriginal() {
  const m = Array.from({ length: FILAS }, () => Array(COLUMNAS).fill(LIBRE));
  BLOQUES.forEach(([x1, y1, x2, y2]) => {
    for (let y = y1; y <= y2; y++) {
      for (let x = x1; x <= x2; x++) m[y][x] = ESCOMBRO;
    }
  });
  FUGAS.forEach(([x, y]) => {
    if (m[y][x] === LIBRE) m[y][x] = GAS;
  });
  return m;
}

// Estado del mapa: mapa[fila][columna] con LIBRE, ESCOMBRO o GAS
let mapa = mapaOriginal();

// Vecinos transitables en el orden derecha, abajo, izquierda, arriba
function vecinos([x, y]) {
  return [[x + 1, y], [x, y + 1], [x - 1, y], [x, y - 1]]
    .filter(([a, b]) => enRango(a, b) && mapa[b][a] !== ESCOMBRO);
}

// ---------- Algoritmos ----------
// Búsqueda de costo: BFS usa costo 1 por paso; Dijkstra y A* suman gas.
// A* además suma la distancia Manhattan estimada al objetivo.
function buscar(algoritmo) {
  const inicioTiempo = performance.now();
  const usaGas = algoritmo !== "bfs";
  const estimar = ([x, y]) => Math.abs(x - OBJETIVO[0]) + Math.abs(y - OBJETIVO[1]);

  const costo = new Map([[clave(INICIO), 0]]);
  const previo = new Map();
  const cerrados = new Set();
  const orden = [];                        // nodos expandidos, en orden
  const frontera = [{ pos: INICIO, g: 0, f: algoritmo === "astar" ? estimar(INICIO) : 0 }];
  let llego = false;

  while (frontera.length > 0) {
    // Se toma el nodo con menor prioridad f
    let mejor = 0;
    for (let i = 1; i < frontera.length; i++) {
      if (frontera[i].f < frontera[mejor].f) mejor = i;
    }
    const { pos, g } = frontera.splice(mejor, 1)[0];
    const k = clave(pos);
    if (cerrados.has(k)) continue;

    cerrados.add(k);
    orden.push(pos);
    if (k === clave(OBJETIVO)) {
      llego = true;
      break;
    }

    vecinos(pos).forEach((vecino) => {
      const kv = clave(vecino);
      const entraGas = usaGas && mapa[vecino[1]][vecino[0]] === GAS;
      const nuevo = g + 1 + (entraGas ? COSTO_GAS : 0);
      if (!costo.has(kv) || nuevo < costo.get(kv)) {
        costo.set(kv, nuevo);
        previo.set(kv, k);
        const f = nuevo + (algoritmo === "astar" ? estimar(vecino) : 0);
        frontera.push({ pos: vecino, g: nuevo, f });
      }
    });
  }

  // Reconstruye la ruta siguiendo los nodos anteriores desde la meta
  const ruta = [];
  if (llego) {
    let k = clave(OBJETIVO);
    while (k) {
      ruta.unshift(k.split(",").map(Number));
      k = previo.get(k);
    }
  }

  return {
    algoritmo,
    ruta,
    orden,
    llego,
    pasos: ruta.length ? ruta.length - 1 : 0,
    gas: ruta.filter(([x, y]) => mapa[y][x] === GAS).length,
    costo: llego ? costo.get(clave(OBJETIVO)) : null,
    ms: performance.now() - inicioTiempo
  };
}

// ---------- Estado de la animación ----------
const lienzo = $("mapa");
const ctx = lienzo.getContext("2d");
const DPR = window.devicePixelRatio || 1;
lienzo.width = COLUMNAS * TAM * DPR;
lienzo.height = FILAS * TAM * DPR;
ctx.setTransform(DPR, 0, 0, DPR, 0, 0);

let fase = "listo";        // listo | explorando | ruta | fin
let temporizador = null;
let ultimo = null;         // último resultado de buscar()
let orden = [];
let ruta = [];
let ordenVisitas = new Map();
let indiceExploracion = 0;
let indiceRuta = 0;
let paso = 1;
let robot = [...INICIO];

// ---------- Dibujo ----------
function texto(t, x, y, color, tamano = 14) {
  ctx.fillStyle = color;
  ctx.font = `600 ${tamano}px system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(t, x, y);
}

function centro([x, y]) {
  return [x * TAM + TAM / 2, y * TAM + TAM / 2];
}

function dibujar() {
  ctx.clearRect(0, 0, COLUMNAS * TAM, FILAS * TAM);

  for (let y = 0; y < FILAS; y++) {
    for (let x = 0; x < COLUMNAS; x++) {
      const px = x * TAM;
      const py = y * TAM;
      const tipo = mapa[y][x];

      // Fondo según el tipo de celda
      ctx.fillStyle = tipo === ESCOMBRO ? "#2c3b4c" : tipo === GAS ? "#3a2a22" : "#112236";
      ctx.fillRect(px + 1, py + 1, TAM - 2, TAM - 2);

      // Celdas exploradas hasta el momento: más oscuras las revisadas al final
      const idx = ordenVisitas.get(clave([x, y]));
      if (tipo === LIBRE && idx !== undefined && idx < indiceExploracion) {
        const f = idx / Math.max(1, orden.length);
        ctx.fillStyle = `rgba(67,217,206,${(0.4 - 0.25 * f).toFixed(2)})`;
        ctx.fillRect(px + 1, py + 1, TAM - 2, TAM - 2);
      }

      const [cx, cy] = centro([x, y]);
      if (tipo === ESCOMBRO) {
        texto("✕", cx, cy, "#8b9bad", 15);
      } else if (tipo === GAS) {
        const brillo = ctx.createRadialGradient(cx, cy, 2, cx, cy, TAM / 2);
        brillo.addColorStop(0, "rgba(255,180,84,.6)");
        brillo.addColorStop(1, "rgba(255,180,84,0)");
        ctx.fillStyle = brillo;
        ctx.fillRect(px, py, TAM, TAM);
        texto("GAS", cx, cy, "#ffc98a", 10);
      }
    }
  }

  // Ruta final: se dibuja conforme el robot avanza
  if ((fase === "ruta" || fase === "fin") && ruta.length > 0) {
    ctx.strokeStyle = "#ff6f80";
    ctx.lineWidth = 4;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    ruta.slice(0, indiceRuta + 1).forEach((celda, i) => {
      const [cx, cy] = centro(celda);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  // Entrada y superviviente
  const [ex, ey] = centro(INICIO);
  ctx.strokeStyle = "#43d9ce";
  ctx.lineWidth = 2;
  ctx.strokeRect(INICIO[0] * TAM + 8, INICIO[1] * TAM + 8, TAM - 16, TAM - 16);
  texto("E", ex, ey, "#43d9ce", 12);

  ctx.fillStyle = "#83e3a0";
  ctx.fillRect(OBJETIVO[0] * TAM + 7, OBJETIVO[1] * TAM + 7, TAM - 14, TAM - 14);
  const [sx, sy] = centro(OBJETIVO);
  texto("S", sx, sy, "#082415", 14);

  // Robot
  const [rx, ry] = centro(robot);
  ctx.save();
  ctx.shadowColor = "rgba(67,217,206,.7)";
  ctx.shadowBlur = 14;
  ctx.fillStyle = "#43d9ce";
  ctx.beginPath();
  ctx.arc(rx, ry, 13, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
  texto("R", rx, ry, "#06201e", 13);
}

// ---------- Interfaz ----------
function registrar(mensaje) {
  const bitacora = $("bitacora");
  const linea = document.createElement("div");
  linea.textContent = mensaje;
  bitacora.appendChild(linea);
  bitacora.scrollTop = bitacora.scrollHeight;
}

function setEstado(texto) {
  $("estado").textContent = texto;
}

function mostrarIndicadores(r) {
  $("pasos").textContent = r.llego ? r.pasos : "—";
  $("explorados").textContent = r.orden.length;
  $("gas").textContent = r.gas;
  $("costo").textContent = r.llego ? r.costo : "—";
  $("tiempo").textContent = `${r.ms.toFixed(2)} ms`;
  $("barra-gas").style.width = `${Math.min(100, r.gas * 20)}%`;
  $("barra-gas").style.background = r.gas ? "#ffb454" : "#83e3a0";
  $("resultado").textContent = r.llego
    ? `${NOMBRES[r.algoritmo]} encontró una ruta de ${r.pasos} pasos con ${r.gas} celda(s) de gas.`
    : `${NOMBRES[r.algoritmo]} no encontró ruta: los escombros bloquean el paso al superviviente.`;
}

// Limpia la búsqueda y deja el robot en la entrada
function reiniciar() {
  clearInterval(temporizador);
  temporizador = null;
  fase = "listo";
  ultimo = null;
  orden = [];
  ruta = [];
  ordenVisitas = new Map();
  indiceExploracion = 0;
  indiceRuta = 0;
  robot = [...INICIO];
  ["pasos", "explorados", "gas", "costo", "tiempo"].forEach((id) => { $(id).textContent = "—"; });
  $("barra-gas").style.width = "0%";
  $("resultado").textContent = "Elige un algoritmo y pulsa Iniciar.";
  $("comparacion").innerHTML = "";
  $("boton-iniciar").textContent = "▶ Iniciar";
  setEstado("Listo");
  dibujar();
}

// Calcula una búsqueda nueva con el algoritmo seleccionado
function nuevaBusqueda() {
  const algoritmo = $("algo").value;
  ultimo = buscar(algoritmo);
  orden = ultimo.orden;
  ruta = ultimo.ruta;
  ordenVisitas = new Map(orden.map((pos, i) => [clave(pos), i]));
  indiceExploracion = 0;
  indiceRuta = 0;
  robot = [...INICIO];
  paso = Math.max(1, Math.ceil(orden.length / CUADROS_EXPLORACION));
  mostrarIndicadores(ultimo);

  if (!ultimo.llego) {
    fase = "fin";
    setEstado("Sin ruta");
    registrar(`${NOMBRES[algoritmo]}: no hay ruta hacia el superviviente.`);
    dibujar();
    return;
  }

  fase = "explorando";
  registrar(`${NOMBRES[algoritmo]} inicia la búsqueda.`);
  dibujar();
}

function alternar() {
  if (temporizador) return pausar();
  if (fase === "listo" || fase === "fin") {
    nuevaBusqueda();
    if (fase === "fin") return;
  }
  temporizador = setInterval(avanzar, Number($("velocidad").value));
  $("boton-iniciar").textContent = "❚❚ Pausar";
  setEstado(fase === "explorando" ? "Explorando" : "Siguiendo la ruta");
}

function pausar() {
  clearInterval(temporizador);
  temporizador = null;
  $("boton-iniciar").textContent = "▶ Reanudar";
  setEstado("En pausa");
}

function terminar() {
  clearInterval(temporizador);
  temporizador = null;
  fase = "fin";
  $("boton-iniciar").textContent = "▶ Iniciar";
  setEstado("Superviviente localizado");
  registrar(`El robot llegó al superviviente en ${ultimo.pasos} pasos.`);
}

// Un cuadro de la animación: primero la exploración, después el recorrido
function avanzar() {
  if (fase === "explorando") {
    indiceExploracion = Math.min(orden.length, indiceExploracion + paso);
    if (indiceExploracion >= orden.length) {
      fase = "ruta";
      registrar(`Exploración completa: ${orden.length} nodos revisados.`);
      setEstado("Siguiendo la ruta");
    }
  } else if (fase === "ruta") {
    if (indiceRuta < ruta.length - 1) {
      indiceRuta++;
      robot = ruta[indiceRuta];
    } else {
      terminar();
    }
  }
  dibujar();
}

// Compara los tres algoritmos sobre el mismo mapa
function comparar() {
  reiniciar();
  const resultados = Object.keys(NOMBRES).map(buscar);
  const costoTotal = (r) => (r.llego ? r.pasos + COSTO_GAS * r.gas : Infinity);
  const recomendado = resultados.reduce((a, b) => {
    if (costoTotal(b) !== costoTotal(a)) return costoTotal(b) < costoTotal(a) ? b : a;
    return b.orden.length < a.orden.length ? b : a;
  });
  const hayRuta = recomendado.llego;

  const filas = resultados.map((r) => {
    const esRecomendado = hayRuta && r === recomendado;
    return `<tr class="${esRecomendado ? "recomendado" : ""}">
      <td>${NOMBRES[r.algoritmo]}${esRecomendado ? " ★" : ""}</td>
      <td>${r.llego ? r.pasos : "—"}</td>
      <td>${r.orden.length}</td>
      <td>${r.gas}</td>
      <td>${r.llego ? costoTotal(r) : "—"}</td>
      <td>${r.ms.toFixed(2)} ms</td>
    </tr>`;
  }).join("");

  $("comparacion").innerHTML = `
    <table>
      <thead><tr><th>Algoritmo</th><th>Pasos</th><th>Explorados</th><th>Gas</th><th>Costo</th><th>Tiempo</th></tr></thead>
      <tbody>${filas}</tbody>
    </table>
    <p class="nota">★ Recomendado por menor costo total (pasos + ${COSTO_GAS} × celdas con gas). En empate, gana el que explora menos nodos.</p>`;

  registrar(hayRuta
    ? `Comparación lista. Recomendado: ${NOMBRES[recomendado.algoritmo]}.`
    : "Comparación lista. Ningún algoritmo encontró ruta.");
}

// ---------- Eventos ----------
$("boton-iniciar").addEventListener("click", alternar);

$("reiniciar").addEventListener("click", () => {
  reiniciar();
  registrar("Simulación reiniciada.");
});

$("comparar").addEventListener("click", comparar);

$("restablecer").addEventListener("click", () => {
  mapa = mapaOriginal();
  reiniciar();
  registrar("Mapa restablecido al original.");
});

$("algo").addEventListener("change", () => {
  $("descripcion").textContent = DESCRIPCIONES[$("algo").value];
  reiniciar();
});

$("velocidad").addEventListener("change", () => {
  if (temporizador) {
    clearInterval(temporizador);
    temporizador = setInterval(avanzar, Number($("velocidad").value));
  }
});

// Editar el mapa: libre → escombro → gas → libre
lienzo.addEventListener("click", (evento) => {
  const r = lienzo.getBoundingClientRect();
  const x = Math.floor(((evento.clientX - r.left) / r.width) * COLUMNAS);
  const y = Math.floor(((evento.clientY - r.top) / r.height) * FILAS);
  if (!enRango(x, y)) return;
  if (clave([x, y]) === clave(INICIO) || clave([x, y]) === clave(OBJETIVO)) return;

  reiniciar();
  mapa[y][x] = (mapa[y][x] + 1) % 3;
  registrar(`Celda (${x}, ${y}) cambiada a ${NOMBRE_TIPO[mapa[y][x]]}.`);
  dibujar();
});

// ---------- Inicio ----------
$("descripcion").textContent = DESCRIPCIONES[$("algo").value];
registrar("Sistema listo. Mapa de 20 × 13 celdas cargado.");
registrar("Entrada en E, superviviente en S. Los escombros bloquean; el gas añade costo.");
dibujar();
