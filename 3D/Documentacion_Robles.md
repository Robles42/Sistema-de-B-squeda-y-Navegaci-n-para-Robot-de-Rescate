# Sistema de Búsqueda y Navegación para Robot de Rescate
## Documentación de la parte de Robles (5.1, interfaz gráfica y 6.1)

---

### 5.1 Código fuente documentado

#### Archivos del proyecto

| Archivo | Función |
|---|---|
| `simulador_3d.py` | Versión principal. Simulación 3D isométrica con pygame. |
| `simulador.py` | Versión 2D con tkinter, conservada como respaldo. No requiere pygame. |
| `vista_previa_3d.png` | Captura de la versión 3D en ejecución, generada durante las pruebas. |
| `Documentacion_Robles.md` | Este documento. |

#### Requisitos y ejecución

* Python 3.8 o superior.
* pygame: `pip install pygame`

Ejecución desde la carpeta del proyecto:

```
python simulador_3d.py
```

#### Modelo del entorno

El edificio es una cuadrícula de **20 columnas × 13 filas**. Cada celda es de uno de tres tipos:

* **Libre:** el robot puede pasar con costo 1.
* **Escombro:** bloquea el paso; se dibuja como un bloque de altura variable.
* **Gas:** se puede cruzar con costo extra de **5 unidades** en Dijkstra y A*. BFS lo ignora. Se dibuja como una zona con humo animado.

La entrada está en `(1, 6)` y el superviviente en `(18, 6)`. El robot se mueve en cuatro direcciones. El mapa es conocido de antemano: el robot no descubre obstáculos durante el recorrido.

#### Representación 3D

La escena usa una proyección isométrica: cada celda es un rombo en el suelo y los escombros son prismas con caras laterales y superior. Solo se dibujan las caras visibles desde la cámara. Los objetos se ordenan por profundidad para que las paredes cercanas tapen correctamente al robot y al superviviente. El robot se desplaza de forma continua entre celdas, con una sombra en el suelo.

La cámara puede girar en pasos de 90 grados (teclas Q y E) y acercarse o alejarse (teclas + y -, o la rueda del ratón).

#### Funciones principales de `simulador_3d.py`

| Función o clase | Qué hace |
|---|---|
| `mapa_original()` | Construye la matriz del mapa a partir de los bloques de escombros y las fugas de gas. |
| `vecinos(mapa, pos)` | Devuelve las celdas transitables vecinas en el orden derecha, abajo, izquierda, arriba. |
| `buscar(mapa, algoritmo)` | Ejecuta BFS, Dijkstra o A* con una cola de prioridad (`heapq`). Devuelve un `Resultado` con ruta, nodos expandidos en orden, costo y tiempo. |
| `Vista` | Cámara isométrica: rotación de 90 grados, zoom y proyección de coordenadas 3D a pantalla. |
| `App.actualizar(dt)` | Avanza la exploración y el movimiento del robot según el tiempo transcurrido. |
| `App.dibujar_mapa()` | Dibuja el suelo, el humo, la ruta, las paredes ordenadas por profundidad, el superviviente y el robot. |
| `App.dibujar_panel()` | Dibuja los botones, los indicadores, la tabla de comparación y la bitácora. |
| `App.comparar()` | Ejecuta los tres algoritmos sobre el mismo mapa y marca la ruta recomendada. |
| `App.editar_celda()` | Convierte un clic en el mapa en una celda y la cambia de tipo. |

#### Algoritmos

* **BFS:** busca la ruta con menos pasos. Aquí se implementa como búsqueda de costo uniforme con costo 1 por paso, por lo que ignora el gas.
* **Dijkstra:** busca la ruta de menor costo total. Cada paso cuesta 1 y cada celda con gas suma 5.
* **A*:** usa el mismo costo que Dijkstra y añade la distancia Manhattan al superviviente como estimación. Esa estimación nunca supera el costo real, por lo que la ruta sigue siendo la de menor costo.

#### Criterio de recomendación

En la comparación, la ruta recomendada es la de **menor costo total = pasos + 5 × celdas con gas**. En caso de empate, se recomienda el algoritmo que explora menos nodos.

#### Limitaciones

Es una demostración didáctica. No modela el movimiento físico del robot, la incertidumbre de los sensores, las comunicaciones, los movimientos diagonales ni los cambios del entorno durante el recorrido. El costo del gas es un supuesto para comparar los algoritmos.

---

### Interfaz gráfica

**Panel derecho:**
* **Algoritmo:** cambia entre A*, Dijkstra y BFS. Cambiarlo reinicia la simulación.
* **Velocidad:** pausada, normal o rápida. Afecta la velocidad de exploración y del robot.
* **Iniciar / Pausar / Reanudar:** controla el recorrido.
* **Reiniciar:** devuelve el robot a la entrada y limpia los resultados.
* **Comparar:** tabla con pasos, nodos explorados, celdas con gas y costo, con la ruta recomendada marcada con ★.
* **Restablecer:** deshace las ediciones del mapa.
* **Girar (Q / E) y Alejar / Acercar (- / +):** controlan la cámara.
* **Indicadores:** pasos, nodos explorados, celdas con gas, costo total y tiempo de cálculo.
* **Bitácora:** registra cada etapa de la búsqueda y del recorrido.

**Mapa:**
* **Clic en una celda:** cambia su tipo en el orden libre → escombro → gas → libre. La entrada y la meta no se modifican.
* **Espacio:** iniciar o pausar. **R:** reiniciar. **C:** comparar.

---

### 6.1 Capturas de pantalla de ejecución etiquetadas

Ejecuta `python simulador_3d.py` y toma capturas de la ventana completa. Guárdalas en esta carpeta con los siguientes nombres:

* `Figura_1_Interfaz.png`
* `Figura_2_Recorrido_A_estrella.png`
* `Figura_3_Comparacion.png`

**Figura 1. Interfaz inicial y mapa tridimensional del edificio colapsado.** Deben verse la entrada (E), el superviviente (S), los escombros y las zonas con gas.

**Figura 2. Recorrido animado con A\*.** Debe verse la ruta final resaltada, el robot avanzando y los indicadores del recorrido.

**Figura 3. Comparación de BFS, Dijkstra y A\*.** Debe verse la tabla generada con el botón Comparar, con la fila recomendada marcada.

> Las capturas deben mostrar la aplicación en ejecución. El archivo `vista_previa_3d.png` es solo una verificación interna y no sustituye las capturas de la entrega.
