# Sistema de Búsqueda y Navegación para Robot de Rescate
## Documentación de la parte de Robles (5.1, interfaz gráfica y 6.1)

---

### 5.1 Código fuente documentado

#### Archivos del proyecto

| Archivo | Función |
|---|---|
| `index.html` | Estructura de la página, controles, paneles y tabla de comparación. |
| `simulador.js` | Mapa, algoritmos de búsqueda, animación del robot, comparación y edición del mapa. |
| `Documentacion_Robles.md` | Este documento. |

No requiere instalación, servidor ni conexión a internet. Basta abrir `index.html` en Chrome, Edge o Firefox.

#### Modelo del entorno

El edificio es una cuadrícula de **20 columnas × 13 filas**. Cada celda tiene uno de tres tipos:

* **Libre:** el robot puede pasar con costo 1.
* **Escombro:** bloquea el paso; ningún algoritmo lo atraviesa.
* **Gas:** se puede cruzar, pero suma **5 unidades** de costo en Dijkstra y A*. BFS lo ignora.

La entrada está en `(1, 6)` y el superviviente en `(18, 6)`. El robot se mueve en cuatro direcciones (derecha, abajo, izquierda, arriba). El mapa es conocido de antemano: el robot no descubre obstáculos durante el recorrido.

#### Funciones principales de `simulador.js`

| Función | Qué hace |
|---|---|
| `mapaOriginal()` | Construye la matriz del mapa a partir de los bloques de escombros y las fugas de gas. |
| `vecinos(posicion)` | Devuelve las celdas transitables vecinas en el orden derecha, abajo, izquierda, arriba. |
| `buscar(algoritmo)` | Ejecuta BFS, Dijkstra o A*. Devuelve la ruta, los nodos expandidos en orden, el costo y el tiempo de cálculo. |
| `nuevaBusqueda()` | Calcula la ruta con el algoritmo elegido y prepara la animación. |
| `avanzar()` | Avanza un cuadro: primero muestra la exploración y después mueve el robot por la ruta. |
| `dibujar()` | Pinta el mapa, las celdas exploradas, la ruta, la entrada, la meta y el robot. |
| `comparar()` | Ejecuta los tres algoritmos sobre el mismo mapa y muestra la tabla con la recomendación. |

#### Algoritmos

* **BFS:** busca la ruta con menos pasos. Aquí se implementa como búsqueda de costo uniforme con costo 1 por paso, por lo que ignora el gas.
* **Dijkstra:** busca la ruta de menor costo total. Cada paso cuesta 1 y cada celda con gas suma 5.
* **A*:** usa el mismo costo que Dijkstra y añade la distancia Manhattan al superviviente como estimación. Esa estimación nunca supera el costo real, por lo que la ruta encontrada sigue siendo la de menor costo.

#### Criterio de recomendación

En la comparación, la ruta recomendada es la de **menor costo total = pasos + 5 × celdas con gas**. En caso de empate, se recomienda el algoritmo que explora menos nodos.

#### Limitaciones

Es una demostración didáctica. No modela el movimiento físico del robot, la incertidumbre de los sensores, las comunicaciones, los movimientos diagonales ni los cambios del entorno durante el recorrido. El costo del gas es un supuesto para comparar los algoritmos.

---

### Interfaz gráfica

* **Mapa:** muestra el edificio, la entrada (E), el superviviente (S), el robot (R), los escombros, las fugas de gas, las celdas exploradas y la ruta final.
* **Algoritmo:** selector con descripción de cada método. Cambiarlo reinicia la simulación.
* **Velocidad:** pausada, normal o rápida. Afecta solo a la animación.
* **Iniciar / Pausar / Reanudar:** controla el recorrido del robot.
* **Reiniciar:** devuelve el robot a la entrada y limpia los resultados.
* **Comparar algoritmos:** tabla con pasos, nodos explorados, celdas con gas, costo y tiempo, con la ruta recomendada marcada.
* **Restablecer mapa original:** deshace las ediciones.
* **Editar el mapa:** cada clic sobre una celda la cambia en el orden libre → escombro → gas → libre. La entrada y la meta no se modifican.
* **Indicadores:** pasos, nodos explorados, celdas con gas, costo total y tiempo de cálculo.
* **Bitácora:** registra cada etapa de la búsqueda y del recorrido.

---

### 6.1 Capturas de pantalla de ejecución etiquetadas

Abre `index.html` en el navegador y toma capturas de la ventana completa. Guárdalas en esta carpeta con los siguientes nombres:

* `Figura_1_Interfaz.png`
* `Figura_2_Recorrido_A_estrella.png`
* `Figura_3_Comparacion.png`

**Figura 1. Interfaz inicial y mapa del edificio colapsado.** Deben verse la entrada (E), el superviviente (S), los escombros y las celdas con gas.

**Figura 2. Recorrido animado con A\*.** Debe verse la ruta final resaltada, el robot avanzando y los indicadores del recorrido.

**Figura 3. Comparación de BFS, Dijkstra y A\*.** Debe verse la tabla generada con el botón Comparar algoritmos, con la fila recomendada marcada.

> Las capturas deben mostrar la aplicación en ejecución. No se deben usar dibujos de ejemplo como si fueran capturas reales.
