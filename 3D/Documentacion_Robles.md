# Sistema de Búsqueda y Navegación para Robot de Rescate

## Documentación de la parte de Robles (5.1, interfaz gráfica y 6.1)



---



### 5.1 Código fuente documentado



#### Archivos del proyecto



| Archivo | Descripción |

|---|---|

| `simulador_3d.py` | Script principal. Implementa la simulación 3D isométrica usando pygame. |

| `simulador.py` | Versión clásica en 2D con tkinter. Se mantiene únicamente como respaldo que no depende de pygame. |

| `vista_previa_3d.png` | Captura generada durante las pruebas de la vista 3D. |

| `Documentacion_Robles.md` | Este archivo con la documentación técnica. |



#### Requisitos y ejecución



- Python 3.8 o superior.

- Librería pygame: `pip install pygame`



Para ejecutar el simulador desde la terminal:



```bash

python simulador_3d.py

```



#### Modelo del entorno



El mapa del edificio está representado por una cuadrícula de **20 columnas por 13 filas**. Cada celda pertenece a una de estas tres categorías:



- **Libre:** celda transitable con un costo base de 1.

- **Escombro:** obstáculo intransitable; se renderiza como un bloque tridimensional de altura variable.

- **Gas:** celda transitable que añade un costo extra de **5 unidades** para Dijkstra y A*, mientras que BFS la atraviesa ignorando esta penalización. Visualmente se representa con una animación de humo.



La posición inicial (entrada) está fijada en `(1, 6)` y la del superviviente en `(18, 6)`. El desplazamiento del robot se realiza en cuatro direcciones ortogonales. El entorno es estático y conocido de antemano, por lo que el robot no implementa descubrimiento dinámico de obstáculos en tiempo real.



#### Representación 3D



Para la vista isométrica, cada celda del suelo es un rombo y los escombros son prismas formados por caras laterales y superiores. El motor gráfico filtra las caras ocultas y ordena los elementos por profundidad (algoritmo tipo pintor) para que las paredes y objetos cercanos oculten correctamente al robot y al superviviente. El robot cuenta con movimiento interpolado entre celdas y una sombra proyectada en el suelo.



La cámara permite rotación en incrementos de 90 grados mediante las teclas `Q` y `E`, así como zoom in/out con las teclas `+`/`-` o usando la rueda del ratón.



#### Funciones principales de `simulador_3d.py`



| Función o clase | Descripción técnica |

|---|---|

| `mapa_original()` | Inicializa y devuelve la matriz del mapa con la distribución de escombros y fugas de gas. |

| `vecinos(mapa, pos)` | Obtiene las celdas vecinas transitables evaluadas en orden estricto: derecha, abajo, izquierda y arriba. |

| `buscar(mapa, algoritmo)` | Ejecuta la búsqueda utilizando BFS, Dijkstra o A* apoyándose en una cola de prioridad (`heapq`). Retorna un objeto `Resultado` con la ruta, el orden de expansión de nodos, el costo acumulado y el tiempo de ejecución. |

| `Vista` | Controlador de la cámara isométrica encargada de la rotación, el zoom y la proyección matemática de coordenadas 3D a pantalla 2D. |

| `App.actualizar(dt)` | Actualiza la lógica de animación, el avance del robot y los estados de exploración según el delta de tiempo (`dt`). |

| `App.dibujar_mapa()` | Renderiza el suelo, los efectos de humo, la ruta calculada, los elementos ordenados por profundidad, el superviviente y el robot. |

| `App.dibujar_panel()` | Dibuja la interfaz de control lateral con botones, indicadores de estado, la tabla comparativa y la bitácora de eventos. |

| `App.comparar()` | Ejecuta secuencialmente los tres algoritmos sobre el mapa actual y determina la ruta óptima recomendada. |

| `App.editar_celda()` | Captura los clics del ratón sobre la cuadrícula para alternar dinámicamente el tipo de celda. |



#### Algoritmos implementados



- **BFS:** busca la ruta minimizando el número de saltos o pasos. Se implementa como búsqueda uniforme con peso unitario por celda, ignorando el factor de gas.

- **Dijkstra:** calcula la ruta de costo real mínimo. Cada celda libre cuesta 1 y cada celda con gas suma una penalización de 5.

- **A*:** emplea la misma función de costo que Dijkstra pero incorpora la distancia Manhattan hasta el superviviente como heurística admisible y consistente, garantizando encontrar la ruta óptima con menor expansión de nodos.



#### Criterio de recomendación



El sistema selecciona como ruta recomendada aquella que obtenga el **menor costo total = pasos + 5 × celdas con gas**. Ante un empate en costo, se prioriza el algoritmo que haya explorado una menor cantidad de nodos.



#### Limitaciones conocidas



Se trata de un modelo didáctico enfocado en la comparación algorítmica. No contempla cinemática de vehículos reales, incertidumbre de sensores, interferencias de comunicación, movimientos diagonales ni la aparición de nuevos obstáculos sobre la marcha. La penalización por gas es un parámetro configurable para ilustrar el comportamiento de los algoritmos de costos.



---



### Interfaz gráfica



**Panel de control derecho:**

- **Algoritmo:** selector entre A*, Dijkstra y BFS. Al cambiarlo, la simulación se reinicia automáticamente.

- **Velocidad:** control de ticks para velocidad pausada, normal o rápida, afectando el paso del robot.

- **Iniciar / Pausar / Reanudar:** botones de control de ejecución del recorrido.

- **Reiniciar:** devuelve al robot al punto de inicio y limpia el historial de ruta.

- **Comparar:** genera y muestra una tabla con los datos de pasos, nodos explorados, celdas de gas y costo total de los tres algoritmos, destacando la mejor opción con una marca especial.

- **Restablecer:** revierte cualquier modificación manual hecha al mapa.

- **Controles de cámara:** rotación (`Q` / `E`) y zoom (`-` / `+`).

- **Indicadores en tiempo real:** métricas de pasos, nodos explorados, celdas con gas, costo y tiempo de cálculo.

- **Bitácora:** registro en texto de los eventos de búsqueda y navegación.



**Interacción con el mapa:**

- **Clic izquierdo en celda:** alterna el tipo de celda en ciclo: libre → escombro → gas → libre (las posiciones de la entrada y la meta están protegidas).

- **Atajos de teclado:** 

  - `Espacio`: Iniciar o pausar la simulación.

  - `R`: Reiniciar simulación.

  - `C`: Ejecutar comparación de algoritmos.



---



### 6.1 Capturas de pantalla de ejecución etiquetadas



Para cumplir con esta sección, ejecuta `python simulador_3d.py` y guarda las capturas de pantalla de la aplicación completa con los nombres indicados:



- `Figura_1_Interfaz.png`




**Figura 1. Interfaz inicial y mapa tridimensional del edificio colapsado.** Muestra la celda de entrada (E), la meta del superviviente (S), los bloques de escombros y las zonas afectadas por gas.



**Figura 2. Recorrido animado con A\*.** Muestra la ruta trazada sobre el suelo, el robot en movimiento y los contadores del panel lateral actualizados.



**Figura 3. Comparación de BFS, Dijkstra y A\*.** Muestra la tabla comparativa desplegada tras presionar el botón correspondiente, con los resultados y la ruta recomendada señalada.



