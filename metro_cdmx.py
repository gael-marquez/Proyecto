## Metro - CDMX
import time
from collections import defaultdict
import matplotlib.pyplot as plt
import networkx as nx

# Estructura del grafo (diccionarios simples)
# estaciones = {nombre: {"lineas": [1,2], "coords": (x,y), "es_transbordo": bool}}
# conexiones = {origen: [(destino, tiempo_min, linea), ...]}
# costos = {"estacion_normal": 2, "transbordo": 3}

def crear_grafo():
    return {
        "estaciones": {},
        "conexiones": defaultdict(list),
        "costos": {
            "estacion_normal": 2,  # minutos
            "transbordo": 3        # minutos adicionales
        }
    }

def agregar_estacion(grafo, nombre, lineas, coords=(0, 0)):
    grafo["estaciones"][nombre] = {
        "lineas": lineas,
        "coords": coords,
        "es_transbordo": len(lineas) > 1
    }

def agregar_conexion(grafo, origen, destino, tiempo_min, linea):
    grafo["conexiones"][origen].append((destino, tiempo_min, linea))
    grafo["conexiones"][destino].append((origen, tiempo_min, linea))

def obtener_vecinos(grafo, estacion):
    return grafo["conexiones"].get(estacion, [])

def calcular_costo_movimiento(grafo, origen, destino, linea_actual):
    estacion_destino = grafo["estaciones"][destino]
    
    # costo modificado del grafo
    costo = grafo["costos"]["estacion_normal"]
    
    # penalizacion por transbordo
    if estacion_destino["es_transbordo"] and linea_actual not in estacion_destino["lineas"]:
        costo = costo + grafo["costos"]["transbordo"]
    
    return costo

def heuristica_euclidiana(grafo, origen, destino):
    coord_o = grafo["estaciones"][origen]["coords"]
    coord_d = grafo["estaciones"][destino]["coords"]
    return ((coord_o[0] - coord_d[0])**2 + (coord_o[1] - coord_d[1])**2)**0.5


# A*

def a_star(grafo, inicio, destino, linea_inicial=None):
    # Validar que las estaciones existan
    if inicio not in grafo["estaciones"] or destino not in grafo["estaciones"]:
        return None, {"error": "Estacion no encontrada"}
    
    # Determinar linea inicial
    if linea_inicial is None:
        linea_inicial = grafo["estaciones"][inicio]["lineas"][0]
    
    start_time = time.time()
    
    # LISTA ABIERTA [f_cost, g_cost, estacion, linea, camino]
    open_list = []
    h_inicial = heuristica_euclidiana(grafo, inicio, destino)
    nodo_inicial = [h_inicial, 0.0, inicio, linea_inicial, [inicio]]
    open_list = open_list + [nodo_inicial]
    
    # guardar el mejor g_cost para cada estado
    g_costs = {}
    g_costs[(inicio, linea_inicial)] = 0.0
    
    # LISTA DE VISITADOS
    visitados = []
    nodos_explorados = 0
    
    # BUCLE PRINCIPAL
    while len(open_list) > 0:
        # nodo con menor f_cost
        indice_mejor = 0
        for i in range(len(open_list)):
            if open_list[i][0] < open_list[indice_mejor][0]:
                indice_mejor = i
        
        # mejor nodo de la lista
        nodo_actual = open_list[indice_mejor]
        open_list = open_list[:indice_mejor] + open_list[indice_mejor+1:]
        
        # informacion del nodo
        f_cost = nodo_actual[0]
        g_cost = nodo_actual[1]
        estacion_actual = nodo_actual[2]
        linea_actual = nodo_actual[3]
        camino = nodo_actual[4]
        
        # Crear tupla del estado
        estado = (estacion_actual, linea_actual)
        
        # Si ya visitamos este estado, saltar
        if estado in visitados:
            continue
        
        # Marcar como visitado
        visitados = visitados + [estado]
        nodos_explorados = nodos_explorados + 1
        
        # ¿LLEGAMOS AL DESTINO?
        if estacion_actual == destino:
            tiempo_total = time.time() - start_time
            estadisticas = {
                "ruta": camino,
                "costo_total": g_cost,
                "nodos_explorados": nodos_explorados,
                "tiempo_segundos": tiempo_total,
                "longitud_ruta": len(camino),
                "eficiencia": len(camino) / nodos_explorados if nodos_explorados > 0 else 0
            }
            return camino, estadisticas
        
        # EXPLORAR VECINOS
        vecinos = obtener_vecinos(grafo, estacion_actual)
        for i in range(len(vecinos)):
            vecino = vecinos[i][0]
            tiempo_minutos = vecinos[i][1]
            linea_conexion = vecinos[i][2]
            
            # Calcular costo del movimiento
            costo_movimiento = calcular_costo_movimiento(
                grafo, estacion_actual, vecino, linea_actual
            )
            nuevo_g = g_cost + costo_movimiento
            
            # Crear tupla del vecino
            estado_vecino = (vecino, linea_conexion)
            
            # ¿Es un camino mejor?
            es_mejor = False
            if estado_vecino not in g_costs:
                es_mejor = True
            elif nuevo_g < g_costs[estado_vecino]:
                es_mejor = True
            
            if es_mejor:
                # Actualizar costo
                g_costs[estado_vecino] = nuevo_g
                
                # Calcular heuristica y f_cost
                h_cost = heuristica_euclidiana(grafo, vecino, destino)
                f_cost = nuevo_g + h_cost
                
                # Crear nuevo camino
                nuevo_camino = camino + [vecino]
                
                # Agregar a la lista abierta
                nuevo_nodo = [f_cost, nuevo_g, vecino, linea_conexion, nuevo_camino]
                open_list = open_list + [nuevo_nodo]
    
    # No se encontro ruta
    tiempo_total = time.time() - start_time
    return None, {
        "error": "No se encontro ruta",
        "nodos_explorados": nodos_explorados,
        "tiempo_segundos": tiempo_total
    }


# LOGICA DE PRIMER ORDEN

# BASE DE CONOCIMIENTO

hora_pico_hechos = [
    ("es_hora_pico", 7, 9),    # Mañana
    ("es_hora_pico", 18, 20),  # Tarde
]

hora_tranquila_hechos = [
    ("es_hora_tranquila", 10, 17),  # Media dia
    ("es_hora_tranquila", 21, 6),   # Noche/madrugada
]

costo_hechos = [
    ("hora_pico", "estacion_normal", 1.5),     # x1.5 tiempo en hora pico
    ("hora_pico", "transbordo", 2),            # x2 tiempo en transbordos
    ("hora_tranquila", "estacion_normal", 1),  # Tiempo normal
    ("hora_tranquila", "transbordo", 1),       # Tiempo normal
    ("prisa", "transbordo", 3),                # Triple penalizacion si tienes prisa
    ("prisa", "estacion_normal", 1),           # Normal para estaciones
    ("accesibilidad", "estacion_normal", 1.5), # x1.5 tiempo por accesibilidad
    ("accesibilidad", "transbordo", 2),        # x2 tiempo en transbordos
]

# Hechos sobre preferencias
preferencia_hechos = [
    ("prisa", "minimizar_transbordos", True),
    ("accesibilidad", "minimizar_distancia", True),
    ("hora_pico", "evitar_aglomeraciones", True),
]

# Inferir conflictos: condiciones que no pueden coexistir
conflictos_pairs = [
    ("hora_pico", "hora_tranquila"),
]

# Inferir combinaciones: cuando dos condiciones juntas amplifican el efecto
combinaciones_pairs = [
    ("prisa", "hora_pico", "urgencia_extrema", 1.5),  # condicion1, condicion2, nombre_resultado, multiplicador_extra
    ("accesibilidad", "hora_tranquila", "viaje_comodo", 1),  # Sin modificador adicional
]

# inferencia

def es_hora_pico(hora):
    #hora pico
    for condicion, inicio, fin in hora_pico_hechos:
        if inicio <= hora <= fin:
            return True
    return False

def es_hora_tranquila(hora):
   #hora tranquila
    for condicion, inicio, fin in hora_tranquila_hechos:
        if inicio <= fin:
            if inicio <= hora <= fin:
                return True
        else:  # Caso nocturno (21 a 6)
            if hora >= inicio or hora <= fin:
                return True
    return False

def obtener_modificador_costo(condicion, tipo_costo):
    #buscar costo para modificaciones
    for cond, tipo, mult in costo_hechos:
        if cond == condicion and tipo == tipo_costo:
            return mult
    return 1.0  # Neutro si no se encuentra

def tiene_preferencia(condicion, accion):
    #condicion implica preferencia
    for cond, acc, valor in preferencia_hechos:
        if cond == condicion and acc == accion:
            return valor
    return False

def detectar_combinacion(condiciones_activas):
    #combinaciones especiales
    combinaciones_detectadas = []
    for cond1, cond2, nombre, mult in combinaciones_pairs:
        if cond1 in condiciones_activas and cond2 in condiciones_activas:
            combinaciones_detectadas.append((nombre, mult))
    return combinaciones_detectadas

#explicaciones
def explicar_condicion(condicion, hora=None):
    if condicion == "hora_pico" and hora is not None:
        return f"hora_pico (hora {hora})"
    elif condicion == "hora_tranquila" and hora is not None:
        return f"hora_tranquila (hora {hora})"
    elif condicion == "prisa":
        return "prisa"
    elif condicion == "accesibilidad":
        return "accesibilidad"
    return condicion

def explicar_modificador(condicion, tipo_costo, multiplicador):
    return f"{condicion} -> {tipo_costo} x{multiplicador}"

def explicar_preferencia(condicion, accion):
    return f"{condicion} -> {accion}"

# MOTOR DE INFERENCIA

#reglas de inferencia
def inferir_contexto(hora, prisa, accesibilidad):
    condiciones_activas = []
    explicaciones = []
    modificadores = {}
    preferencias = []
    
    # INFERENCIA: Determinar condiciones temporales
    if es_hora_pico(hora):
        condiciones_activas.append("hora_pico")
        explicaciones.append(explicar_condicion("hora_pico", hora))
    elif es_hora_tranquila(hora):
        condiciones_activas.append("hora_tranquila")
        explicaciones.append(explicar_condicion("hora_tranquila", hora))
    
    # INFERENCIA: Condiciones de usuario
    if prisa:
        condiciones_activas.append("prisa")
        explicaciones.append(explicar_condicion("prisa"))
    
    if accesibilidad:
        condiciones_activas.append("accesibilidad")
        explicaciones.append(explicar_condicion("accesibilidad"))
    
    # INFERENCIA: Detectar combinaciones especiales
    combinaciones = detectar_combinacion(condiciones_activas)
    for nombre_comb, mult_extra in combinaciones:
        explicaciones.append(f"¡Combinacion especial detectada: {nombre_comb}! (multiplicador adicional: {mult_extra})")
        modificadores[nombre_comb] = mult_extra
    
    # DERIVACIoN: Aplicar modificadores de costo
    for condicion in condiciones_activas:
        # Buscar todos los modificadores para esta condicion
        for cond, tipo, mult in costo_hechos:
            if cond == condicion:
                if tipo not in modificadores:
                    modificadores[tipo] = 1.0
                modificadores[tipo] *= mult
                explicaciones.append(explicar_modificador(condicion, tipo, mult))
        
        # Buscar preferencias
        for cond, accion, valor in preferencia_hechos:
            if cond == condicion and valor:
                preferencias.append(accion)
                explicaciones.append(explicar_preferencia(condicion, accion))
    
    return {
        "condiciones_activas": condiciones_activas,
        "modificadores": modificadores,
        "preferencias": preferencias,
        "explicaciones": explicaciones,
        "combinaciones": combinaciones
    }

def aplicar_logica_primer_orden(grafo, hora, prisa, accesibilidad):
    print("LOGICA DE PRIMER ORDEN")
    
    print(f"Entrada: hora={hora}, prisa={prisa}, accesibilidad={accesibilidad}")
    
    contexto = inferir_contexto(hora, prisa, accesibilidad)
    
    print(f"Condiciones: {contexto['condiciones_activas']}")

    print("Modificadores aplicados:")
    for exp in contexto['explicaciones']:
        print(f"  - {exp}")
    
    print("Costos finales:")
    for tipo, mult in contexto['modificadores'].items():
        print(f"  {tipo}: x{mult:.1f}")
    
    # modificar el grafo
    if "estacion_normal" in contexto['modificadores']:
        grafo["costos"]["estacion_normal"] *= contexto['modificadores']['estacion_normal']
        print(f"\nTiempo estacion: {grafo['costos']['estacion_normal']:.1f} min")
    
    if "transbordo" in contexto['modificadores']:
        grafo["costos"]["transbordo"] *= contexto['modificadores']['transbordo']
        print(f"Tiempo transbordo: {grafo['costos']['transbordo']:.1f} min")
    
    return contexto


# Visualizacion

def visualizar_grafo_metro(grafo, ruta=None):    
    G = nx.Graph()
    pos = {}
    
    # Agregar nodos con posiciones
    for nombre, estacion in grafo["estaciones"].items():
        G.add_node(nombre)
        pos[nombre] = estacion["coords"]
    
    # Agregar aristas
    for origen, conexiones in grafo["conexiones"].items():
        for destino, tiempo_min, linea in conexiones:
            G.add_edge(origen, destino, linea=linea, tiempo=tiempo_min)
    
    plt.figure(figsize=(16, 12))    
    nx.draw_networkx_edges(G, pos, alpha=0.2, width=1, edge_color='gray')
    
    # Dibujar la ruta en verde si existe
    if ruta and len(ruta) > 1:
        ruta_edges = [(ruta[i], ruta[i+1]) for i in range(len(ruta)-1)]
        nx.draw_networkx_edges(G, pos, edgelist=ruta_edges, 
                              edge_color='green', width=4, alpha=0.8)
    
    # colores lineas
    colores_linea = {
        1: '#F54EA2',  # Rosa (Linea 1)
        2: '#0066CC',  # Azul (Linea 2)
        3: '#B5BD00',  # Verde (Linea 3)
        4: '#63C5C7',  # Cian (Linea 4)
        5: '#FFD200',  # Amarillo (Linea 5)
    }
    
    # Separar nodos por tipo
    nodos_transbordo = [n for n, e in grafo["estaciones"].items() if e["es_transbordo"]]
    nodos_por_linea = {1: [], 2: [], 3: [], 4: [], 5: []}
    
    # Clasificar nodos normales por linea
    for nombre, estacion in grafo["estaciones"].items():
        if not estacion["es_transbordo"]:
            # Tomar la primera linea de la estacion
            linea_principal = estacion["lineas"][0]
            nodos_por_linea[linea_principal].append(nombre)
    
    # Dibujar nodos normales coloreados por linea
    for linea, nodos in nodos_por_linea.items():
        if nodos:
            nx.draw_networkx_nodes(G, pos, nodelist=nodos,
                                  node_color=colores_linea[linea], 
                                  node_size=300, alpha=0.8,
                                  edgecolors='black', linewidths=0.5)
    
    # Nodos de transbordo en ROJO
    nx.draw_networkx_nodes(G, pos, nodelist=nodos_transbordo,
                          node_color='red', node_size=400, alpha=0.9,
                          edgecolors='darkred', linewidths=1.5)
    
    # Resaltar origen y destino si hay ruta
    if ruta:
        nx.draw_networkx_nodes(G, pos, nodelist=[ruta[0]],
                              node_color='lime', node_size=500, alpha=1,
                              edgecolors='darkgreen', linewidths=2)
        nx.draw_networkx_nodes(G, pos, nodelist=[ruta[-1]],
                              node_color='darkred', node_size=500, alpha=1,
                              edgecolors='black', linewidths=2)
    
    labels = {nombre: nombre for nombre in grafo["estaciones"].keys()}
    nx.draw_networkx_labels(G, pos, labels, font_size=6, font_weight='bold')
    
    # Titulo
    if ruta:
        plt.title(f"Metro CDMX - Ruta: {ruta[0]} → {ruta[-1]} ({len(ruta)} estaciones)",
                 fontsize=14, fontweight='bold')
    else:
        plt.title("Red del Metro CDMX (5 Lineas) - Nodos coloreados por linea, Transbordos en ROJO", 
                 fontsize=14, fontweight='bold')
    
    # Leyenda
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=colores_linea[1], edgecolor='black', label='Linea 1 (Rosa)'),
        Patch(facecolor=colores_linea[2], edgecolor='black', label='Linea 2 (Azul)'),
        Patch(facecolor=colores_linea[3], edgecolor='black', label='Linea 3 (Verde)'),
        Patch(facecolor=colores_linea[4], edgecolor='black', label='Linea 4 (Cian)'),
        Patch(facecolor=colores_linea[5], edgecolor='black', label='Linea 5 (Amarillo)'),
        Patch(facecolor='red', edgecolor='darkred', label='Transbordo'),
    ]
    plt.legend(handles=legend_elements, loc='upper left', fontsize=10)
    
    plt.axis('off')
    plt.tight_layout()
    plt.show()


# Creacion grafo

def crear_metro_cdmx_completo():
    metro = crear_grafo()
    
    # Linea 1 (ROSA) - Observatorio a Pantitlan 
    estaciones_l1 = [
        ("Observatorio", [1], (0, 50)),
        ("Tacubaya", [1], (3, 51)),
        ("Juanacatlan", [1], (6, 53)),
        ("Chapultepec", [1], (9, 54)),
        ("Sevilla", [1], (12, 55)),
        ("Insurgentes", [1], (15, 55)),
        ("Cuauhtemoc", [1], (18, 56)),
        ("Balderas", [1, 3], (21, 56)),
        ("Salto del Agua", [1], (24, 56)),
        ("Isabel la Catolica", [1], (27, 55)),
        ("Pino Suarez", [1, 2], (30, 55)),
        ("Merced", [1], (33, 54)),
        ("Candelaria", [1, 4], (36, 55)),
        ("San Lazaro", [1], (39, 56)),
        ("Moctezuma", [1], (42, 55)),
        ("Balbuena", [1], (45, 54)),
        ("Boulevard Puerto Aereo", [1], (48, 53)),
        ("Gomez Farias", [1], (51, 52)),
        ("Zaragoza", [1], (54, 51)),
        ("Pantitlan", [1, 5], (57, 50)),
    ]
    
    conexiones_l1 = [
        ("Observatorio", "Tacubaya", 2),
        ("Tacubaya", "Juanacatlan", 2),
        ("Juanacatlan", "Chapultepec", 2),
        ("Chapultepec", "Sevilla", 2),
        ("Sevilla", "Insurgentes", 2),
        ("Insurgentes", "Cuauhtemoc", 2),
        ("Cuauhtemoc", "Balderas", 2),
        ("Balderas", "Salto del Agua", 2),
        ("Salto del Agua", "Isabel la Catolica", 2),
        ("Isabel la Catolica", "Pino Suarez", 2),
        ("Pino Suarez", "Merced", 2),
        ("Merced", "Candelaria", 2),
        ("Candelaria", "San Lazaro", 2),
        ("San Lazaro", "Moctezuma", 2),
        ("Moctezuma", "Balbuena", 2),
        ("Balbuena", "Boulevard Puerto Aereo", 2),
        ("Boulevard Puerto Aereo", "Gomez Farias", 2),
        ("Gomez Farias", "Zaragoza", 2),
        ("Zaragoza", "Pantitlan", 2),
    ]
    
    # Linea 2 (AZUL) - Cuatro Caminos a Tasqueña 
    estaciones_l2 = [
        ("Cuatro Caminos", [2], (0, 80)),
        ("Panteones", [2], (3, 79)),
        ("Tacuba", [2], (7, 79)),
        ("Cuitlahuac", [2], (9, 76)),
        ("Popotla", [2], (11, 74)),
        ("Colegio Militar", [2], (13, 72)),
        ("Normal", [2], (15, 70)),
        ("San Cosme", [2], (17, 67)),
        ("Revolucion", [2], (19, 66)),
        ("Hidalgo", [2, 3], (21, 64)),
        ("Bellas Artes", [2], (24, 62)),
        ("Allende", [2], (26, 60)),
        ("Zocalo", [2], (27, 58)),
        # Pino Suarez ya existe en L1
        ("San Antonio Abad", [2], (31, 50)),
        ("Chabacano", [2], (31, 45)),
        ("Viaducto", [2], (31, 40)),
        ("Xola", [2], (31, 35)),
        ("Villa de Cortes", [2], (31, 30)),
        ("Nativitas", [2], (31, 25)),
        ("Portales", [2], (31, 20)),
        ("Ermita", [2], (31, 15)),
        ("General Anaya", [2], (31, 10)),
        ("Tasqueña", [2], (31, 5)),
    ]
    
    conexiones_l2 = [
        ("Cuatro Caminos", "Panteones", 2),
        ("Panteones", "Tacuba", 2),
        ("Tacuba", "Cuitlahuac", 2),
        ("Cuitlahuac", "Popotla", 2),
        ("Popotla", "Colegio Militar", 2),
        ("Colegio Militar", "Normal", 2),
        ("Normal", "San Cosme", 2),
        ("San Cosme", "Revolucion", 2),
        ("Revolucion", "Hidalgo", 2),
        ("Hidalgo", "Bellas Artes", 2),
        ("Bellas Artes", "Allende", 2),
        ("Allende", "Zocalo", 2),
        ("Zocalo", "Pino Suarez", 2),
        ("Pino Suarez", "San Antonio Abad", 2),
        ("San Antonio Abad", "Chabacano", 2),
        ("Chabacano", "Viaducto", 2),
        ("Viaducto", "Xola", 2),
        ("Xola", "Villa de Cortes", 2),
        ("Villa de Cortes", "Nativitas", 2),
        ("Nativitas", "Portales", 2),
        ("Portales", "Ermita", 2),
        ("Ermita", "General Anaya", 2),
        ("General Anaya", "Tasqueña", 2),
    ]
    
    # Linea 3 (VERDE) - Indios Verdes a Universidad
    estaciones_l3 = [
        ("Indios Verdes", [3], (36, 90)),
        ("Deportivo 18 de Marzo", [3], (34, 85)),
        ("Potrero", [3], (32, 80)),
        ("La Raza", [3, 5], (30, 75)),
        ("Tlatelolco", [3], (28, 70)),
        ("Guerrero", [3], (24, 67)),
        # Hidalgo L2
        ("Juarez", [3], (21, 60)),
        # Balderas L1
        ("Niños Heroes", [3], (21, 52)),
        ("Hospital General", [3], (19, 49)),
        ("Centro Medico", [3], (19, 46)),
        ("Etiopia", [3], (18, 41)),
        ("Eugenia", [3], (17, 36)),
        ("Division del Norte", [3], (16, 31)),
        ("Zapata", [3], (15, 26)),
        ("Coyoacan", [3], (14, 21)),
        ("Viveros", [3], (12, 16)),
        ("Miguel angel de Quevedo", [3], (13, 11)),
        ("Copilco", [3], (12, 6)),
        ("Universidad", [3], (11, 1)),
    ]
    
    conexiones_l3 = [
        ("Indios Verdes", "Deportivo 18 de Marzo", 2),
        ("Deportivo 18 de Marzo", "Potrero", 2),
        ("Potrero", "La Raza", 2),
        ("La Raza", "Tlatelolco", 2),
        ("Tlatelolco", "Guerrero", 2),
        ("Guerrero", "Hidalgo", 2),
        ("Hidalgo", "Juarez", 2),
        ("Juarez", "Balderas", 2),
        ("Balderas", "Niños Heroes", 2),
        ("Niños Heroes", "Hospital General", 2),
        ("Hospital General", "Centro Medico", 2),
        ("Centro Medico", "Etiopia", 2),
        ("Etiopia", "Eugenia", 2),
        ("Eugenia", "Division del Norte", 2),
        ("Division del Norte", "Zapata", 2),
        ("Zapata", "Coyoacan", 2),
        ("Coyoacan", "Viveros", 2),
        ("Viveros", "Miguel angel de Quevedo", 2),
        ("Miguel angel de Quevedo", "Copilco", 2),
        ("Copilco", "Universidad", 2),
    ]
    
    # Linea 4 (CIAN) - Martin Carrera a Santa Anita 
    estaciones_l4 = [
        ("Martin Carrera", [4], (42, 85)),
        ("Talisman", [4], (44, 80)),
        ("Bondojito", [4], (50, 73)),
        ("Consulado", [4, 5], (50, 69)),
        ("Canal del Norte", [4], (47, 64)),
        ("Morelos", [4], (44, 60)),
        # Candelaria L1
        ("Fray Servando", [4], (34, 50)),
        ("Jamaica", [4], (34, 45)),
        ("Santa Anita", [4], (34, 40)),
    ]
    
    conexiones_l4 = [
        ("Martin Carrera", "Talisman", 2),
        ("Talisman", "Bondojito", 2),
        ("Bondojito", "Consulado", 2),
        ("Consulado", "Canal del Norte", 2),
        ("Canal del Norte", "Morelos", 2),
        ("Morelos", "Candelaria", 2),
        ("Candelaria", "Fray Servando", 2),
        ("Fray Servando", "Jamaica", 2),
        ("Jamaica", "Santa Anita", 2),
    ]
    
    # Linea 5 (AMARILLA) - Politecnico a Pantitlan
    estaciones_l5 = [
        ("Politecnico", [5], (21, 90)),
        ("Instituto del Petroleo", [5], (24, 85)),
        ("Autobuses del Norte", [5], (27, 80)),
        # La Raza L3
        ("Misterios", [5], (35, 73)),
        ("tranquila Gomez", [5], (40, 71)),
        # Consulado L4
        ("Eduardo Molina", [5], (52, 65)),
        ("Aragon", [5], (53, 62)),
        ("Oceania", [5], (54, 59)),
        ("Terminal Aerea", [5], (55, 56)),
        ("Hangares", [5], (56, 53)),
        # Pantitlan L1
    ]
    
    conexiones_l5 = [
        ("Politecnico", "Instituto del Petroleo", 2),
        ("Instituto del Petroleo", "Autobuses del Norte", 2),
        ("Autobuses del Norte", "La Raza", 2),
        ("La Raza", "Misterios", 2),
        ("Misterios", "tranquila Gomez", 2),
        ("tranquila Gomez", "Consulado", 2),
        ("Consulado", "Eduardo Molina", 2),
        ("Eduardo Molina", "Aragon", 2),
        ("Aragon", "Oceania", 2),
        ("Oceania", "Terminal Aerea", 2),
        ("Terminal Aerea", "Hangares", 2),
        ("Hangares", "Pantitlan", 2),
    ]
    
    # Agregar todas las estaciones
    for estacion, lineas, coords in estaciones_l1:
        agregar_estacion(metro, estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l2:
        if estacion not in metro["estaciones"]:
            agregar_estacion(metro, estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l3:
        if estacion not in metro["estaciones"]:
            agregar_estacion(metro, estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l4:
        if estacion not in metro["estaciones"]:
            agregar_estacion(metro, estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l5:
        if estacion not in metro["estaciones"]:
            agregar_estacion(metro, estacion, lineas, coords)
    
    # Agregar conexiones
    for origen, destino, dist in conexiones_l1:
        agregar_conexion(metro, origen, destino, dist, 1)
    for origen, destino, dist in conexiones_l2:
        agregar_conexion(metro, origen, destino, dist, 2)
    for origen, destino, dist in conexiones_l3:
        agregar_conexion(metro, origen, destino, dist, 3)
    for origen, destino, dist in conexiones_l4:
        agregar_conexion(metro, origen, destino, dist, 4)
    for origen, destino, dist in conexiones_l5:
        agregar_conexion(metro, origen, destino, dist, 5)
    
    return metro


# Alias para mantener compatibilidad
def crear_metro_cdmx_simplificado():
    return crear_metro_cdmx_completo()


def func(inicio, destino, hora, prisa, accesibilidad):
    print("SISTEMA INTELIGENTE PARA EL METRO CDMX")
    print("Algoritmos: A* + Logica de Primer Orden")
    
    # Crear grafo del metro
    metro = crear_metro_cdmx_simplificado()
    print(f"\nGrafo creado: {len(metro['estaciones'])} estaciones")
    
    # PRIMERO: Aplicar logica de primer orden para modificar los costos
    contexto = aplicar_logica_primer_orden(metro, hora, prisa, accesibilidad)
    
    # SEGUNDO: Ejecutar A* con los costos ya modificados
    print("\n--- A* ---")
    print(f"Origen: {inicio}")
    print(f"Destino: {destino}")
    print(f"\nBuscando ruta con costos modificados...")
    print(f"  Costo estacion: {metro['costos']['estacion_normal']:.1f} min")
    print(f"  Costo transbordo: {metro['costos']['transbordo']:.1f} min")
    
    ruta, stats = a_star(metro, inicio, destino)
    
    if ruta:
        print(f"\nRuta encontrada:")
        print(f"  Estaciones: {' → '.join(ruta)}")
        print(f"  Longitud: {stats['longitud_ruta']} estaciones")
        print(f"  Tiempo estimado: {stats['costo_total']:.1f} minutos")
        print(f"  Nodos explorados: {stats['nodos_explorados']}")
        print(f"  Tiempo: {stats['tiempo_segundos']:.4f} segundos")
        print(f"  Eficiencia: {stats['eficiencia']:.2%}")
        
        # Visualizar
        print("\nGenerando visualizacion...")
        visualizar_grafo_metro(metro, ruta)
    else:
        print(f"\nNo se encontro ruta")
        print(f"  {stats}")


print("MENU - SISTEMA DE NAVEGACIoN DEL METRO CDMX")
print("Lineas disponibles: 1 (Rosa), 2 (Azul), 3 (Verde), 4 (Cian), 5 (Amarilla)")
while True: 
    linea = input("\nSelecciona la linea del metro (1-5) o 0 para salir: ")
    if linea in ['1', '2', '3', '4', '5']:
        print(f"\nHas seleccionado la linea {linea}. Estaciones disponibles:")
        metro = crear_metro_cdmx_simplificado()
        estaciones_linea = [nombre for nombre, est in metro["estaciones"].items() if int(linea) in est["lineas"]]
        print(", ".join(estaciones_linea))
        inicio = input("\nIngresa la estacion de origen: ")
        lineaD = input("Selecciona la linea de destino: ")
        if lineaD in ['1', '2', '3', '4', '5']:
            estaciones_lineaD = [nombre for nombre, est in metro["estaciones"].items() if int(lineaD) in est["lineas"]]
            print(f"\nHas seleccionado la linea {lineaD}. Estaciones disponibles:")
            print(", ".join(estaciones_lineaD))
            destino = input("Ingresa la estacion de destino: ")
            hora = int(input("Define la hora actual (0-23): "))
            prisa_input = input("¿Tienes prisa? (si/no): ").strip().lower()
            prisa = prisa_input == 'si'
            accesibilidad_input = input("¿Necesitas accesibilidad? (si/no): ").strip().lower()
            accesibilidad = accesibilidad_input == 'si'
            func(inicio, destino, hora, prisa, accesibilidad)
        else:
            print("Linea de destino no valida. Por favor, selecciona una linea entre 1 y 5.")
    elif linea == '0':
        print("\n¡Hasta luego!")
        break  
    else:
        print("Linea no valida. Por favor, selecciona una linea entre 1 y 5.")
