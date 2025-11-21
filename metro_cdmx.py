## Metro - CDMX
import time
import heapq
from collections import defaultdict
from typing import Dict, List, Tuple, Set, Optional
import matplotlib.pyplot as plt
import networkx as nx

# ==================== GRAFO ====================

#Estacion
class EstacionMetro:
    def __init__(self, nombre: str, lineas: List[int], coordenadas: Tuple[float, float] = None):
        self.nombre = nombre
        self.lineas = lineas  # Lista de líneas que pasan por esta estación
        self.coordenadas = coordenadas or (0, 0)
        self.es_transbordo = len(lineas) > 1
        
    def __repr__(self):
        return f"Estación({self.nombre}, L{self.lineas})"
    
    def __hash__(self):
        return hash(self.nombre)
    
    def __eq__(self, other):
        return self.nombre == other.nombre if isinstance(other, EstacionMetro) else False


#Grafo
class GrafoMetro:
    def __init__(self):
        self.estaciones: Dict[str, EstacionMetro] = {}
        self.conexiones: Dict[str, List[Tuple[str, float, int]]] = defaultdict(list)
        # conexiones[origen] = [(destino, distancia_km, linea), ...]
        
        # Costos base (inspirados en tu sistema de terrenos)
        self.COSTO_ESTACION_NORMAL = 1.0 # Costo por estación normal
        self.COSTO_TRANSBORDO = 10.0 # Costo por transbordo (MUY PESADO para evitar cambios innecesarios)
        self.COSTO_KM = 0.3 # Costo por kilómetro
        
    def agregar_estacion(self, nombre: str, lineas: List[int], coords: Tuple[float, float] = None):
        #Agregar estación
        self.estaciones[nombre] = EstacionMetro(nombre, lineas, coords)
        
    def agregar_conexion(self, origen: str, destino: str, distancia_km: float, linea: int):
        #Agregar conexión en las dos direcciones
        self.conexiones[origen].append((destino, distancia_km, linea))
        self.conexiones[destino].append((origen, distancia_km, linea))
        
    def obtener_vecinos(self, estacion_nombre: str) -> List[Tuple[str, float, int]]:
        #Retorna lista de vecinos: (nombre_estacion, distancia, linea)
        return self.conexiones.get(estacion_nombre, [])
    
    def calcular_costo_movimiento(self, origen: str, destino: str, linea_actual: int) -> float:
        estacion_destino = self.estaciones[destino]
        
        # Encontrar la conexión específica
        distancia = 1.0
        for vecino, dist, linea in self.conexiones[origen]:
            if vecino == destino:
                distancia = dist
                break
        
        # Costo base por distancia
        costo = self.COSTO_ESTACION_NORMAL + (self.COSTO_KM * distancia)
        
        # PENALIZACIÓN POR TRANSBORDO
        if estacion_destino.es_transbordo and linea_actual not in estacion_destino.lineas:
            costo += self.COSTO_TRANSBORDO
            
        return costo
    
    def heuristica_linea_recta(self, origen: str, destino: str) -> float:
        #Euclidiana
        coord_o = self.estaciones[origen].coordenadas
        coord_d = self.estaciones[destino].coordenadas
        return ((coord_o[0] - coord_d[0])**2 + (coord_o[1] - coord_d[1])**2)**0.5


# A* 

def a_star_metro(grafo: GrafoMetro, inicio: str, destino: str, linea_inicial: int = None) -> Tuple[Optional[List[str]], Dict]:
    #
    # A* para encontrar la ruta óptima en el metro.
    # Adaptado de tu implementación manual con open_list y costos de terreno.
    #
    # Returns:
    #     (ruta, estadisticas) donde ruta es lista de nombres de estaciones
    #
    
    if inicio not in grafo.estaciones or destino not in grafo.estaciones:
        return None, {"error": "Estación no encontrada"}
    
    # Determinar línea inicial
    if linea_inicial is None:
        linea_inicial = grafo.estaciones[inicio].lineas[0]
    
    start_time = time.time()
    
    # Open list: (f_cost, g_cost, estacion_nombre, linea_actual, camino)
    open_list = []
    h_inicial = grafo.heuristica_linea_recta(inicio, destino)
    heapq.heappush(open_list, (h_inicial, 0.0, inicio, linea_inicial, [inicio]))
    
    # Diccionarios de control
    g_costs = {(inicio, linea_inicial): 0.0}
    visitados = set()
    nodos_explorados = 0
    
    while open_list:
        f_cost, g_cost, estacion_actual, linea_actual, camino = heapq.heappop(open_list)
        
        estado = (estacion_actual, linea_actual)
        
        if estado in visitados:
            continue
            
        visitados.add(estado)
        nodos_explorados += 1
        
        # ¿Llegamos al destino?
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
        
        # Explorar vecinos
        for vecino, distancia, linea_conexion in grafo.obtener_vecinos(estacion_actual):
            
            # Calcular costo del movimiento (considerando transbordos)
            costo_movimiento = grafo.calcular_costo_movimiento(
                estacion_actual, vecino, linea_actual
            )
            nuevo_g = g_cost + costo_movimiento
            
            estado_vecino = (vecino, linea_conexion)
            
            # Si encontramos un camino mejor
            if estado_vecino not in g_costs or nuevo_g < g_costs[estado_vecino]:
                g_costs[estado_vecino] = nuevo_g
                h_cost = grafo.heuristica_linea_recta(vecino, destino)
                f_cost = nuevo_g + h_cost
                
                nuevo_camino = camino + [vecino]
                heapq.heappush(open_list, (f_cost, nuevo_g, vecino, linea_conexion, nuevo_camino))
    
    # No se encontró ruta
    tiempo_total = time.time() - start_time
    return None, {
        "error": "No se encontró ruta",
        "nodos_explorados": nodos_explorados,
        "tiempo_segundos": tiempo_total
    }


# =============================================================================
# LÓGICA DE PRIMER ORDEN
# =============================================================================

# ===== BASE DE CONOCIMIENTO =====

hora_pico_hechos = [
    ("es_hora_pico", 7, 9),    # Mañana
    ("es_hora_pico", 18, 20),  # Tarde
]

hora_valle_hechos = [
    ("es_hora_valle", 10, 17),  # Media día
    ("es_hora_valle", 21, 6),   # Noche/madrugada
]

# Hechos sobre modificadores de costo: (condicion, tipo_costo, multiplicador)
costo_hechos = [
    ("hora_pico", "estacion_normal", 1.3),
    ("hora_pico", "transbordo", 2.0),  # Más pesado en hora pico
    ("hora_valle", "estacion_normal", 0.9),
    ("hora_valle", "transbordo", 1.0),  # Neutro en hora valle
    ("prisa", "transbordo", 3.0),  # MUY pesado si tienes prisa
    ("prisa", "km", 0.8),
    ("accesibilidad", "km", 0.5),
    ("accesibilidad", "transbordo", 1.5),
]

# Hechos sobre preferencias
preferencia_hechos = [
    ("prisa", "minimizar_transbordos", True),
    ("accesibilidad", "minimizar_distancia", True),
    ("hora_pico", "evitar_aglomeraciones", True),
]

# Inferir conflictos: condiciones que no pueden coexistir
conflictos_pairs = [
    ("hora_pico", "hora_valle"),
]

# Inferir combinaciones: cuando dos condiciones juntas amplifican el efecto
combinaciones_pairs = [
    ("prisa", "hora_pico", "urgencia_extrema", 2.2),  # condicion1, condicion2, nombre_resultado, multiplicador_extra
    ("accesibilidad", "hora_valle", "viaje_comodo", 0.7),
]

# ===== FUNCIONES DE INFERENCIA =====

def es_hora_pico(hora):
    #hora pico
    for condicion, inicio, fin in hora_pico_hechos:
        if inicio <= hora <= fin:
            return True
    return False

def es_hora_valle(hora):
   #hora valle
    for condicion, inicio, fin in hora_valle_hechos:
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

#pruebas, se va a comentar
def explicar_condicion(condicion, hora=None):
    """Genera explicación de por qué se aplica una condición"""
    if condicion == "hora_pico" and hora is not None:
        for cond, inicio, fin in hora_pico_hechos:
            if inicio <= hora <= fin:
                return f"Es hora pico porque la hora {hora} está entre {inicio}:00 y {fin}:00 (hecho: hora_pico({inicio},{fin}))"
    elif condicion == "hora_valle" and hora is not None:
        for cond, inicio, fin in hora_valle_hechos:
            if inicio <= fin and inicio <= hora <= fin:
                return f"Es hora valle porque la hora {hora} está entre {inicio}:00 y {fin}:00 (hecho: hora_valle({inicio},{fin}))"
    elif condicion == "prisa":
        return "Usuario tiene prisa según entrada (hecho: prisa(usuario))"
    elif condicion == "accesibilidad":
        return "Usuario requiere accesibilidad según entrada (hecho: accesibilidad(usuario))"
    return f"Condición {condicion} activa."

def explicar_modificador(condicion, tipo_costo, multiplicador):
    """Explica por qué se aplica un modificador de costo"""
    return f"Modificador aplicado: {condicion} afecta '{tipo_costo}' con factor {multiplicador} (hecho: costo({condicion},{tipo_costo},{multiplicador}))"

def explicar_preferencia(condicion, accion):
    """Explica una preferencia derivada de la condición"""
    return f"Preferencia: {condicion} implica {accion} (hecho: preferencia({condicion},{accion}))"

# ===== MOTOR DE INFERENCIA =====

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
    elif es_hora_valle(hora):
        condiciones_activas.append("hora_valle")
        explicaciones.append(explicar_condicion("hora_valle", hora))
    
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
        explicaciones.append(f"¡Combinación especial detectada: {nombre_comb}! (multiplicador adicional: {mult_extra})")
        modificadores[nombre_comb] = mult_extra
    
    # DERIVACIÓN: Aplicar modificadores de costo
    for condicion in condiciones_activas:
        # Buscar todos los modificadores para esta condición
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
    """
    Aplica la lógica de primer orden al grafo del metro.
    Estilo similar a outcome() y explain_outcome() del juego.
    """
    print("\n" + "="*70)
    print("SISTEMA DE LÓGICA DE PRIMER ORDEN - BASE DE CONOCIMIENTO")
    print("="*70)
    
    # Mostrar hechos base
    print("\n[Hechos Base - Horarios]")
    for cond, inicio, fin in hora_pico_hechos:
        print(f"  • hora_pico({inicio}, {fin})")
    for cond, inicio, fin in hora_valle_hechos:
        print(f"  • hora_valle({inicio}, {fin})")
    
    print("\n[Hechos Base - Modificadores de Costo]")
    for cond, tipo, mult in sorted(costo_hechos):
        print(f"  • costo({cond}, {tipo}, {mult})")
    
    print("\n[Hechos Base - Preferencias]")
    for cond, accion, valor in preferencia_hechos:
        print(f"  • preferencia({cond}, {accion}, {valor})")
    
    # Ejecutar motor de inferencia
    print("\n" + "="*70)
    print("MOTOR DE INFERENCIA - Derivando Consecuencias")
    print("="*70)
    print(f"\nEntrada: hora={hora}, prisa={prisa}, accesibilidad={accesibilidad}")
    
    contexto = inferir_contexto(hora, prisa, accesibilidad)
    
    print(f"\n[Condiciones Activas Inferidas]: {contexto['condiciones_activas']}")
    
    print("\n[Explicaciones - Cadena de Inferencia]:")
    for i, exp in enumerate(contexto['explicaciones'], 1):
        print(f"  {i}. {exp}")
    
    print("\n[Modificadores Derivados]:")
    for tipo, mult in contexto['modificadores'].items():
        print(f"  • {tipo}: x{mult:.2f}")
    
    if contexto['preferencias']:
        print("\n[Preferencias Activas]:")
        for pref in contexto['preferencias']:
            print(f"  • {pref}")
    
    # Aplicar modificadores al grafo
    print("\n[Aplicando Modificadores al Grafo del Metro]")
    if "estacion_normal" in contexto['modificadores']:
        grafo.COSTO_ESTACION_NORMAL *= contexto['modificadores']['estacion_normal']
        print(f"  ✓ Costo estación normal: {grafo.COSTO_ESTACION_NORMAL:.2f}")
    
    if "transbordo" in contexto['modificadores']:
        grafo.COSTO_TRANSBORDO *= contexto['modificadores']['transbordo']
        print(f"  ✓ Costo transbordo: {grafo.COSTO_TRANSBORDO:.2f}")
    
    if "km" in contexto['modificadores']:
        grafo.COSTO_KM *= contexto['modificadores']['km']
        print(f"  ✓ Costo por km: {grafo.COSTO_KM:.2f}")
    
    print("\n" + "="*70)
    
    return contexto


# =============================================================================
# 5. VISUALIZACIÓN INTERACTIVA
# =============================================================================

def visualizar_grafo_metro(grafo: GrafoMetro, ruta: List[str] = None):
    """Visualiza el grafo del metro con la ruta encontrada - Versión SIMPLE"""
    
    G = nx.Graph()
    pos = {}
    
    # Agregar nodos con posiciones
    for nombre, estacion in grafo.estaciones.items():
        G.add_node(nombre)
        pos[nombre] = estacion.coordenadas
    
    # Agregar aristas
    for origen, conexiones in grafo.conexiones.items():
        for destino, distancia, linea in conexiones:
            G.add_edge(origen, destino, linea=linea, distancia=distancia)
    
    # Configurar figura
    plt.figure(figsize=(16, 12))
    
    # Dibujar todas las conexiones en gris claro
    nx.draw_networkx_edges(G, pos, alpha=0.2, width=1, edge_color='gray')
    
    # Dibujar la ruta en verde si existe
    if ruta and len(ruta) > 1:
        ruta_edges = [(ruta[i], ruta[i+1]) for i in range(len(ruta)-1)]
        nx.draw_networkx_edges(G, pos, edgelist=ruta_edges, 
                              edge_color='green', width=4, alpha=0.8)
    
    # Dibujar nodos
    nodos_transbordo = [n for n, e in grafo.estaciones.items() if e.es_transbordo]
    nodos_normales = [n for n, e in grafo.estaciones.items() if not e.es_transbordo]
    
    # Nodos normales
    nx.draw_networkx_nodes(G, pos, nodelist=nodos_normales,
                          node_color='lightblue', node_size=300, alpha=0.8)
    
    # Nodos de transbordo
    nx.draw_networkx_nodes(G, pos, nodelist=nodos_transbordo,
                          node_color='red', node_size=400, alpha=0.8)
    
    # Resaltar origen y destino si hay ruta
    if ruta:
        nx.draw_networkx_nodes(G, pos, nodelist=[ruta[0]],
                              node_color='green', node_size=500, alpha=1)
        nx.draw_networkx_nodes(G, pos, nodelist=[ruta[-1]],
                              node_color='darkred', node_size=500, alpha=1)
    
    # Etiquetas - mostrar TODOS los nombres
    labels = {nombre: nombre for nombre in grafo.estaciones.keys()}
    nx.draw_networkx_labels(G, pos, labels, font_size=6, font_weight='bold')
    
    # Título
    if ruta:
        plt.title(f"Metro CDMX - Ruta: {ruta[0]} → {ruta[-1]} ({len(ruta)} estaciones)",
                 fontsize=14, fontweight='bold')
    else:
        plt.title("Red del Metro CDMX (5 Líneas)", fontsize=14, fontweight='bold')
    
    plt.axis('off')
    plt.tight_layout()
    plt.show()


# =============================================================================
# 6. EJEMPLO DE USO Y PRUEBA
# =============================================================================

def crear_metro_cdmx_completo() -> GrafoMetro:
    """
    Metro CDMX - Primeras 5 Líneas
    Líneas 1, 2, 3, 4, 5 completas para pruebas
    """
    metro = GrafoMetro()
    
    # ========== LÍNEA 1 (ROSA) - Observatorio ↔ Pantitlán ==========
    estaciones_l1 = [
        ("Observatorio", [1], (0, 50)),
        ("Tacubaya", [1], (3, 50)),
        ("Juanacatlán", [1], (6, 50)),
        ("Chapultepec", [1], (9, 50)),
        ("Sevilla", [1], (12, 50)),
        ("Insurgentes", [1], (15, 50)),
        ("Cuauhtémoc", [1], (18, 50)),
        ("Balderas", [1, 3], (21, 50)),
        ("Salto del Agua", [1], (24, 50)),
        ("Isabel la Católica", [1], (27, 50)),
        ("Pino Suárez", [1, 2], (30, 50)),
        ("Merced", [1], (33, 50)),
        ("Candelaria", [1, 4], (36, 50)),
        ("San Lázaro", [1], (39, 50)),
        ("Moctezuma", [1], (42, 50)),
        ("Balbuena", [1], (45, 50)),
        ("Boulevard Puerto Aéreo", [1], (48, 50)),
        ("Gómez Farías", [1], (51, 50)),
        ("Zaragoza", [1], (54, 50)),
        ("Pantitlán", [1, 5], (57, 50)),
    ]
    
    conexiones_l1 = [
        ("Observatorio", "Tacubaya", 1.15),
        ("Tacubaya", "Juanacatlán", 1.04),
        ("Juanacatlán", "Chapultepec", 0.88),
        ("Chapultepec", "Sevilla", 0.75),
        ("Sevilla", "Insurgentes", 0.85),
        ("Insurgentes", "Cuauhtémoc", 0.82),
        ("Cuauhtémoc", "Balderas", 0.79),
        ("Balderas", "Salto del Agua", 0.71),
        ("Salto del Agua", "Isabel la Católica", 0.64),
        ("Isabel la Católica", "Pino Suárez", 0.68),
        ("Pino Suárez", "Merced", 0.75),
        ("Merced", "Candelaria", 0.82),
        ("Candelaria", "San Lázaro", 0.78),
        ("San Lázaro", "Moctezuma", 1.05),
        ("Moctezuma", "Balbuena", 1.18),
        ("Balbuena", "Boulevard Puerto Aéreo", 0.99),
        ("Boulevard Puerto Aéreo", "Gómez Farías", 0.87),
        ("Gómez Farías", "Zaragoza", 1.07),
        ("Zaragoza", "Pantitlán", 1.43),
    ]
    
    # ========== LÍNEA 2 (AZUL) - Cuatro Caminos ↔ Tasqueña ==========
    estaciones_l2 = [
        ("Cuatro Caminos", [2], (0, 40)),
        ("Panteones", [2], (3, 40)),
        ("Tacuba", [2], (6, 40)),
        ("Cuitláhuac", [2], (9, 40)),
        ("Popotla", [2], (12, 40)),
        ("Colegio Militar", [2], (15, 40)),
        ("Normal", [2], (18, 40)),
        ("San Cosme", [2], (21, 40)),
        ("Revolución", [2], (24, 40)),
        ("Hidalgo", [2, 3], (27, 40)),
        ("Bellas Artes", [2], (30, 40)),
        ("Allende", [2], (33, 40)),
        ("Zócalo", [2], (36, 40)),
        # Pino Suárez ya existe en L1
        ("San Antonio Abad", [2], (42, 40)),
        ("Chabacano", [2], (45, 40)),
        ("Viaducto", [2], (48, 40)),
        ("Xola", [2], (51, 40)),
        ("Villa de Cortés", [2], (54, 40)),
        ("Nativitas", [2], (57, 40)),
        ("Portales", [2], (60, 40)),
        ("Ermita", [2], (63, 40)),
        ("General Anaya", [2], (66, 40)),
        ("Tasqueña", [2], (69, 40)),
    ]
    
    conexiones_l2 = [
        ("Cuatro Caminos", "Panteones", 1.02),
        ("Panteones", "Tacuba", 1.13),
        ("Tacuba", "Cuitláhuac", 0.84),
        ("Cuitláhuac", "Popotla", 0.89),
        ("Popotla", "Colegio Militar", 0.76),
        ("Colegio Militar", "Normal", 0.92),
        ("Normal", "San Cosme", 0.88),
        ("San Cosme", "Revolución", 0.95),
        ("Revolución", "Hidalgo", 0.73),
        ("Hidalgo", "Bellas Artes", 0.69),
        ("Bellas Artes", "Allende", 0.65),
        ("Allende", "Zócalo", 0.72),
        ("Zócalo", "Pino Suárez", 0.68),
        ("Pino Suárez", "San Antonio Abad", 0.98),
        ("San Antonio Abad", "Chabacano", 0.85),
        ("Chabacano", "Viaducto", 0.79),
        ("Viaducto", "Xola", 0.88),
        ("Xola", "Villa de Cortés", 0.91),
        ("Villa de Cortés", "Nativitas", 0.87),
        ("Nativitas", "Portales", 0.84),
        ("Portales", "Ermita", 0.93),
        ("Ermita", "General Anaya", 0.96),
        ("General Anaya", "Tasqueña", 1.14),
    ]
    
    # ========== LÍNEA 3 (VERDE) - Indios Verdes ↔ Universidad ==========
    estaciones_l3 = [
        ("Indios Verdes", [3], (27, 0)),
        ("Deportivo 18 de Marzo", [3], (27, 5)),
        ("Potrero", [3], (27, 10)),
        ("La Raza", [3, 5], (27, 15)),
        ("Tlatelolco", [3], (27, 20)),
        # Hidalgo ya existe en L2
        ("Juárez", [3], (27, 30)),
        # Balderas ya existe en L1
        ("Niños Héroes", [3], (27, 40)),
        ("Hospital General", [3], (27, 45)),
        ("Centro Médico", [3], (27, 50)),
        ("Etiopía", [3], (27, 55)),
        ("Eugenia", [3], (27, 60)),
        ("División del Norte", [3], (27, 65)),
        ("Zapata", [3], (27, 70)),
        ("Coyoacán", [3], (27, 75)),
        ("Viveros", [3], (27, 80)),
        ("Miguel Ángel de Quevedo", [3], (27, 85)),
        ("Copilco", [3], (27, 90)),
        ("Universidad", [3], (27, 95)),
    ]
    
    conexiones_l3 = [
        ("Indios Verdes", "Deportivo 18 de Marzo", 1.27),
        ("Deportivo 18 de Marzo", "Potrero", 0.92),
        ("Potrero", "La Raza", 0.96),
        ("La Raza", "Tlatelolco", 1.08),
        ("Tlatelolco", "Hidalgo", 0.94),
        ("Hidalgo", "Juárez", 0.71),
        ("Juárez", "Balderas", 0.66),
        ("Balderas", "Niños Héroes", 0.88),
        ("Niños Héroes", "Hospital General", 0.75),
        ("Hospital General", "Centro Médico", 0.82),
        ("Centro Médico", "Etiopía", 0.79),
        ("Etiopía", "Eugenia", 0.86),
        ("Eugenia", "División del Norte", 0.91),
        ("División del Norte", "Zapata", 0.77),
        ("Zapata", "Coyoacán", 1.12),
        ("Coyoacán", "Viveros", 0.89),
        ("Viveros", "Miguel Ángel de Quevedo", 0.94),
        ("Miguel Ángel de Quevedo", "Copilco", 1.05),
        ("Copilco", "Universidad", 1.38),
    ]
    
    # ========== LÍNEA 4 (CIAN) - Martín Carrera ↔ Santa Anita ==========
    estaciones_l4 = [
        ("Martín Carrera", [4], (50, 0)),
        ("Talismán", [4], (50, 5)),
        ("Bondojito", [4], (50, 10)),
        ("Consulado", [4, 5], (50, 15)),
        ("Canal del Norte", [4], (50, 20)),
        # Candelaria ya existe en L1
        ("Fray Servando", [4], (50, 30)),
        ("Jamaica", [4], (50, 35)),
        ("Santa Anita", [4], (50, 40)),
    ]
    
    conexiones_l4 = [
        ("Martín Carrera", "Talismán", 1.18),
        ("Talismán", "Bondojito", 0.87),
        ("Bondojito", "Consulado", 0.94),
        ("Consulado", "Canal del Norte", 1.03),
        ("Canal del Norte", "Candelaria", 0.96),
        ("Candelaria", "Fray Servando", 0.89),
        ("Fray Servando", "Jamaica", 0.92),
        ("Jamaica", "Santa Anita", 1.07),
    ]
    
    # ========== LÍNEA 5 (AMARILLA) - Politécnico ↔ Pantitlán ==========
    estaciones_l5 = [
        ("Politécnico", [5], (15, 15)),
        ("Instituto del Petróleo", [5], (18, 18)),
        ("Autobuses del Norte", [5], (21, 21)),
        # La Raza ya existe en L3
        ("Misterios", [5], (27, 27)),
        ("Valle Gómez", [5], (30, 30)),
        # Consulado ya existe en L4
        ("Eduardo Molina", [5], (36, 36)),
        ("Aragón", [5], (39, 39)),
        ("Oceanía", [5], (42, 42)),
        ("Terminal Aérea", [5], (45, 45)),
        ("Hangares", [5], (48, 48)),
        # Pantitlán ya existe en L1
    ]
    
    conexiones_l5 = [
        ("Politécnico", "Instituto del Petróleo", 0.92),
        ("Instituto del Petróleo", "Autobuses del Norte", 1.15),
        ("Autobuses del Norte", "La Raza", 1.23),
        ("La Raza", "Misterios", 0.88),
        ("Misterios", "Valle Gómez", 0.96),
        ("Valle Gómez", "Consulado", 1.02),
        ("Consulado", "Eduardo Molina", 1.08),
        ("Eduardo Molina", "Aragón", 0.94),
        ("Aragón", "Oceanía", 1.17),
        ("Oceanía", "Terminal Aérea", 1.05),
        ("Terminal Aérea", "Hangares", 0.99),
        ("Hangares", "Pantitlán", 1.32),
    ]
    
    # Agregar todas las estaciones
    for estacion, lineas, coords in estaciones_l1:
        metro.agregar_estacion(estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l2:
        if estacion not in metro.estaciones:
            metro.agregar_estacion(estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l3:
        if estacion not in metro.estaciones:
            metro.agregar_estacion(estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l4:
        if estacion not in metro.estaciones:
            metro.agregar_estacion(estacion, lineas, coords)
    for estacion, lineas, coords in estaciones_l5:
        if estacion not in metro.estaciones:
            metro.agregar_estacion(estacion, lineas, coords)
    
    # Agregar conexiones
    for origen, destino, dist in conexiones_l1:
        metro.agregar_conexion(origen, destino, dist, 1)
    for origen, destino, dist in conexiones_l2:
        metro.agregar_conexion(origen, destino, dist, 2)
    for origen, destino, dist in conexiones_l3:
        metro.agregar_conexion(origen, destino, dist, 3)
    for origen, destino, dist in conexiones_l4:
        metro.agregar_conexion(origen, destino, dist, 4)
    for origen, destino, dist in conexiones_l5:
        metro.agregar_conexion(origen, destino, dist, 5)
    
    return metro


# Alias para mantener compatibilidad
def crear_metro_cdmx_simplificado():
    return crear_metro_cdmx_completo()


def func(inicio, destino, hora, prisa, accesibilidad):
    """Función principal de prueba"""
    print("="*70)
    print("SISTEMA INTELIGENTE PARA EL METRO CDMX")
    print("Algoritmos: A* + Lógica de Primer Orden + [Por definir]")
    print("="*70)
    
    # Crear grafo del metro
    metro = crear_metro_cdmx_simplificado()
    print(f"\n✓ Grafo creado: {len(metro.estaciones)} estaciones")
    
    # Aplicar lógica de primer orden (estilo piedra-papel-tijera)
    contexto = aplicar_logica_primer_orden(metro, hora, prisa, accesibilidad)
    
    # Ejecutar A*
    print("\n--- Ejecutando A* ---")
    
    print(f"Origen: {inicio}")
    print(f"Destino: {destino}")
    
    ruta, stats = a_star_metro(metro, inicio, destino)
    
    if ruta:
        print(f"\n✓ Ruta encontrada:")
        print(f"  Estaciones: {' → '.join(ruta)}")
        print(f"  Longitud: {stats['longitud_ruta']} estaciones")
        print(f"  Costo total: {stats['costo_total']:.2f}")
        print(f"  Nodos explorados: {stats['nodos_explorados']}")
        print(f"  Tiempo: {stats['tiempo_segundos']:.4f} segundos")
        print(f"  Eficiencia: {stats['eficiencia']:.2%}")
        
        # Visualizar
        print("\n[Generando visualización...]")
        visualizar_grafo_metro(metro, ruta)
    else:
        print(f"\n✗ No se encontró ruta")
        print(f"  {stats}")
    
    print("\n" + "="*70)
    print("PROYECTO BASE COMPLETADO")
    print("Siguiente paso: Expandir con datos reales y elegir 3er algoritmo")
    print("="*70)



print("MENU - SISTEMA DE NAVEGACIÓN DEL METRO CDMX")
print("Líneas disponibles: 1 (Rosa), 2 (Azul), 3 (Verde), 4 (Cian), 5 (Amarilla)")
while True: 
    linea = input("\nSelecciona la línea del metro (1-5) o 0 para salir: ")
    if linea in ['1', '2', '3', '4', '5']:
        print(f"\n✓ Has seleccionado la línea {linea}. Estaciones disponibles:")
        metro = crear_metro_cdmx_simplificado()
        estaciones_linea = [est.nombre for est in metro.estaciones.values() if int(linea) in est.lineas]
        print(", ".join(estaciones_linea))
        inicio = input("\nIngresa la estación de origen: ")
        destino = input("Ingresa la estación de destino: ")
        hora = int(input("Define la hora actual (0-23): "))
        prisa_input = input("¿Tienes prisa? (si/no): ").strip().lower()
        prisa = prisa_input == 'si'
        accesibilidad_input = input("¿Necesitas accesibilidad? (si/no): ").strip().lower()
        accesibilidad = accesibilidad_input == 'si'
        func(inicio, destino, hora, prisa, accesibilidad)
    elif linea == '0':
        print("\n¡Hasta luego!")
        break  
    else:
        print("❌ Línea no válida. Por favor, selecciona una línea entre 1 y 5.")
