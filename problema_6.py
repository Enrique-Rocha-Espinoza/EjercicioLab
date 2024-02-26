import random
from collections import deque

def generarmatriz():
    n_filas, n_columnas = 5, 5  # Dimensiones de la cuadrícula
    inicio = (0, 0)  # Posición de inicio (S)
    fin = (4, 4)  # Posición de fin (E)
    obstaculo_char = 'X'
    libre_char = 'o'

    # Crear una cuadrícula con todos los espacios libres
    cuadricula = [[libre_char for _ in range(n_columnas)] for _ in range(n_filas)]
    #colocar el inicio
    cuadricula[inicio[0]][inicio[1]] = 'S'
    #colocar el final
    cuadricula[fin[0]][fin[1]] = 'E'

    # Generar posiciones aleatorias para los obstáculos
    obstaculos = set()
    while len(obstaculos) < n_filas:  # Limitar el número de obstáculos
        obstaculo = (random.randint(0, n_filas - 1), random.randint(0, n_columnas - 1))
        if obstaculo != inicio and obstaculo != fin:
            obstaculos.add(obstaculo)

    # Colocar los obstáculos en la cuadrícula
    for obstaculo in obstaculos:
        r, c = obstaculo
        cuadricula[r][c] = obstaculo_char

    # Imprimir la cuadrícula
    print("Este es el laberinto a resolver: ")
    for fila in cuadricula:
        print(' '.join(fila))
    return cuadricula

def Buscar_camino(cuadricula):
    # Obtener las dimensiones de la cuadrícula
    n_filas, n_columnas = len(cuadricula), len(cuadricula[0])
    
    # Definir las posiciones de inicio y fin
    inicio = (0, 0)
    fin = (n_filas - 1, n_columnas - 1)
    
    # Inicializar el conjunto de visitados y la cola para la búsqueda BFS
    visitados = set()
    cola = deque([((inicio, None), [inicio])])  # Cola para BFS, contiene tuplas de ((posición, dirección), camino)

    # Bucle para explorar la cuadrícula
    while cola:
        # Extraer la posición actual, la dirección actual y el camino actual de la cola
        (posicion_actual, direccion_actual), camino = cola.popleft()
        
        # Verificar si se ha alcanzado la posición final
        if posicion_actual == fin:
            return camino  # Devolver el camino encontrado

        # Iterar sobre las posibles direcciones de movimiento
        for direccion, simbolo in [((1, 0), '↓'), ((0, 1), '→'), ((-1, 0), '↑'), ((0, -1), '←')]:
            # Calcular la nueva posición basada en la dirección de movimiento
            nueva_posicion = (posicion_actual[0] + direccion[0], posicion_actual[1] + direccion[1])
            
            # Verificar si la nueva posición es válida y no ha sido visitada
            if 0 <= nueva_posicion[0] < n_filas and 0 <= nueva_posicion[1] < n_columnas and \
                    cuadricula[nueva_posicion[0]][nueva_posicion[1]] != 'X' and nueva_posicion not in visitados:
                # Marcar la nueva posición como visitada y añadirla a la cola con el camino actualizado
                visitados.add(nueva_posicion)
                cola.append(((nueva_posicion, simbolo), camino + [(nueva_posicion, simbolo)]))

    # Si no se encontró un camino, devolver un mensaje indicando que es imposible llegar al destino
    return "Imposible llegar al destino"

def imprimir_camino(cuadricula, camino):
    # Recorrer el camino y actualizar la cuadrícula con los símbolos de las direcciones
    for posicion, direccion in camino:
        if direccion:  # Ignorar la dirección None del inicio
            r, c = posicion
            cuadricula[r][c] = direccion

    # Imprimir la cuadrícula actualizada con el camino marcado
    for fila in cuadricula:
        print(' '.join(fila))

if __name__ == "__main__":
    # Generar la cuadrícula y buscar el camino
    cuadricula = generarmatriz()
    camino = Buscar_camino(cuadricula)
    
    # Imprimir el camino o el mensaje de que es imposible llegar al destino
    if isinstance(camino, list):  # Verificar que se encontró un camino
        print()
        print("Se encontro un camino cn exito")
        imprimir_camino(cuadricula, camino)
    else:
        print(camino)

