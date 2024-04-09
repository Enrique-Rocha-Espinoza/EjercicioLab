import random

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
    # Determinar el número de filas y columnas de la cuadrícula
    n_filas, n_columnas = len(cuadricula), len(cuadricula[0])
    
    # Establecer la posición inicial en la esquina superior izquierda y la final en la inferior derecha
    inicio = (0, 0)
    fin = (n_filas - 1, n_columnas - 1)
    
    # Direcciones de movimiento permitidas: abajo, derecha, izquierda
    direcciones = [(1, 0), (0, 1), (0, -1)]
    
    # Inicializar la posición actual en el inicio y el camino con la posición inicial
    posicion_actual = inicio
    camino = [inicio]

    # Bucle que se ejecuta hasta que la posición actual alcance la posición final
    while posicion_actual != fin:
        # Iterar a través de cada dirección de movimiento permitida
        for direccion in direcciones:
            # Calcular la nueva posición basada en la dirección actual
            nueva_posicion = (posicion_actual[0] + direccion[0], posicion_actual[1] + direccion[1])
            
            # Verificar si la nueva posición es válida (dentro de la cuadrícula y no es un obstáculo)
            if 0 <= nueva_posicion[0] < n_filas and 0 <= nueva_posicion[1] < n_columnas and cuadricula[nueva_posicion[0]][nueva_posicion[1]] != 'X':
                # Actualizar la posición actual a la nueva posición válida
                posicion_actual = nueva_posicion
                # Añadir la nueva posición al camino
                camino.append(nueva_posicion)
                # Romper el bucle for y continuar con el siguiente paso en el while
                break
        else:
            # Si después de intentar todas las direcciones no se puede avanzar, devolver que es imposible encontrar un camino
            return "Imposible llegar al destino"

    # Devolver el camino encontrado desde la posición inicial hasta la final
    return camino




def imprimir_cuadricula_con_camino(cuadricula, camino):
    # Crear una copia de la cuadrícula para no modificar la original
    cuadricula_con_camino = [fila[:] for fila in cuadricula]

    # Definir las direcciones con sus respectivas flechas
    direcciones = {
        (1, 0): '↓',   # Abajo
        (0, 1): '→',   # Derecha
        (0, -1): '←',  # Izquierda
        (-1, 0): '↑'   # Arriba
    }

    # Recorrer el camino y marcar las flechas en la cuadrícula
    for i in range(len(camino) - 1):
        posicion_actual = camino[i]
        posicion_siguiente = camino[i + 1]
        direccion = (posicion_siguiente[0] - posicion_actual[0], posicion_siguiente[1] - posicion_actual[1])
        cuadricula_con_camino[posicion_actual[0]][posicion_actual[1]] = direcciones[direccion]

    # Imprimir la cuadrícula con el camino
    for fila in cuadricula_con_camino:
        print(' '.join(fila))



if __name__ == "__main__":

    cuadricula = generarmatriz()
    camino = Buscar_camino(cuadricula)

    if isinstance(camino, list):  # Verificar si se encontró un camino
        print("Camino encontrado:")
        imprimir_cuadricula_con_camino(cuadricula, camino)
    else:
        print(camino)  # Imprimir "Imposible llegar al destino"

    

