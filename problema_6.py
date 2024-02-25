import random

def generarmatriz():
    n_filas, n_columnas = 5, 5  # Dimensiones de la cuadrícula
    inicio = (0, 0)  # Posición de inicio (S)
    fin = (4, 4)  # Posición de fin (E)
    obstaculo_char = 'X'
    libre_char = 'o'

    # Crear una cuadrícula con todos los espacios libres
    cuadricula = [[libre_char for _ in range(n_columnas)] for _ in range(n_filas)]
    cuadricula[inicio[0]][inicio[1]] = 'S'
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
    for fila in cuadricula:
        print(' '.join(fila))
    return cuadricula



if __name__ == "__main__":
    
    andn



