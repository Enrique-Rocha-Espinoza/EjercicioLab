# Lista de números enteros
Lista = [2, 4, 5, 8, 4, 3, 1, 19, 23, 29]

# Crear una lista de números pares y calcular su promedio
numeros_pares = [num for num in Lista if num % 2 == 0]  # Lista de números pares
promedio_pares = sum(numeros_pares) / len(numeros_pares) if numeros_pares else 0  # Promedio de números pares

# Crear una lista de números impares y calcular su producto
numeros_impares = [num for num in Lista if num % 2 != 0]  # Lista de números impares
producto_impares = 1  # Inicializar el producto de números impares
for num in numeros_impares:  # Calcular el producto de los números impares
    producto_impares *= num

# Imprimir los resultados
print(f"Promedio de los números pares: {promedio_pares}")
print(f"Producto de los números impares: {producto_impares}")
